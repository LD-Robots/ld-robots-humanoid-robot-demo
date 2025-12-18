#!/usr/bin/env python3
"""
PPO trainer for MuJoCo humanoid model (robot.xml) with vectorized rollouts.
Runs multiple envs in parallel in a single process and auto-resets on fall.
"""

from __future__ import annotations

import argparse
import math
import os
import time
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

try:
    import mujoco
    try:
        import mujoco.viewer
        MUJOCO_VIEWER_AVAILABLE = True
    except Exception:
        MUJOCO_VIEWER_AVAILABLE = False
except ImportError as exc:
    raise SystemExit("MuJoCo not installed. Run: pip install mujoco") from exc

import torch
import torch.nn as nn


@dataclass
class PPOConfig:
    num_envs: int = 32
    total_steps: int = 2_000_000
    rollout_steps: int = 1024
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_ratio: float = 0.2
    ent_coef: float = 0.01
    vf_coef: float = 0.5
    lr: float = 3e-4
    max_grad_norm: float = 0.5
    update_epochs: int = 10
    minibatch_size: int = 2048


class ActorCritic(nn.Module):
    def __init__(self, obs_dim: int, act_dim: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 256),
            nn.Tanh(),
            nn.Linear(256, 256),
            nn.Tanh(),
        )
        self.mu = nn.Linear(256, act_dim)
        self.v = nn.Linear(256, 1)
        self.log_std = nn.Parameter(torch.zeros(act_dim))

    def forward(self, obs: torch.Tensor):
        h = self.net(obs)
        mu = self.mu(h)
        v = self.v(h).squeeze(-1)
        return mu, v

    def sample_action(self, obs: torch.Tensor):
        mu, v = self(obs)
        std = self.log_std.exp()
        dist = torch.distributions.Normal(mu, std)
        raw = dist.rsample()
        action = torch.tanh(raw)
        log_prob = dist.log_prob(raw) - torch.log(1.0 - action.pow(2) + 1e-6)
        log_prob = log_prob.sum(-1)
        return action, log_prob, v

    @staticmethod
    def log_prob_from_action(mu: torch.Tensor, log_std: torch.Tensor, action: torch.Tensor):
        std = log_std.exp()
        eps = 1e-6
        raw = 0.5 * torch.log((1.0 + action + eps) / (1.0 - action + eps))
        dist = torch.distributions.Normal(mu, std)
        log_prob = dist.log_prob(raw) - torch.log(1.0 - action.pow(2) + eps)
        return log_prob.sum(-1)


class HumanoidBatchEnv:
    def __init__(
        self,
        model_path: str,
        num_envs: int,
        fall_z: float = 0.6,
        fall_angle_deg: float = 45.0,
        frame_skip: int = 5,
        reward_weights: dict = None,
        target_speed: float = 1.2,
        phase_rate_hz: float = 1.6,
        foot_slip_threshold: float = 0.5,
    ):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.num_envs = num_envs
        self.data = [mujoco.MjData(self.model) for _ in range(num_envs)]
        self.fall_z = float(fall_z)
        self.fall_angle = math.radians(fall_angle_deg)
        self.frame_skip = int(frame_skip)
        self.target_speed = float(target_speed)
        self.phase_rate = 2.0 * math.pi * float(phase_rate_hz)
        self.foot_slip_threshold = float(foot_slip_threshold)

        # Reward function weights
        self.rw = reward_weights or {
            'survival': 1.0,
            'forward_vel': 2.5,
            'vel_cap': 1.5,
            'roll': 0.5,
            'pitch': 0.5,
            'pitch_threshold': math.radians(14.0),  # threshold angle for extra penalty
            'pitch_extra': 2.0,  # extra penalty above threshold
            'height': 1.0,
            'lateral': 0.3,
            'energy': 0.005,
            'target_speed': 2.0,  # penalty weight for deviation from target walking speed
            'fall': 5.0,
            'arms_high': 1.0,  # penalty for arms raised too high
            'foot_clearance': 0.5,  # reward for alternating gait
            'wrist_movement': 1.0,  # penalty for wrist movement (keep wrists fixed)
            'waist_movement': 0.5,  # penalty for waist/torso rotation (keep stable)
            'slip': 0.5,  # penalty for tangential slip while in contact
            'double_support': 0.5,  # penalty when both feet are off the ground
            'gait_phase': 0.5,  # reward for matching desired contact pattern
            'action_rate': 0.1,  # smoothness penalty
        }
        self.target_height = 0.793
        self.dt = self.frame_skip * self.model.opt.timestep

        self.act_low, self.act_high = self._compute_actuator_ranges()
        self.act_scale = (self.act_high - self.act_low) * 0.5
        self.act_bias = (self.act_high + self.act_low) * 0.5

        # Track previous control to penalize jerky actions
        self.prev_ctrl = np.zeros((self.num_envs, self.model.nu), dtype=np.float32)

        # Track gait phase per environment (drives alternating pattern)
        self.phase = np.zeros(self.num_envs, dtype=np.float32)

        # Find wrist actuator indices (to penalize wrist movement)
        self.wrist_actuator_ids = []
        # Find waist actuator indices (to penalize waist rotation)
        self.waist_actuator_ids = []
        for i in range(self.model.nu):
            act_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if act_name:
                if 'wrist' in act_name.lower():
                    self.wrist_actuator_ids.append(i)
                if 'waist' in act_name.lower() or 'torso' in act_name.lower():
                    self.waist_actuator_ids.append(i)

        self.torso_body_id = self._resolve_body_id(['torso', 'torso_link', 'pelvis'])
        if self.torso_body_id is None:
            raise RuntimeError("No torso body found in model.")

        # Find hand and head bodies for arm position penalty
        self.left_hand_id = self._resolve_body_id(['left_hand', 'left_palm', 'l_hand', 'hand_left'])
        self.right_hand_id = self._resolve_body_id(['right_hand', 'right_palm', 'r_hand', 'hand_right'])
        self.head_id = self._resolve_body_id(['head', 'head_link', 'skull'])

        # Find foot bodies for gait reward
        self.left_foot_id = self._resolve_body_id([
            'left_foot', 'left_ankle', 'l_foot', 'foot_left',
            'left_foot_link', 'left_ankle_link', 'left_toe', 'left_toe_link', 'left_ankle_roll', 'left_ankle_pitch'
        ])
        self.right_foot_id = self._resolve_body_id([
            'right_foot', 'right_ankle', 'r_foot', 'foot_right',
            'right_foot_link', 'right_ankle_link', 'right_toe', 'right_toe_link', 'right_ankle_roll', 'right_ankle_pitch'
        ])

        self.free_joint_id = self._find_free_joint()
        self.joint_qpos = self._build_joint_qpos_map()

        # obs: qpos | qvel | torso_z | roll | pitch | left_contact | right_contact | gait_phase | target_speed
        self.extra_obs_dim = 7
        self.obs_dim = self.model.nq + self.model.nv + self.extra_obs_dim
        self.act_dim = self.model.nu

        # Debug: print detected bodies (only once for env 0)
        if num_envs > 0:
            print("\n=== Body Detection Debug ===")
            print(f"Torso body ID: {self.torso_body_id}")
            print(f"Head body ID: {self.head_id}")
            print(f"Left hand body ID: {self.left_hand_id}")
            print(f"Right hand body ID: {self.right_hand_id}")
            print(f"Left foot body ID: {self.left_foot_id}")
            print(f"Right foot body ID: {self.right_foot_id}")
            print(f"Wrist actuators found: {len(self.wrist_actuator_ids)}")
            if len(self.wrist_actuator_ids) > 0:
                wrist_names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                              for i in self.wrist_actuator_ids]
                print(f"  Wrist actuator names: {wrist_names}")
            print(f"Waist actuators found: {len(self.waist_actuator_ids)}")
            if len(self.waist_actuator_ids) > 0:
                waist_names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                              for i in self.waist_actuator_ids]
                print(f"  Waist actuator names: {waist_names}")
            print("=" * 30)

        for i in range(num_envs):
            self.reset(i)

    def _resolve_body_id(self, names):
        for name in names:
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if body_id >= 0:
                return body_id
        return None

    def _find_free_joint(self):
        for jid in range(self.model.njnt):
            if self.model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_FREE:
                return jid
        return None

    def _compute_actuator_ranges(self):
        ctrlrange = self.model.actuator_ctrlrange.copy()
        act_low = ctrlrange[:, 0].copy()
        act_high = ctrlrange[:, 1].copy()
        for i in range(self.model.nu):
            if act_low[i] == act_high[i]:
                joint_id = int(self.model.actuator_trnid[i, 0])
                if joint_id >= 0:
                    jrange = self.model.jnt_range[joint_id]
                    if jrange[0] != jrange[1]:
                        act_low[i], act_high[i] = float(jrange[0]), float(jrange[1])
                    else:
                        act_low[i], act_high[i] = -1.0, 1.0
                else:
                    act_low[i], act_high[i] = -1.0, 1.0
        return act_low, act_high

    def _build_joint_qpos_map(self):
        mapping = {}
        for jid in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            if name:
                mapping[name] = self.model.jnt_qposadr[jid]
        return mapping

    def reset(self, idx: int):
        data = self.data[idx]
        mujoco.mj_resetData(self.model, data)
        data.qvel[:] = 0.0

        if self.free_joint_id is not None:
            qpos_addr = self.model.jnt_qposadr[self.free_joint_id]
            # Add small random yaw and lateral offset to improve robustness
            yaw = np.random.uniform(-0.05, 0.05)
            data.qpos[qpos_addr:qpos_addr + 7] = np.array([0.0, 0.0, 0.793, math.cos(yaw * 0.5), 0.0, 0.0, math.sin(yaw * 0.5)])

        # Slightly bent knees/ankles with random noise
        self._set_joint_pose(data, 'left_knee_joint', 0.12 + np.random.uniform(-0.02, 0.02))
        self._set_joint_pose(data, 'right_knee_joint', 0.12 + np.random.uniform(-0.02, 0.02))
        self._set_joint_pose(data, 'left_ankle_pitch_joint', -0.06 + np.random.uniform(-0.02, 0.02))
        self._set_joint_pose(data, 'right_ankle_pitch_joint', -0.06 + np.random.uniform(-0.02, 0.02))

        # Randomize hip roll/pitch slightly to avoid overfitting to a single posture
        self._set_joint_pose(data, 'left_hip_roll_joint', np.random.uniform(-0.05, 0.05))
        self._set_joint_pose(data, 'right_hip_roll_joint', np.random.uniform(-0.05, 0.05))
        self._set_joint_pose(data, 'left_hip_pitch_joint', np.random.uniform(-0.05, 0.05))
        self._set_joint_pose(data, 'right_hip_pitch_joint', np.random.uniform(-0.05, 0.05))

        # Reset gait phase and previous control
        self.phase[idx] = np.random.uniform(0.0, 2.0 * math.pi)
        self.prev_ctrl[idx] = 0.0

        mujoco.mj_forward(self.model, data)

    def _set_joint_pose(self, data, name: str, value: float):
        qpos_addr = self.joint_qpos.get(name)
        if qpos_addr is not None:
            data.qpos[qpos_addr] = float(value)

    def _body_roll_pitch(self, data):
        r = data.xmat[self.torso_body_id].reshape(3, 3)
        pitch = math.asin(-max(-1.0, min(1.0, r[2, 0])))
        roll = math.atan2(r[2, 1], r[2, 2])
        return roll, pitch

    def _foot_contact(self, data, foot_body_id: int) -> bool:
        """Returns True if the given foot is in contact with any other body."""
        if foot_body_id is None:
            return False
        for c in data.contact[: data.ncon]:
            if c.geom1 == -1 or c.geom2 == -1:
                continue
            body1 = self.model.geom_bodyid[c.geom1]
            body2 = self.model.geom_bodyid[c.geom2]
            if body1 == foot_body_id or body2 == foot_body_id:
                return True
        return False

    def _foot_slip(self, data, foot_body_id: int) -> float:
        """Magnitude of tangential velocity at the foot contact point (approx)."""
        if foot_body_id is None:
            return 0.0
        vel = data.cvel[foot_body_id]  # 6D spatial vel; last 3 are linear in body frame
        tangential = math.sqrt(vel[3] ** 2 + vel[4] ** 2)  # planar speed along x/y
        return float(tangential)

    def _get_obs(self, data, idx: int):
        roll, pitch = self._body_roll_pitch(data)
        torso_z = data.xpos[self.torso_body_id][2]
        left_contact = 1.0 if self._foot_contact(data, self.left_foot_id) else 0.0
        right_contact = 1.0 if self._foot_contact(data, self.right_foot_id) else 0.0
        # Phase in [-1, 1] via sin to give smooth cyclical cue
        phase_feat = math.sin(self.phase[idx]) if self.num_envs > 0 else 0.0
        obs = np.concatenate(
            [data.qpos, data.qvel, np.array([torso_z, roll, pitch, left_contact, right_contact, phase_feat, self.target_speed])],
            axis=0,
        )
        return obs.astype(np.float32, copy=False)

    def step(self, actions: np.ndarray):
        obs = np.zeros((self.num_envs, self.obs_dim), dtype=np.float32)
        rewards = np.zeros(self.num_envs, dtype=np.float32)
        dones = np.zeros(self.num_envs, dtype=np.bool_)

        for i in range(self.num_envs):
            data = self.data[i]
            data.ctrl[:] = actions[i]
            for _ in range(self.frame_skip):
                mujoco.mj_step(self.model, data)

            roll, pitch = self._body_roll_pitch(data)
            torso_z = data.xpos[self.torso_body_id][2]
            forward_vel = data.cvel[self.torso_body_id][3]
            lateral_vel = abs(data.cvel[self.torso_body_id][4])
            energy = float(np.mean(np.square(data.ctrl)))
            # Gait phase update
            self.phase[i] = (self.phase[i] + self.phase_rate * self.dt) % (2.0 * math.pi)
            phase_sign = math.sin(self.phase[i])

            left_contact = self._foot_contact(data, self.left_foot_id)
            right_contact = self._foot_contact(data, self.right_foot_id)
            slip_left = self._foot_slip(data, self.left_foot_id) if left_contact else 0.0
            slip_right = self._foot_slip(data, self.right_foot_id) if right_contact else 0.0
            height_error = abs(torso_z - self.target_height)
            speed_error = forward_vel - self.target_speed

            # Penalty for arms raised too high (natural arm swing should be low)
            arms_penalty = 0.0
            # Target: arms should swing between hip and chest level (torso_z - 0.3 to torso_z + 0.1)
            target_max_hand_z = torso_z + 0.1  # max ~chest level

            if self.left_hand_id is not None:
                left_hand_z = data.xpos[self.left_hand_id][2]
                if left_hand_z > target_max_hand_z:
                    arms_penalty += (left_hand_z - target_max_hand_z)

            if self.right_hand_id is not None:
                right_hand_z = data.xpos[self.right_hand_id][2]
                if right_hand_z > target_max_hand_z:
                    arms_penalty += (right_hand_z - target_max_hand_z)

            # Extra penalty if hands go above head
            if self.head_id is not None:
                head_z = data.xpos[self.head_id][2]
                if self.left_hand_id is not None and left_hand_z > head_z:
                    arms_penalty += 0.5 * (left_hand_z - head_z)  # extra penalty
                if self.right_hand_id is not None and right_hand_z > head_z:
                    arms_penalty += 0.5 * (right_hand_z - head_z)  # extra penalty

            # Reward for alternating gait (one foot up, one foot down)
            foot_alternation_reward = 0.0
            foot_excessive_height_penalty = 0.0

            if self.left_foot_id is not None and self.right_foot_id is not None:
                left_foot_z = data.xpos[self.left_foot_id][2]
                right_foot_z = data.xpos[self.right_foot_id][2]

                # Reward when feet are at different heights (forces alternation)
                if forward_vel > 0.1:  # only during forward movement
                    foot_height_diff = abs(left_foot_z - right_foot_z)
                    # Reward proportional to height difference (up to ~0.10m)
                    foot_alternation_reward = min(foot_height_diff, 0.10)

                # NEW: Penalty for EACH foot being raised TOO high (absolute height)
                max_reasonable_foot_height = 0.12  # 12cm is max reasonable clearance
                if left_foot_z > max_reasonable_foot_height:
                    foot_excessive_height_penalty += (left_foot_z - max_reasonable_foot_height)
                if right_foot_z > max_reasonable_foot_height:
                    foot_excessive_height_penalty += (right_foot_z - max_reasonable_foot_height)

            # Penalty for wrist movement (wrists should stay fixed during walking)
            wrist_movement_penalty = 0.0
            if len(self.wrist_actuator_ids) > 0:
                wrist_controls = np.abs(data.ctrl[self.wrist_actuator_ids])
                wrist_movement_penalty = float(np.mean(wrist_controls))

            # Penalty for waist movement (waist should stay stable during walking)
            waist_movement_penalty = 0.0
            if len(self.waist_actuator_ids) > 0:
                waist_controls = np.abs(data.ctrl[self.waist_actuator_ids])
                waist_movement_penalty = float(np.mean(waist_controls))

            # Reward/penalty for matching desired gait phase: alternate contacts with phase
            desired_left_contact = phase_sign > 0.0
            desired_right_contact = phase_sign <= 0.0
            gait_match = 0.0
            if left_contact == desired_left_contact:
                gait_match += 0.5
            if right_contact == desired_right_contact:
                gait_match += 0.5

            # Penalty if both feet off the ground (hop)
            double_support_penalty = 0.0
            if not left_contact and not right_contact:
                double_support_penalty = 1.0

            # Slip penalty for stance foot
            slip_penalty = 0.0
            if left_contact:
                slip_penalty += max(0.0, slip_left - self.foot_slip_threshold)
            if right_contact:
                slip_penalty += max(0.0, slip_right - self.foot_slip_threshold)

            # Smoothness: penalize action rate of change
            action_rate = float(np.mean(np.abs(data.ctrl - self.prev_ctrl[i])))
            self.prev_ctrl[i] = data.ctrl.copy()

            # Orientation penalties with threshold-based extra penalty
            roll_penalty = self.rw['roll'] * roll**2
            pitch_penalty = self.rw['pitch'] * pitch**2

            # Extra penalty if pitch exceeds critical threshold (prevents falling)
            if abs(pitch) > self.rw['pitch_threshold']:
                pitch_excess = abs(pitch) - self.rw['pitch_threshold']
                pitch_penalty += self.rw['pitch_extra'] * pitch_excess

            # Improved reward function for stable walking
            reward = (
                # Base survival reward
                self.rw['survival']
                # Forward velocity reward (capped to encourage consistent speed, not just max speed)
                + self.rw['forward_vel'] * min(forward_vel, self.rw['vel_cap'])
                # Penalty for deviating from target speed band
                - self.rw['target_speed'] * speed_error**2
                # Quadratic penalties for orientation (more tolerant to small deviations)
                - roll_penalty
                - pitch_penalty
                # Height stability (quadratic penalty for deviation from target)
                - self.rw['height'] * height_error**2
                # Lateral velocity penalty (discourage sideways drift)
                - self.rw['lateral'] * lateral_vel
                # Energy efficiency (smoother movements)
                - self.rw['energy'] * energy
                # Arms above head penalty (unnatural for walking)
                - self.rw['arms_high'] * arms_penalty
                # Foot alternation reward (encourages proper gait - one up, one down)
                + self.rw['foot_clearance'] * foot_alternation_reward
                # Penalty for feet raised TOO high (absolute height limit)
                - self.rw['foot_clearance'] * 2.0 * foot_excessive_height_penalty
                # Wrist movement penalty (keep wrists fixed during walking)
                - self.rw['wrist_movement'] * wrist_movement_penalty
                # Waist movement penalty (keep waist stable during walking)
                - self.rw['waist_movement'] * waist_movement_penalty
                # Contact pattern reward
                + self.rw['gait_phase'] * gait_match
                # Penalties for hopping and slipping
                - self.rw['double_support'] * double_support_penalty
                - self.rw['slip'] * slip_penalty
                # Penalize jerky action changes
                - self.rw['action_rate'] * action_rate
            )

            fallen = (
                torso_z < self.fall_z
                or abs(roll) > self.fall_angle
                or abs(pitch) > self.fall_angle
            )

            # Consider a fall if yaw drift becomes too high (prevents spinning)
            yaw_rate = abs(data.cvel[self.torso_body_id][5])
            if yaw_rate > 2.5:
                fallen = True

            if fallen:
                reward -= self.rw['fall']
                dones[i] = True
                self.reset(i)

            obs[i] = self._get_obs(data, i)
            rewards[i] = reward

        return obs, rewards, dones

    def reset_all(self):
        for i in range(self.num_envs):
            self.reset(i)
        return np.stack([self._get_obs(d, i) for i, d in enumerate(self.data)], axis=0).astype(np.float32, copy=False)


def compute_gae(rewards, dones, values, last_value, gamma, lam):
    advantages = np.zeros_like(rewards)
    gae = 0.0
    for t in reversed(range(len(rewards))):
        next_value = last_value if t == len(rewards) - 1 else values[t + 1]
        nonterminal = 1.0 - dones[t].astype(np.float32)
        delta = rewards[t] + gamma * next_value * nonterminal - values[t]
        gae = delta + gamma * lam * nonterminal * gae
        advantages[t] = gae
    returns = advantages + values
    return advantages, returns


def main():
    parser = argparse.ArgumentParser(description="PPO training for MuJoCo humanoid.")
    parser.add_argument("--model", default="robot_description/humanoid_description/urdf/robot.xml")
    parser.add_argument("--num-envs", type=int, default=32)
    parser.add_argument("--total-steps", type=int, default=2_000_000)
    parser.add_argument("--rollout-steps", type=int, default=1024)
    parser.add_argument("--save-path", default="tools/ppo_humanoid.pt")
    parser.add_argument("--resume", action="store_true", help="Resume training from checkpoint")
    parser.add_argument("--resume-path", default="", help="Checkpoint path (defaults to --save-path)")
    parser.add_argument("--fall-z", type=float, default=0.6)
    parser.add_argument("--fall-angle-deg", type=float, default=45.0)
    parser.add_argument("--frame-skip", type=int, default=3)
    parser.add_argument("--viewer", action="store_true", help="Open MuJoCo viewer for env 0")
    parser.add_argument("--viewer-stride", type=int, default=10, help="Viewer sync stride in steps")
    parser.add_argument("--target-speed", type=float, default=1.2, help="Desired forward COM speed (m/s)")
    parser.add_argument("--phase-rate-hz", type=float, default=1.6, help="Hz of gait oscillator driving alternation")
    parser.add_argument("--slip-threshold", type=float, default=0.5, help="Tangential velocity threshold before slip penalty")

    # Reward function weights
    parser.add_argument("--rw-survival", type=float, default=1.0, help="Survival reward weight")
    parser.add_argument("--rw-forward-vel", type=float, default=2.5, help="Forward velocity reward weight")
    parser.add_argument("--rw-vel-cap", type=float, default=1.5, help="Velocity cap (m/s)")
    parser.add_argument("--rw-roll", type=float, default=0.5, help="Roll penalty weight")
    parser.add_argument("--rw-pitch", type=float, default=0.5, help="Pitch penalty weight")
    parser.add_argument("--rw-pitch-threshold", type=float, default=14.0, help="Pitch threshold angle (degrees) for extra penalty")
    parser.add_argument("--rw-pitch-extra", type=float, default=2.0, help="Extra pitch penalty above threshold")
    parser.add_argument("--rw-height", type=float, default=1.0, help="Height deviation penalty weight")
    parser.add_argument("--rw-lateral", type=float, default=0.3, help="Lateral velocity penalty weight")
    parser.add_argument("--rw-energy", type=float, default=0.005, help="Energy penalty weight")
    parser.add_argument("--rw-target-speed", type=float, default=2.0, help="Target speed penalty weight")
    parser.add_argument("--rw-fall", type=float, default=5.0, help="Fall penalty")
    parser.add_argument("--rw-arms-high", type=float, default=1.0, help="Arms raised too high penalty weight")
    parser.add_argument("--rw-foot-clearance", type=float, default=0.5, help="Foot alternation reward weight")
    parser.add_argument("--rw-wrist-movement", type=float, default=1.0, help="Wrist movement penalty weight (keep fixed)")
    parser.add_argument("--rw-waist-movement", type=float, default=0.5, help="Waist movement penalty weight (keep stable)")
    parser.add_argument("--rw-slip", type=float, default=0.5, help="Slip penalty weight")
    parser.add_argument("--rw-double-support", type=float, default=0.5, help="Penalty when both feet are off-ground (hopping)")
    parser.add_argument("--rw-gait-phase", type=float, default=0.5, help="Reward for matching desired alternating contact pattern")
    parser.add_argument("--rw-action-rate", type=float, default=0.1, help="Penalty for rapid changes in torques")
    args = parser.parse_args()

    cfg = PPOConfig(num_envs=args.num_envs, total_steps=args.total_steps, rollout_steps=args.rollout_steps)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # Build reward weights dict from CLI args
    reward_weights = {
        'survival': args.rw_survival,
        'forward_vel': args.rw_forward_vel,
        'vel_cap': args.rw_vel_cap,
        'roll': args.rw_roll,
        'pitch': args.rw_pitch,
        'pitch_threshold': math.radians(args.rw_pitch_threshold),
        'pitch_extra': args.rw_pitch_extra,
        'height': args.rw_height,
        'lateral': args.rw_lateral,
        'energy': args.rw_energy,
        'target_speed': args.rw_target_speed,
        'fall': args.rw_fall,
        'arms_high': args.rw_arms_high,
        'foot_clearance': args.rw_foot_clearance,
        'wrist_movement': args.rw_wrist_movement,
        'waist_movement': args.rw_waist_movement,
        'slip': args.rw_slip,
        'double_support': args.rw_double_support,
        'gait_phase': args.rw_gait_phase,
        'action_rate': args.rw_action_rate,
    }

    env = HumanoidBatchEnv(
        model_path=args.model,
        num_envs=cfg.num_envs,
        fall_z=args.fall_z,
        fall_angle_deg=args.fall_angle_deg,
        frame_skip=args.frame_skip,
        reward_weights=reward_weights,
        target_speed=args.target_speed,
        phase_rate_hz=args.phase_rate_hz,
        foot_slip_threshold=args.slip_threshold,
    )

    print("\n=== Reward Function Configuration ===")
    print(f"Survival bonus:        {reward_weights['survival']:.3f}")
    print(f"Forward velocity:      {reward_weights['forward_vel']:.3f} * min(vel, {reward_weights['vel_cap']:.2f})")
    print(f"Roll penalty:          {reward_weights['roll']:.3f} * roll^2")
    print(f"Pitch penalty:         {reward_weights['pitch']:.3f} * pitch^2")
    pitch_threshold_deg = math.degrees(reward_weights['pitch_threshold'])
    print(f"  + Extra penalty:     {reward_weights['pitch_extra']:.3f} * excess above {pitch_threshold_deg:.1f}°")
    print(f"Height penalty:        {reward_weights['height']:.3f} * height_error^2")
    print(f"Lateral vel penalty:   {reward_weights['lateral']:.3f} * |lateral_vel|")
    print(f"Energy penalty:        {reward_weights['energy']:.5f} * energy")
    print(f"Target speed:          {reward_weights['target_speed']:.3f} * (vel - {args.target_speed:.2f})^2")
    print(f"Arms too high:         {reward_weights['arms_high']:.3f} * arms_penalty (limit: chest level)")
    print(f"Foot alternation:      {reward_weights['foot_clearance']:.3f} * foot_height_diff")
    print(f"Wrist movement:        {reward_weights['wrist_movement']:.3f} * wrist_controls (keep fixed)")
    print(f"Waist movement:        {reward_weights['waist_movement']:.3f} * waist_controls (keep stable)")
    print(f"Gait phase reward:     {reward_weights['gait_phase']:.3f} (match oscillator contacts)")
    print(f"Slip penalty:          {reward_weights['slip']:.3f} * excess slip above {args.slip_threshold:.2f} m/s")
    print(f"Double support penal.: {reward_weights['double_support']:.3f} when both feet airborne")
    print(f"Action rate penalty:   {reward_weights['action_rate']:.3f} * |u_t - u_(t-1)|")
    print(f"Fall penalty:          {reward_weights['fall']:.1f}")
    print("=" * 40 + "\n")

    obs_dim = env.obs_dim
    act_dim = env.act_dim

    policy = ActorCritic(obs_dim, act_dim).to(device)
    optimizer = torch.optim.Adam(policy.parameters(), lr=cfg.lr)

    ckpt = None
    start_update = 0
    global_step = 0
    if args.resume or args.resume_path:
        resume_path = args.resume_path or args.save_path
        ckpt = torch.load(resume_path, map_location=device, weights_only=False)
        if ckpt.get("obs_dim") != obs_dim or ckpt.get("act_dim") != act_dim:
            raise RuntimeError("Checkpoint dimensions do not match current model/env.")
        policy.load_state_dict(ckpt["model"])
        if "optimizer" in ckpt:
            optimizer.load_state_dict(ckpt["optimizer"])
        global_step = int(ckpt.get("global_step", 0))
        start_update = int(ckpt.get("update", 0))
        print(f"Resumed from {resume_path} at step {global_step}, update {start_update}")

    obs = env.reset_all()
    steps_per_update = cfg.rollout_steps * cfg.num_envs
    num_updates = cfg.total_steps // steps_per_update
    if global_step >= cfg.total_steps:
        print("Checkpoint already reached total_steps; nothing to do.")
        return
    remaining_updates = max(1, (cfg.total_steps - global_step) // steps_per_update)
    num_updates = remaining_updates
    start_time = time.time()

    sim_step = 0

    def train_loop(viewer=None):
        nonlocal obs, global_step, sim_step
        for update in range(start_update + 1, start_update + num_updates + 1):
            obs_buf = np.zeros((cfg.rollout_steps, cfg.num_envs, obs_dim), dtype=np.float32)
            act_buf = np.zeros((cfg.rollout_steps, cfg.num_envs, act_dim), dtype=np.float32)
            logp_buf = np.zeros((cfg.rollout_steps, cfg.num_envs), dtype=np.float32)
            rew_buf = np.zeros((cfg.rollout_steps, cfg.num_envs), dtype=np.float32)
            done_buf = np.zeros((cfg.rollout_steps, cfg.num_envs), dtype=np.bool_)
            val_buf = np.zeros((cfg.rollout_steps, cfg.num_envs), dtype=np.float32)

            for t in range(cfg.rollout_steps):
                obs_tensor = torch.from_numpy(obs).to(device)
                with torch.no_grad():
                    action, logp, value = policy.sample_action(obs_tensor)
                act = action.cpu().numpy()

                scaled_act = act * env.act_scale + env.act_bias
                next_obs, reward, done = env.step(scaled_act)

                obs_buf[t] = obs
                act_buf[t] = act
                logp_buf[t] = logp.cpu().numpy()
                val_buf[t] = value.cpu().numpy()
                rew_buf[t] = reward
                done_buf[t] = done

                obs = next_obs
                global_step += cfg.num_envs
                sim_step += 1

                if viewer is not None and sim_step % max(1, args.viewer_stride) == 0:
                    viewer.sync()

            with torch.no_grad():
                next_value = policy(torch.from_numpy(obs).to(device))[1].cpu().numpy()

            adv, ret = compute_gae(
                rew_buf,
                done_buf,
                val_buf,
                next_value,
                cfg.gamma,
                cfg.gae_lambda,
            )

            obs_flat = torch.from_numpy(obs_buf.reshape(-1, obs_dim)).to(device)
            act_flat = torch.from_numpy(act_buf.reshape(-1, act_dim)).to(device)
            logp_flat = torch.from_numpy(logp_buf.reshape(-1)).to(device)
            adv_flat = torch.from_numpy(adv.reshape(-1)).to(device)
            ret_flat = torch.from_numpy(ret.reshape(-1)).to(device)

            adv_flat = (adv_flat - adv_flat.mean()) / (adv_flat.std() + 1e-8)

            batch_size = cfg.rollout_steps * cfg.num_envs
            indices = np.arange(batch_size)

            for _ in range(cfg.update_epochs):
                np.random.shuffle(indices)
                for start in range(0, batch_size, cfg.minibatch_size):
                    end = start + cfg.minibatch_size
                    mb_idx = indices[start:end]

                    mu, value = policy(obs_flat[mb_idx])
                    logp = policy.log_prob_from_action(mu, policy.log_std, act_flat[mb_idx])
                    ratio = torch.exp(logp - logp_flat[mb_idx])

                    surr1 = ratio * adv_flat[mb_idx]
                    surr2 = torch.clamp(ratio, 1.0 - cfg.clip_ratio, 1.0 + cfg.clip_ratio) * adv_flat[mb_idx]
                    policy_loss = -torch.min(surr1, surr2).mean()

                    value_loss = ((value - ret_flat[mb_idx]) ** 2).mean()
                    entropy = -logp.mean()

                    loss = policy_loss + cfg.vf_coef * value_loss + cfg.ent_coef * entropy

                    optimizer.zero_grad()
                    loss.backward()
                    nn.utils.clip_grad_norm_(policy.parameters(), cfg.max_grad_norm)
                    optimizer.step()

            if update % 10 == 0 or update == 1:
                fps = int(global_step / max(1.0, time.time() - start_time))
                avg_reward = float(np.mean(rew_buf))
                print(
                    f"Update {update}/{num_updates} | Step {global_step} | "
                    f"Avg reward {avg_reward:.3f} | FPS {fps}"
                )
                torch.save(
                    {
                        "model": policy.state_dict(),
                        "optimizer": optimizer.state_dict(),
                        "obs_dim": obs_dim,
                        "act_dim": act_dim,
                        "act_scale": env.act_scale,
                        "act_bias": env.act_bias,
                        "global_step": global_step,
                        "update": update,
                    },
                    args.save_path,
                )

    if args.viewer:
        if not MUJOCO_VIEWER_AVAILABLE:
            raise SystemExit("MuJoCo viewer not available. Install mujoco with viewer support.")
        with mujoco.viewer.launch_passive(env.model, env.data[0]) as viewer:
            train_loop(viewer)
    else:
        train_loop()
    print(f"Training complete. Saved to: {args.save_path}")


if __name__ == "__main__":
    main()
