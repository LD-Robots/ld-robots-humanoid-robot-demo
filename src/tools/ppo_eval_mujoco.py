#!/usr/bin/env python3
"""
Evaluate a trained PPO policy in MuJoCo with optional viewer.
"""

from __future__ import annotations

import argparse
import math
import time

import numpy as np

try:
    import mujoco
    import mujoco.viewer
except ImportError as exc:
    raise SystemExit("MuJoCo not installed. Run: pip install mujoco") from exc

import torch
import torch.nn as nn


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


class HumanoidEnv:
    def __init__(
        self,
        model_path: str,
        frame_skip: int = 3,
        target_speed: float = 1.2,
        phase_rate_hz: float = 1.6,
    ):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.frame_skip = int(frame_skip)
        self.target_speed = float(target_speed)
        self.phase_rate = 2.0 * math.pi * float(phase_rate_hz)
        self.dt = self.frame_skip * self.model.opt.timestep

        self.torso_body_id = self._resolve_body_id(['torso', 'torso_link', 'pelvis'])
        if self.torso_body_id is None:
            raise RuntimeError("No torso body found in model.")

        # Locate feet for contact flags
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

        self.act_low, self.act_high = self._compute_actuator_ranges()
        self.act_scale = (self.act_high - self.act_low) * 0.5
        self.act_bias = (self.act_high + self.act_low) * 0.5

        # obs: qpos | qvel | torso_z | roll | pitch | left_contact | right_contact | gait_phase | target_speed
        self.extra_obs_dim = 7
        self.obs_dim = self.model.nq + self.model.nv + self.extra_obs_dim
        self.act_dim = self.model.nu

        self.phase = 0.0
        self.reset()

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

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qvel[:] = 0.0

        if self.free_joint_id is not None:
            qpos_addr = self.model.jnt_qposadr[self.free_joint_id]
            yaw = np.random.uniform(-0.05, 0.05)
            self.data.qpos[qpos_addr:qpos_addr + 7] = np.array([0.0, 0.0, 0.793, math.cos(yaw * 0.5), 0.0, 0.0, math.sin(yaw * 0.5)])

        self._set_joint_pose('left_knee_joint', 0.12 + np.random.uniform(-0.02, 0.02))
        self._set_joint_pose('right_knee_joint', 0.12 + np.random.uniform(-0.02, 0.02))
        self._set_joint_pose('left_ankle_pitch_joint', -0.06 + np.random.uniform(-0.02, 0.02))
        self._set_joint_pose('right_ankle_pitch_joint', -0.06 + np.random.uniform(-0.02, 0.02))
        self._set_joint_pose('left_hip_roll_joint', np.random.uniform(-0.05, 0.05))
        self._set_joint_pose('right_hip_roll_joint', np.random.uniform(-0.05, 0.05))
        self._set_joint_pose('left_hip_pitch_joint', np.random.uniform(-0.05, 0.05))
        self._set_joint_pose('right_hip_pitch_joint', np.random.uniform(-0.05, 0.05))

        self.phase = np.random.uniform(0.0, 2.0 * math.pi)

        mujoco.mj_forward(self.model, self.data)

    def _set_joint_pose(self, name: str, value: float):
        qpos_addr = self.joint_qpos.get(name)
        if qpos_addr is not None:
            self.data.qpos[qpos_addr] = float(value)

    def _body_roll_pitch(self):
        r = self.data.xmat[self.torso_body_id].reshape(3, 3)
        pitch = math.asin(-max(-1.0, min(1.0, r[2, 0])))
        roll = math.atan2(r[2, 1], r[2, 2])
        return roll, pitch

    def _foot_contact(self, foot_body_id: int) -> bool:
        if foot_body_id is None:
            return False
        for c in self.data.contact[: self.data.ncon]:
            if c.geom1 == -1 or c.geom2 == -1:
                continue
            body1 = self.model.geom_bodyid[c.geom1]
            body2 = self.model.geom_bodyid[c.geom2]
            if body1 == foot_body_id or body2 == foot_body_id:
                return True
        return False

    def obs(self):
        roll, pitch = self._body_roll_pitch()
        torso_z = self.data.xpos[self.torso_body_id][2]
        left_contact = 1.0 if self._foot_contact(self.left_foot_id) else 0.0
        right_contact = 1.0 if self._foot_contact(self.right_foot_id) else 0.0
        phase_feat = math.sin(self.phase)
        obs = np.concatenate(
            [self.data.qpos, self.data.qvel, np.array([torso_z, roll, pitch, left_contact, right_contact, phase_feat, self.target_speed])],
            axis=0,
        )
        return obs.astype(np.float32, copy=False)

    def step(self, action: np.ndarray):
        self.data.ctrl[:] = action
        for _ in range(self.frame_skip):
            mujoco.mj_step(self.model, self.data)
        self.phase = (self.phase + self.phase_rate * self.dt) % (2.0 * math.pi)

    def scaled_action(self, policy_action: np.ndarray):
        return policy_action * self.act_scale + self.act_bias


def main():
    parser = argparse.ArgumentParser(description="Evaluate PPO policy in MuJoCo.")
    parser.add_argument("--model", default="robot_description/humanoid_description/urdf/robot.xml")
    parser.add_argument("--checkpoint", default="tools/ppo_humanoid.pt")
    parser.add_argument("--steps", type=int, default=5000)
    parser.add_argument("--frame-skip", type=int, default=3)
    parser.add_argument("--target-speed", type=float, default=1.2, help="Desired forward COM speed (for observation consistency)")
    parser.add_argument("--phase-rate-hz", type=float, default=1.6, help="Hz of gait oscillator driving alternation")
    parser.add_argument("--viewer", action="store_true", help="Open MuJoCo viewer")
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    ckpt = torch.load(args.checkpoint, map_location=device, weights_only=False)

    env = HumanoidEnv(
        args.model,
        frame_skip=args.frame_skip,
        target_speed=args.target_speed,
        phase_rate_hz=args.phase_rate_hz,
    )
    if ckpt.get("obs_dim") != env.obs_dim:
        raise RuntimeError(f"Checkpoint obs_dim {ckpt.get('obs_dim')} != env obs_dim {env.obs_dim}. Ensure trainer/eval stay in sync.")
    policy = ActorCritic(ckpt["obs_dim"], ckpt["act_dim"]).to(device)
    policy.load_state_dict(ckpt["model"])
    policy.eval()

    def run_loop(viewer=None):
        obs = env.obs()
        for _ in range(args.steps):
            obs_tensor = torch.from_numpy(obs).to(device)
            with torch.no_grad():
                mu, _ = policy(obs_tensor.unsqueeze(0))
            action = torch.tanh(mu).squeeze(0).cpu().numpy()
            env.step(env.scaled_action(action))
            obs = env.obs()
            if viewer is not None:
                viewer.sync()
                # Real-time playback: frame_skip * dt per action
                # MuJoCo default dt = 0.002s, frame_skip=5 -> 0.01s per action
                time.sleep(args.frame_skip * 0.002)

    if args.viewer:
        with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
            run_loop(viewer)
    else:
        run_loop()


if __name__ == "__main__":
    main()
