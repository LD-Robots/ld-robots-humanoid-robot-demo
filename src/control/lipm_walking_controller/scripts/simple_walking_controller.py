#!/usr/bin/env python3
"""
Simple Walking Controller for simple_humanoid.xml
Uses proven stable model with careful weight shifting strategy
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import sys


class SimpleWalkingController:
    """Walking controller for simple_humanoid.xml with proven stability"""

    def __init__(self, model_path: str):
        """Initialize the walking controller"""
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Walking parameters - VERY conservative for first attempt
        self.step_duration = 1.5  # 1.5 seconds per step
        self.lift_height = 0.03  # Only 3cm lift

        # State
        self.walking_phase = 0.0  # 0 to 1
        self.current_support = 'left'  # which leg supports
        self.steps_taken = 0
        self.target_steps = 8

        # Standing pose for simple_humanoid
        self.standing_pose = {
            'left_hip_yaw': 0.0,
            'left_hip_roll': 0.0,
            'left_hip_pitch': 0.0,
            'left_knee': 0.1,  # Slight bend for stability
            'left_ankle': -0.05,
            'right_hip_yaw': 0.0,
            'right_hip_roll': 0.0,
            'right_hip_pitch': 0.0,
            'right_knee': 0.1,
            'right_ankle': -0.05,
            'left_shoulder_pitch': 0.2,
            'left_elbow': 0.5,
            'right_shoulder_pitch': 0.2,
            'right_elbow': 0.5,
        }

        # Balance parameters
        self.hip_roll_shift = 0.12  # Lateral weight shift

        # Get joint/actuator indices
        self.joint_indices = {}
        self.actuator_indices = {}

        for joint_name in self.standing_pose.keys():
            try:
                joint_id = self.model.joint(joint_name).id
                self.joint_indices[joint_name] = self.model.jnt_qposadr[joint_id]
            except KeyError:
                print(f"Warning: Joint '{joint_name}' not found")

        for i in range(self.model.nu):
            actuator_name = self.model.actuator(i).name
            self.actuator_indices[actuator_name] = i

        print(f"✓ Found {len(self.joint_indices)} controlled joints")
        print(f"✓ Found {len(self.actuator_indices)} actuators")

    def set_initial_pose(self):
        """Set robot to standing pose"""
        for joint_name, target_angle in self.standing_pose.items():
            if joint_name in self.joint_indices:
                qpos_idx = self.joint_indices[joint_name]
                self.data.qpos[qpos_idx] = target_angle

        mujoco.mj_forward(self.model, self.data)
        print("✓ Initial standing pose set")

    def compute_swing_trajectory(self, phase):
        """
        Compute swing leg trajectory with minimal lift
        phase: 0 to 1
        Returns: (hip_pitch, knee, ankle, hip_roll)
        """
        # Smooth trajectory using sine wave
        smooth = 0.5 * (1 - np.cos(phase * np.pi))  # 0 to 1, smooth

        if phase < 0.5:
            # Lift phase
            normalized = phase * 2.0
            hip_pitch = 0.15 * np.sin(normalized * np.pi)  # Forward swing
            knee = 0.1 + 0.3 * np.sin(normalized * np.pi)  # Bend knee
            ankle = -0.05 + 0.05 * np.sin(normalized * np.pi)
        else:
            # Lower phase
            normalized = (phase - 0.5) * 2.0
            hip_pitch = 0.15 * np.sin((0.5 + normalized * 0.5) * np.pi)
            knee = 0.4 - 0.3 * normalized
            ankle = 0.0 - 0.05 * normalized

        # Hip roll for weight shift
        hip_roll = self.hip_roll_shift * np.sin(phase * np.pi)

        return hip_pitch, knee, ankle, hip_roll

    def compute_walking_pose(self):
        """Compute target pose for current walking phase"""
        pose = self.standing_pose.copy()

        if self.current_support == 'left':
            # Left leg supports, right leg swings
            hip_pitch, knee, ankle, hip_roll = self.compute_swing_trajectory(self.walking_phase)

            # Support leg (left) - nearly straight
            pose['left_hip_pitch'] = 0.0
            pose['left_hip_roll'] = -hip_roll  # Shift weight to left
            pose['left_knee'] = 0.1
            pose['left_ankle'] = -0.05

            # Swing leg (right)
            pose['right_hip_pitch'] = hip_pitch
            pose['right_hip_roll'] = hip_roll
            pose['right_knee'] = knee
            pose['right_ankle'] = ankle

        else:
            # Right leg supports, left leg swings
            hip_pitch, knee, ankle, hip_roll = self.compute_swing_trajectory(self.walking_phase)

            # Support leg (right)
            pose['right_hip_pitch'] = 0.0
            pose['right_hip_roll'] = hip_roll  # Shift weight to right
            pose['right_knee'] = 0.1
            pose['right_ankle'] = -0.05

            # Swing leg (left)
            pose['left_hip_pitch'] = hip_pitch
            pose['left_hip_roll'] = -hip_roll
            pose['left_knee'] = knee
            pose['left_ankle'] = ankle

        # Arm swing (opposite to legs)
        arm_swing = 0.3 * np.sin(self.walking_phase * np.pi)
        if self.current_support == 'left':
            # Right leg forward -> left arm forward
            pose['left_shoulder_pitch'] = 0.2 + arm_swing
            pose['right_shoulder_pitch'] = 0.2 - arm_swing
        else:
            pose['left_shoulder_pitch'] = 0.2 - arm_swing
            pose['right_shoulder_pitch'] = 0.2 + arm_swing

        return pose

    def send_position_commands(self, target_pose):
        """Send position commands to actuators"""
        for joint_name, target_angle in target_pose.items():
            actuator_name = joint_name + '_act'
            if actuator_name in self.actuator_indices:
                actuator_idx = self.actuator_indices[actuator_name]
                self.data.ctrl[actuator_idx] = target_angle

    def update_phase(self, dt):
        """Update walking phase"""
        self.walking_phase += dt / self.step_duration

        if self.walking_phase >= 1.0:
            self.walking_phase = 0.0
            self.steps_taken += 1

            # Switch support leg
            if self.current_support == 'left':
                self.current_support = 'right'
            else:
                self.current_support = 'left'

            print(f"✓ Step {self.steps_taken} completed, switching to {self.current_support} support")

    def run(self, duration: float = 30.0):
        """Run the walking controller"""
        print(f"\nStarting Simple Walking Controller...")
        print(f"Model: simple_humanoid.xml (proven stable)")
        print(f"Target: {self.target_steps} steps")
        print(f"Step duration: {self.step_duration}s\n")

        self.set_initial_pose()

        # Stand still for 2 seconds
        print("Standing for 2 seconds before walking...")
        start_time = time.time()
        standing_duration = 2.0

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                step_start = time.time()
                elapsed = step_start - start_time

                if elapsed < standing_duration:
                    # Standing phase
                    target_pose = self.standing_pose.copy()

                elif self.steps_taken < self.target_steps:
                    # Walking phase
                    target_pose = self.compute_walking_pose()
                    self.update_phase(self.model.opt.timestep)

                else:
                    # Finished, return to standing
                    target_pose = self.standing_pose.copy()

                # Send commands
                self.send_position_commands(target_pose)

                # Step simulation
                mujoco.mj_step(self.model, self.data)
                viewer.sync()

                # Print status
                if int(elapsed) != int(elapsed - self.model.opt.timestep):
                    base_height = self.data.qpos[2]
                    com = self.data.subtree_com[0][0:2]
                    print(f"Time: {elapsed:.1f}s | Steps: {self.steps_taken}/{self.target_steps} | "
                          f"Phase: {self.walking_phase:.2f} | Support: {self.current_support} | "
                          f"Base: {base_height:.3f}m | COM: ({com[0]:.3f}, {com[1]:.3f})")

                if elapsed > duration:
                    break

                # Timing
                time_until_next = self.model.opt.timestep - (time.time() - step_start)
                if time_until_next > 0:
                    time.sleep(time_until_next)

        print(f"\n✓ Walking finished after {elapsed:.1f}s")
        print(f"✓ Completed {self.steps_taken}/{self.target_steps} steps")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 simple_walking_controller.py <model_path> [duration]")
        sys.exit(1)

    model_path = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0

    print("=" * 60)
    print("SIMPLE WALKING CONTROLLER")
    print("=" * 60)
    print(f"Model: {model_path}")
    print(f"Duration: {duration}s")
    print("=" * 60)

    try:
        controller = SimpleWalkingController(model_path)
        controller.run(duration)
    except KeyboardInterrupt:
        print("\n✗ Stopped by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
