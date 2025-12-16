#!/usr/bin/env python3
"""
MuJoCo Visualizer - Lightweight viewer without ROS 2.
For quick model testing and debugging.
"""

import argparse
import time
import numpy as np

try:
    import mujoco
    import mujoco.viewer
    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False
    print("ERROR: MuJoCo not installed. Install with: pip install mujoco")


def add_perturbation(model, data, force_magnitude=10.0):
    """Apply random perturbation to torso for testing balance."""
    torso_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'torso')
    if torso_id >= 0:
        # Random horizontal force
        angle = np.random.uniform(0, 2 * np.pi)
        data.xfrc_applied[torso_id][0] = force_magnitude * np.cos(angle)
        data.xfrc_applied[torso_id][1] = force_magnitude * np.sin(angle)
        print(f"Applied perturbation: Fx={data.xfrc_applied[torso_id][0]:.2f}, Fy={data.xfrc_applied[torso_id][1]:.2f}")


def reset_to_standing(model, data):
    """Reset robot to standing pose."""
    # Reset simulation
    mujoco.mj_resetData(model, data)

    # Set to standing keyframe if available
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'standing')
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        print("Reset to 'standing' keyframe")
    else:
        print("No 'standing' keyframe found, using default reset")


def print_model_info(model):
    """Print MuJoCo model information."""
    print("\n" + "="*60)
    print("MuJoCo Model Information")
    print("="*60)
    print(f"Model name: {model.names().decode() if model.names else 'N/A'}")
    print(f"Number of bodies: {model.nbody}")
    print(f"Number of joints: {model.njnt}")
    print(f"Number of DoFs: {model.nv}")
    print(f"Number of actuators: {model.nu}")
    print(f"Number of sensors: {model.nsensor}")
    print(f"Timestep: {model.opt.timestep} seconds")
    print(f"Gravity: {model.opt.gravity}")

    print("\nJoints:")
    for i in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        joint_type = model.jnt_type[i]
        type_names = {0: 'free', 1: 'ball', 2: 'slide', 3: 'hinge'}
        print(f"  [{i}] {joint_name}: {type_names.get(joint_type, 'unknown')}")

    print("\nActuators:")
    for i in range(model.nu):
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print(f"  [{i}] {actuator_name}")

    print("\nSensors:")
    for i in range(model.nsensor):
        sensor_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
        print(f"  [{i}] {sensor_name}")

    print("="*60 + "\n")


def print_controls():
    """Print viewer controls."""
    print("\n" + "="*60)
    print("Viewer Controls")
    print("="*60)
    print("Mouse:")
    print("  Left button:   Rotate view")
    print("  Right button:  Move view")
    print("  Scroll:        Zoom")
    print("\nKeyboard:")
    print("  Space:         Pause/Resume simulation")
    print("  R:             Reset to standing pose")
    print("  P:             Apply random perturbation")
    print("  Esc:           Exit")
    print("="*60 + "\n")


def main():
    parser = argparse.ArgumentParser(description='MuJoCo Visualizer')
    parser.add_argument('model', type=str, help='Path to MuJoCo XML model file')
    parser.add_argument('--info', action='store_true', help='Print model info and exit')
    parser.add_argument('--realtime', type=float, default=1.0, help='Realtime factor (default: 1.0)')
    parser.add_argument('--test-balance', action='store_true', help='Periodically apply perturbations')

    args = parser.parse_args()

    if not MUJOCO_AVAILABLE:
        return

    # Load model
    try:
        print(f"Loading model: {args.model}")
        model = mujoco.MjModel.from_xml_path(args.model)
        data = mujoco.MjData(model)
        print("Model loaded successfully!")
    except Exception as e:
        print(f"ERROR: Failed to load model: {e}")
        return

    # Print model info
    print_model_info(model)

    if args.info:
        return

    # Print controls
    print_controls()

    # Launch viewer
    print("Launching MuJoCo viewer...")

    paused = False
    perturbation_timer = 0.0
    perturbation_interval = 5.0  # seconds

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Set camera
        viewer.cam.distance = 3.0
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20

        start_time = time.time()
        last_time = start_time

        while viewer.is_running():
            current_time = time.time()
            elapsed = current_time - last_time

            if not paused:
                # Step simulation
                mujoco.mj_step(model, data)

                # Test balance mode - apply periodic perturbations
                if args.test_balance:
                    perturbation_timer += elapsed
                    if perturbation_timer >= perturbation_interval:
                        add_perturbation(model, data, force_magnitude=20.0)
                        perturbation_timer = 0.0

            # Sync viewer
            viewer.sync()

            # Maintain realtime factor
            time_until_next_step = model.opt.timestep * args.realtime - elapsed
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

            last_time = current_time

            # Check for keyboard input
            # Note: MuJoCo viewer doesn't expose keyboard events directly
            # You would need to implement this using a separate input handler


if __name__ == '__main__':
    main()
