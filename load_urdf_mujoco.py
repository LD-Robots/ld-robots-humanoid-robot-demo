#!/usr/bin/env python3
"""
Load URDF in MuJoCo by changing to URDF directory (for relative mesh paths).
"""

import mujoco
import mujoco.viewer
import os
import sys

def load_urdf_in_mujoco(urdf_path):
    """Load URDF by changing to its directory (mesh paths are relative)."""

    urdf_abs = os.path.abspath(urdf_path)
    urdf_dir = os.path.dirname(urdf_abs)
    urdf_file = os.path.basename(urdf_abs)

    print("=" * 60)
    print("Loading URDF in MuJoCo")
    print("=" * 60)
    print(f"URDF file: {urdf_abs}")
    print(f"Changing to directory: {urdf_dir}")
    print(f"Loading: {urdf_file}")

    # Save current directory
    original_dir = os.getcwd()

    try:
        # Change to URDF directory so relative paths work
        os.chdir(urdf_dir)

        # Load URDF
        print(f"\nLoading model...")
        model = mujoco.MjModel.from_xml_path(urdf_file)

        print(f"✓ Model loaded successfully!")
        print(f"  Bodies: {model.nbody}")
        print(f"  Joints: {model.njnt}")
        print(f"  DOFs (nv): {model.nv}")
        print(f"  Actuators: {model.nu}")

        # Print joint names
        print(f"\nJoint names:")
        for i in range(model.njnt):
            joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            joint_type = model.jnt_type[i]
            print(f"  [{i}] {joint_name} (type={joint_type})")

        # Create data
        data = mujoco.MjData(model)

        # Set initial pose (standing)
        # Assumes freejoint at root with 7 DOFs: x, y, z, qw, qx, qy, qz
        if model.nv >= 6:
            data.qpos[0] = 0.0  # X
            data.qpos[1] = 0.0  # Y
            data.qpos[2] = 0.89  # Z - standing height
            data.qpos[3] = 1.0  # qw (quaternion)
            data.qpos[4] = 0.0  # qx
            data.qpos[5] = 0.0  # qy
            data.qpos[6] = 0.0  # qz

        # Forward kinematics
        mujoco.mj_forward(model, data)

        print(f"\n✓ Initial pose set (standing at Z=0.89m)")

        # Return to original directory
        os.chdir(original_dir)

        return model, data

    except Exception as e:
        os.chdir(original_dir)
        print(f"\n✗ Error: {e}")
        return None, None

def view_model(model, data):
    """Open MuJoCo viewer."""
    print("\n" + "=" * 60)
    print("Opening MuJoCo Viewer")
    print("=" * 60)
    print("Controls:")
    print("  - Mouse drag: Rotate view")
    print("  - Scroll: Zoom")
    print("  - ESC: Exit")
    print("=" * 60)

    mujoco.viewer.launch(model, data)

if __name__ == '__main__':
    urdf_path = 'src/robot_description/humanoid_description/urdf/robot.urdf'

    if len(sys.argv) > 1:
        urdf_path = sys.argv[1]

    model, data = load_urdf_in_mujoco(urdf_path)

    if model is not None:
        print(f"\nSuccess! Model loaded.")
        print(f"\nTo view in MuJoCo viewer, add --view flag:")
        print(f"  python3 {__file__} {urdf_path} --view")

        if '--view' in sys.argv or '-v' in sys.argv:
            view_model(model, data)
    else:
        print(f"\nFailed to load model.")
        sys.exit(1)
