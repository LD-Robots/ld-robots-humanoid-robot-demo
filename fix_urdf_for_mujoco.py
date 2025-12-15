#!/usr/bin/env python3
"""
Fix URDF for MuJoCo by replacing package:// URIs with relative paths.
"""

import re
import os

def fix_urdf_for_mujoco(input_urdf, output_urdf):
    """
    Replace package:// URIs with relative paths for MuJoCo.
    """
    print("=" * 60)
    print("Fixing URDF for MuJoCo")
    print("=" * 60)
    print(f"Input:  {input_urdf}")
    print(f"Output: {output_urdf}")

    # Read URDF
    with open(input_urdf, 'r') as f:
        urdf_content = f.read()

    # Replace package:// with relative path
    # package://humanoid_description/meshes/... -> ../meshes/...
    modified_content = re.sub(
        r'package://humanoid_description/meshes/',
        '../meshes/',
        urdf_content
    )

    # Count replacements
    count = len(re.findall(r'package://humanoid_description/meshes/', urdf_content))
    print(f"✓ Replaced {count} package:// URIs with relative paths")

    # Write modified URDF
    with open(output_urdf, 'w') as f:
        f.write(modified_content)

    print(f"✓ Fixed URDF saved to: {output_urdf}")

    return output_urdf

if __name__ == '__main__':
    input_urdf = 'src/robot_description/humanoid_description/urdf/robot.urdf'
    output_urdf = 'src/robot_description/humanoid_description/urdf/robot_fixed.urdf'

    if not os.path.exists(input_urdf):
        print(f"✗ URDF not found: {input_urdf}")
        exit(1)

    fixed_urdf = fix_urdf_for_mujoco(input_urdf, output_urdf)

    print("\n" + "=" * 60)
    print("Testing with MuJoCo...")
    print("=" * 60)

    # Try loading in MuJoCo
    try:
        import mujoco
        import sys

        # Save current dir
        original_dir = os.getcwd()

        # Change to URDF directory
        urdf_dir = os.path.dirname(os.path.abspath(fixed_urdf))
        os.chdir(urdf_dir)

        print(f"Loading from: {urdf_dir}")
        print(f"File: {os.path.basename(fixed_urdf)}")

        model = mujoco.MjModel.from_xml_path(os.path.basename(fixed_urdf))

        print(f"\n✓ SUCCESS! Model loaded in MuJoCo!")
        print(f"  Bodies: {model.nbody}")
        print(f"  Joints: {model.njnt}")
        print(f"  DOFs: {model.nv}")
        print(f"  Actuators: {model.nu}")

        # Return to original dir
        os.chdir(original_dir)

        print(f"\n" + "=" * 60)
        print(f"Next steps:")
        print(f"=" * 60)
        print(f"1. View model:")
        print(f"   python3 load_urdf_mujoco.py {fixed_urdf} --view")
        print(f"")
        print(f"2. Use in walking controller:")
        print(f"   Update test_mujoco_walking.py to use: {fixed_urdf}")

    except Exception as e:
        os.chdir(original_dir)
        print(f"\n✗ Error loading in MuJoCo: {e}")
        import traceback
        traceback.print_exc()
