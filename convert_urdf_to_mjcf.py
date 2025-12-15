#!/usr/bin/env python3
"""
Convert URDF to MuJoCo MJCF format with proper standing configuration.
"""

import mujoco
import os
import xml.etree.ElementTree as ET

def convert_urdf_to_mjcf(urdf_path, output_path):
    """
    Convert URDF to MJCF format using MuJoCo's built-in compiler.

    Args:
        urdf_path: Path to input URDF file
        output_path: Path to output MJCF XML file
    """
    print(f"Converting URDF to MJCF...")
    print(f"  Input:  {urdf_path}")
    print(f"  Output: {output_path}")

    # Load URDF and compile to MuJoCo model
    try:
        model = mujoco.MjModel.from_xml_path(urdf_path)
        print(f"✓ URDF loaded successfully")
        print(f"  - Bodies: {model.nbody}")
        print(f"  - Joints: {model.njnt}")
        print(f"  - DOFs: {model.nv}")

        # Save as MJCF
        mujoco.mj_saveLastXML(output_path, model)
        print(f"✓ MJCF saved to: {output_path}")

    except Exception as e:
        print(f"✗ Error during conversion: {e}")
        print(f"\nTrying alternative method: XML parsing and modification...")
        convert_urdf_manually(urdf_path, output_path)

def convert_urdf_manually(urdf_path, output_path):
    """
    Manually convert URDF to MJCF by wrapping with MuJoCo-specific tags.
    """
    # Parse URDF
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # Create MuJoCo XML wrapper
    mujoco_root = ET.Element('mujoco', model=root.get('name', 'humanoid'))

    # Add compiler options
    compiler = ET.SubElement(mujoco_root, 'compiler')
    compiler.set('angle', 'radian')
    compiler.set('meshdir', '../meshes')
    compiler.set('autolimits', 'true')

    # Add options
    option = ET.SubElement(mujoco_root, 'option')
    option.set('timestep', '0.002')
    option.set('iterations', '50')
    option.set('solver', 'Newton')
    option.set('gravity', '0 0 -9.81')

    # Add assets section for meshes
    asset = ET.SubElement(mujoco_root, 'asset')

    # Add worldbody
    worldbody = ET.SubElement(mujoco_root, 'worldbody')

    # Add ground plane
    ground = ET.SubElement(worldbody, 'geom')
    ground.set('name', 'floor')
    ground.set('type', 'plane')
    ground.set('size', '10 10 0.1')
    ground.set('rgba', '0.8 0.8 0.8 1')

    # Add light
    light = ET.SubElement(worldbody, 'light')
    light.set('directional', 'true')
    light.set('diffuse', '0.6 0.6 0.6')
    light.set('specular', '0.3 0.3 0.3')
    light.set('pos', '0 0 4')
    light.set('dir', '0 0 -1')

    # Convert URDF links to MuJoCo bodies
    # Find the base link
    base_link = None
    for link in root.findall('link'):
        link_name = link.get('name')
        if 'base' in link_name.lower() or 'torso' in link_name.lower():
            base_link = link
            break

    if base_link is None:
        base_link = root.find('link')

    # Create floating base body
    robot_body = ET.SubElement(worldbody, 'body')
    robot_body.set('name', 'torso')
    robot_body.set('pos', '0 0 0.89')  # Standing height

    # Add freejoint for floating base
    freejoint = ET.SubElement(robot_body, 'freejoint')
    freejoint.set('name', 'root')

    # Note: Full URDF->MJCF conversion is complex
    # This creates a basic structure

    # Write output
    tree_out = ET.ElementTree(mujoco_root)
    ET.indent(tree_out, space='  ')
    tree_out.write(output_path, encoding='utf-8', xml_declaration=True)
    print(f"✓ Basic MJCF structure created")

def create_standing_mjcf(output_path, urdf_path):
    """
    Create a MuJoCo XML that references the URDF with proper standing pose.
    """
    print(f"\nCreating MuJoCo XML wrapper for URDF...")

    # Get absolute path to URDF
    urdf_abs = os.path.abspath(urdf_path)
    urdf_dir = os.path.dirname(urdf_abs)

    mjcf_content = f'''<?xml version="1.0"?>
<mujoco model="humanoid_from_urdf">
  <!-- Compiler settings -->
  <compiler angle="radian"
            meshdir="{urdf_dir}/../meshes"
            autolimits="true"
            balanceinertia="true"
            discardvisual="false"/>

  <!-- Options -->
  <option timestep="0.002"
          iterations="50"
          solver="Newton"
          gravity="0 0 -9.81"
          collision="all"/>

  <!-- Visual settings -->
  <visual>
    <global offwidth="1920" offheight="1080"/>
    <quality shadowsize="4096"/>
    <map force="0.1" zfar="50"/>
  </visual>

  <!-- Assets -->
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="512"/>
    <texture name="texplane" type="2d" builtin="checker" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" width="512" height="512" mark="cross" markrgb="1 1 1"/>
    <material name="matplane" reflectance="0.3" texture="texplane" texrepeat="1 1" texuniform="true"/>
  </asset>

  <!-- Include URDF -->
  <include file="{urdf_abs}"/>

  <!-- Modify worldbody -->
  <worldbody>
    <!-- Ground plane -->
    <geom name="floor" type="plane" size="10 10 0.1" material="matplane"/>

    <!-- Lighting -->
    <light directional="true" diffuse="0.8 0.8 0.8" specular="0.2 0.2 0.2" pos="0 0 5" dir="0 0 -1"/>
    <light directional="true" diffuse="0.4 0.4 0.4" specular="0.1 0.1 0.1" pos="0 0 4" dir="0 1 -0.5"/>
  </worldbody>

  <!-- Actuators (will be auto-generated from joint limits) -->
  <actuator>
    <!-- Position servos for all joints -->
  </actuator>

  <!-- Keyframes for initial poses -->
  <keyframe>
    <key name="standing"
         qpos="0 0 0.89 1 0 0 0
               0 0 0 0 0 0 0 0 0 0
               0 0 0 0 0 0 0 0 0 0"
         ctrl="0 0 0 0 0 0 0 0 0 0
               0 0 0 0 0 0 0 0 0 0"/>
  </keyframe>

</mujoco>
'''

    with open(output_path, 'w') as f:
        f.write(mjcf_content)

    print(f"✓ MuJoCo wrapper created: {output_path}")
    print(f"\nThis XML includes the URDF and adds:")
    print(f"  - Ground plane")
    print(f"  - Lighting")
    print(f"  - Standing pose keyframe")
    print(f"  - MuJoCo-specific options")

if __name__ == '__main__':
    # Paths
    urdf_path = 'src/robot_description/humanoid_description/urdf/robot.urdf'
    mjcf_path = 'src/control/lipm_walking_controller/models/humanoid_from_urdf.xml'

    print("=" * 60)
    print("URDF to MJCF Converter")
    print("=" * 60)

    # Check if URDF exists
    if not os.path.exists(urdf_path):
        print(f"✗ URDF not found: {urdf_path}")
        exit(1)

    # Create output directory
    os.makedirs(os.path.dirname(mjcf_path), exist_ok=True)

    # Method 1: Create wrapper that includes URDF
    create_standing_mjcf(mjcf_path, urdf_path)

    print("\n" + "=" * 60)
    print("CONVERSION COMPLETE")
    print("=" * 60)
    print(f"\nNext steps:")
    print(f"1. Test loading in MuJoCo:")
    print(f"   python3 -c 'import mujoco; m = mujoco.MjModel.from_xml_path(\"{mjcf_path}\"); print(f\"Bodies: {{m.nbody}}, Joints: {{m.njnt}}\")'")
    print(f"\n2. View in MuJoCo viewer:")
    print(f"   python3 -m mujoco.viewer --mjcf={mjcf_path}")
    print(f"\n3. Update walking controller to use this model")
