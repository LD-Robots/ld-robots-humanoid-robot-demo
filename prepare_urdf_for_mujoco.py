#!/usr/bin/env python3
"""
Prepare URDF for MuJoCo by fixing mesh paths and adding floating base.
"""

import xml.etree.ElementTree as ET
import os

def prepare_urdf_for_mujoco(input_urdf, output_xml):
    """
    Convert URDF to MuJoCo-compatible format.
    """
    print("=" * 60)
    print("Preparing URDF for MuJoCo")
    print("=" * 60)
    print(f"Input:  {input_urdf}")
    print(f"Output: {output_xml}")

    # Parse URDF
    tree = ET.parse(input_urdf)
    root = tree.getroot()

    # Get robot name
    robot_name = root.get('name', 'humanoid')

    # Create MuJoCo XML
    mujoco = ET.Element('mujoco', model=robot_name)

    # Add compiler
    compiler = ET.SubElement(mujoco, 'compiler')
    compiler.set('angle', 'radian')
    compiler.set('meshdir', os.path.dirname(input_urdf) + '/../meshes')
    compiler.set('balanceinertia', 'true')
    compiler.set('discardvisual', 'false')

    # Add option
    option = ET.SubElement(mujoco, 'option')
    option.set('timestep', '0.002')
    option.set('iterations', '50')
    option.set('solver', 'Newton')
    option.set('gravity', '0 0 -9.81')

    # Add visual
    visual = ET.SubElement(mujoco, 'visual')
    global_vis = ET.SubElement(visual, 'global')
    global_vis.set('offwidth', '1920')
    global_vis.set('offheight', '1080')

    # Add asset
    asset = ET.SubElement(mujoco, 'asset')

    # Ground textures
    texture_floor = ET.SubElement(asset, 'texture')
    texture_floor.set('name', 'texplane')
    texture_floor.set('type', '2d')
    texture_floor.set('builtin', 'checker')
    texture_floor.set('rgb1', '0.2 0.3 0.4')
    texture_floor.set('rgb2', '0.1 0.2 0.3')
    texture_floor.set('width', '512')
    texture_floor.set('height', '512')

    mat_floor = ET.SubElement(asset, 'material')
    mat_floor.set('name', 'matplane')
    mat_floor.set('reflectance', '0.3')
    mat_floor.set('texture', 'texplane')
    mat_floor.set('texrepeat', '1 1')

    # Create worldbody
    worldbody = ET.SubElement(mujoco, 'worldbody')

    # Add ground
    floor = ET.SubElement(worldbody, 'geom')
    floor.set('name', 'floor')
    floor.set('type', 'plane')
    floor.set('size', '10 10 0.1')
    floor.set('material', 'matplane')

    # Add light
    light1 = ET.SubElement(worldbody, 'light')
    light1.set('directional', 'true')
    light1.set('diffuse', '0.8 0.8 0.8')
    light1.set('specular', '0.2 0.2 0.2')
    light1.set('pos', '0 0 5')
    light1.set('dir', '0 0 -1')

    # Find base link (convert to floating base)
    base_link_name = None
    for link in root.findall('link'):
        link_name = link.get('name')
        if 'base' in link_name.lower():
            base_link_name = link_name
            break

    # Find first joint to get the real root
    first_joint = root.find('joint[@type="fixed"]')
    if first_joint is not None:
        root_link_name = first_joint.find('child').get('link') if first_joint.find('child') is not None else first_joint.get('child')
        if root_link_name is None:
            # Try direct attribute
            root_link_name = first_joint.attrib.get('child', 'KD_B_102B_TORSO_BTM')
    else:
        root_link_name = 'KD_B_102B_TORSO_BTM'

    print(f"Root link: {root_link_name}")

    # Create floating torso body
    torso_body = ET.SubElement(worldbody, 'body')
    torso_body.set('name', 'floating_base')
    torso_body.set('pos', '0 0 0.89')  # Standing height

    # Add freejoint
    freejoint = ET.SubElement(torso_body, 'freejoint')
    freejoint.set('name', 'root_joint')

    # Convert URDF links and joints to MuJoCo bodies
    # This is simplified - copy structure from URDF
    print(f"Converting {len(root.findall('link'))} links...")
    print(f"Converting {len(root.findall('joint'))} joints...")

    # For now, just copy the robot element content
    # MuJoCo will handle URDF-style definitions

    # Actually, let's use a simpler approach: wrap the URDF directly
    # Copy all links
    for link in root.findall('link'):
        worldbody.append(link)

    # Copy all joints
    for joint in root.findall('joint'):
        if joint.get('type') != 'fixed':  # Skip fixed joints
            mujoco.append(joint)

    # Add actuators
    actuator = ET.SubElement(mujoco, 'actuator')
    actuator_count = 0

    for joint in root.findall('joint'):
        if joint.get('type') == 'revolute' or joint.get('type') == 'continuous':
            joint_name = joint.get('name')

            # Add position actuator
            motor = ET.SubElement(actuator, 'position')
            motor.set('name', f'{joint_name}_actuator')
            motor.set('joint', joint_name)
            motor.set('kp', '100')
            motor.set('ctrlrange', '-3.14 3.14')
            actuator_count += 1

    print(f"Created {actuator_count} actuators")

    # Write output
    tree_out = ET.ElementTree(mujoco)
    ET.indent(tree_out, space='  ')
    tree_out.write(output_xml, encoding='utf-8', xml_declaration=True)

    print(f"✓ MuJoCo XML created: {output_xml}")
    return output_xml

if __name__ == '__main__':
    input_urdf = 'src/robot_description/humanoid_description/urdf/robot.urdf'
    output_xml = 'src/control/lipm_walking_controller/models/humanoid_mujoco.xml'

    if not os.path.exists(input_urdf):
        print(f"✗ URDF not found: {input_urdf}")
        exit(1)

    os.makedirs(os.path.dirname(output_xml), exist_ok=True)

    prepare_urdf_for_mujoco(input_urdf, output_xml)

    print("\n" + "=" * 60)
    print("Testing MuJoCo load...")
    print("=" * 60)

    try:
        import mujoco
        model = mujoco.MjModel.from_xml_path(output_xml)
        print(f"✓ Model loaded successfully!")
        print(f"  Bodies: {model.nbody}")
        print(f"  Joints: {model.njnt}")
        print(f"  DOFs: {model.nv}")
        print(f"  Actuators: {model.nu}")
    except Exception as e:
        print(f"✗ Error loading model: {e}")
        print("\nFalling back to direct URDF load from current directory...")

        # Try loading URDF directly by changing to URDF directory
        urdf_dir = os.path.dirname(os.path.abspath(input_urdf))
        print(f"Changing to: {urdf_dir}")

        import subprocess
        result = subprocess.run(
            ['python3', '-c',
             f"import os; os.chdir('{urdf_dir}'); import mujoco; m = mujoco.MjModel.from_xml_path('robot.urdf'); print(f'Bodies: {{m.nbody}}, Joints: {{m.njnt}}, DOFs: {{m.nv}}')"],
            capture_output=True, text=True
        )
        print(result.stdout)
        if result.returncode != 0:
            print(result.stderr)
