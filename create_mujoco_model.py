#!/usr/bin/env python3
"""
Create standalone MuJoCo XML from URDF with proper mesh directory configuration.
"""

import xml.etree.ElementTree as ET
import os

def create_mujoco_wrapper(urdf_path, output_path):
    """
    Create a MuJoCo XML that wraps URDF with proper compiler settings.
    """
    print("=" * 60)
    print("Creating MuJoCo Model from URDF")
    print("=" * 60)

    # Get absolute paths
    urdf_abs = os.path.abspath(urdf_path)
    urdf_dir = os.path.dirname(urdf_abs)
    mesh_dir = os.path.join(urdf_dir, '../meshes')
    mesh_dir_abs = os.path.abspath(mesh_dir)

    print(f"URDF: {urdf_abs}")
    print(f"Mesh dir: {mesh_dir_abs}")

    # Read the fixed URDF
    tree = ET.parse(urdf_path)
    robot = tree.getroot()
    robot_name = robot.get('name', 'humanoid')

    # Create MuJoCo root
    mujoco = ET.Element('mujoco', model=robot_name)

    # Compiler with absolute mesh path
    compiler = ET.SubElement(mujoco, 'compiler')
    compiler.set('angle', 'radian')
    compiler.set('meshdir', mesh_dir_abs)
    compiler.set('balanceinertia', 'true')

    # Option
    option = ET.SubElement(mujoco, 'option')
    option.set('timestep', '0.002')
    option.set('iterations', '50')
    option.set('solver', 'Newton')
    option.set('gravity', '0 0 -9.81')

    # Assets
    asset = ET.SubElement(mujoco, 'asset')
    tex = ET.SubElement(asset, 'texture')
    tex.set('name', 'texplane')
    tex.set('type', '2d')
    tex.set('builtin', 'checker')
    tex.set('rgb1', '0.2 0.3 0.4')
    tex.set('rgb2', '0.1 0.2 0.3')
    tex.set('width', '512')
    tex.set('height', '512')

    mat = ET.SubElement(asset, 'material')
    mat.set('name', 'matplane')
    mat.set('texture', 'texplane')
    mat.set('texrepeat', '1 1')

    # Worldbody
    worldbody = ET.SubElement(mujoco, 'worldbody')

    # Ground
    floor = ET.SubElement(worldbody, 'geom')
    floor.set('name', 'floor')
    floor.set('type', 'plane')
    floor.set('size', '10 10 0.1')
    floor.set('material', 'matplane')

    # Light
    light = ET.SubElement(worldbody, 'light')
    light.set('directional', 'true')
    light.set('diffuse', '0.8 0.8 0.8')
    light.set('pos', '0 0 5')
    light.set('dir', '0 0 -1')

    # Convert robot to floating base body
    torso = ET.SubElement(worldbody, 'body')
    torso.set('name', 'floating_base')
    torso.set('pos', '0 0 0.89')

    freejoint = ET.SubElement(torso, 'freejoint')
    freejoint.set('name', 'root_joint')

    # Process URDF links - add to torso body
    # We need to rebuild the kinematic tree

    # Find links and joints
    links = {link.get('name'): link for link in robot.findall('link')}
    joints = list(robot.findall('joint'))

    print(f"Found {len(links)} links, {len(joints)} joints")

    # Find base link (one without parent joint or with fixed base joint)
    base_link_name = None
    for joint in joints:
        if joint.get('type') == 'fixed':
            parent = joint.find('parent')
            child = joint.find('child')
            if parent is not None and child is not None:
                parent_name = parent.get('link')
                child_name = child.get('link')
                if 'base' in parent_name.lower():
                    base_link_name = child_name
                    break

    if base_link_name is None:
        base_link_name = 'KD_B_102B_TORSO_BTM'  # Default

    print(f"Base link: {base_link_name}")

    # Build kinematic tree recursively
    def add_link_to_body(link_name, parent_body_elem):
        if link_name not in links:
            return

        link_elem = links[link_name]

        # Add inertial
        inertial_urdf = link_elem.find('inertial')
        if inertial_urdf is not None:
            inertial = ET.SubElement(parent_body_elem, 'inertial')
            mass_elem = inertial_urdf.find('mass')
            if mass_elem is not None:
                inertial.set('mass', mass_elem.get('value', '1.0'))

            inertia_elem = inertial_urdf.find('inertia')
            if inertia_elem is not None:
                inertia = ET.SubElement(inertial, 'diaginertia')
                inertia.set('diag', f"{inertia_elem.get('ixx', '0.01')} {inertia_elem.get('iyy', '0.01')} {inertia_elem.get('izz', '0.01')}")

            origin_elem = inertial_urdf.find('origin')
            if origin_elem is not None:
                pos = origin_elem.get('xyz', '0 0 0')
                inertial.set('pos', pos)

        # Add collision geoms
        for collision in link_elem.findall('collision'):
            geom_elem = collision.find('geometry/mesh')
            if geom_elem is not None:
                geom = ET.SubElement(parent_body_elem, 'geom')
                mesh_file = geom_elem.get('filename', '')
                # Remove ../ prefix
                mesh_file = mesh_file.replace('../meshes/', '')
                geom.set('type', 'mesh')
                geom.set('mesh', mesh_file.replace('.stl', '').replace('.collision', ''))
                geom.set('rgba', '0.5 0.5 0.5 1')

                # Add mesh to assets
                mesh_asset = ET.SubElement(asset, 'mesh')
                mesh_asset.set('name', mesh_file.replace('.stl', '').replace('.collision', ''))
                mesh_asset.set('file', mesh_file)

        # Find child joints
        for joint in joints:
            parent_elem = joint.find('parent')
            if parent_elem is not None and parent_elem.get('link') == link_name:
                child_elem = joint.find('child')
                if child_elem is not None:
                    child_link_name = child_elem.get('link')

                    # Create child body
                    child_body = ET.SubElement(parent_body_elem, 'body')
                    child_body.set('name', child_link_name)

                    # Add joint origin as body pos
                    origin = joint.find('origin')
                    if origin is not None:
                        pos = origin.get('xyz', '0 0 0')
                        rpy = origin.get('rpy', '0 0 0')
                        child_body.set('pos', pos)
                        # TODO: convert RPY to quaternion for orientation

                    # Add joint
                    if joint.get('type') != 'fixed':
                        joint_elem = ET.SubElement(child_body, 'joint')
                        joint_elem.set('name', joint.get('name'))
                        joint_elem.set('type', 'hinge' if joint.get('type') == 'revolute' else joint.get('type'))

                        axis_elem = joint.find('axis')
                        if axis_elem is not None:
                            joint_elem.set('axis', axis_elem.get('xyz', '0 0 1'))

                        limit_elem = joint.find('limit')
                        if limit_elem is not None:
                            joint_elem.set('range', f"{limit_elem.get('lower', '-3.14')} {limit_elem.get('upper', '3.14')}")

                    # Recursively add child
                    add_link_to_body(child_link_name, child_body)

    # Start from base link
    add_link_to_body(base_link_name, torso)

    # Add actuators
    actuator = ET.SubElement(mujoco, 'actuator')
    for joint in joints:
        if joint.get('type') in ['revolute', 'continuous']:
            motor = ET.SubElement(actuator, 'position')
            motor.set('name', f"{joint.get('name')}_motor")
            motor.set('joint', joint.get('name'))
            motor.set('kp', '100')

    # Write output
    tree_out = ET.ElementTree(mujoco)
    ET.indent(tree_out, space='  ')
    tree_out.write(output_path, encoding='utf-8', xml_declaration=True)

    print(f"✓ MuJoCo model created: {output_path}")

    return output_path

if __name__ == '__main__':
    urdf_path = 'src/robot_description/humanoid_description/urdf/robot_fixed.urdf'
    output_path = 'src/control/lipm_walking_controller/models/humanoid_real.xml'

    if not os.path.exists(urdf_path):
        print(f"✗ Fixed URDF not found. Run fix_urdf_for_mujoco.py first!")
        exit(1)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    mjcf_path = create_mujoco_wrapper(urdf_path, output_path)

    print("\n" + "=" * 60)
    print("Testing model load...")
    print("=" * 60)

    try:
        import mujoco
        model = mujoco.MjModel.from_xml_path(mjcf_path)
        print(f"✓ Model loaded successfully!")
        print(f"  Bodies: {model.nbody}")
        print(f"  Joints: {model.njnt}")
        print(f"  DOFs: {model.nv}")
        print(f"  Actuators: {model.nu}")

        print(f"\n✓ SUCCESS! Ready to use in MuJoCo!")
        print(f"\nNext: python3 load_urdf_mujoco.py {mjcf_path} --view")

    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
