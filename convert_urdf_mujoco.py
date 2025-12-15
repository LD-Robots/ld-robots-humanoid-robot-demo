#!/usr/bin/env python3
"""
Convert URDF to MuJoCo MJCF by creating a wrapper with proper mesh directory.
"""

import os
import xml.etree.ElementTree as ET

def create_mujoco_wrapper_for_urdf(urdf_path, output_mjcf):
    """
    Create a MuJoCo XML wrapper that loads URDF with proper mesh directory.
    """
    print("=" * 60)
    print("Creating MuJoCo MJCF from URDF")
    print("=" * 60)

    urdf_abs = os.path.abspath(urdf_path)
    urdf_dir = os.path.dirname(urdf_abs)
    mesh_dir = os.path.join(urdf_dir, '../meshes')
    mesh_dir_abs = os.path.abspath(mesh_dir)

    print(f"URDF: {urdf_abs}")
    print(f"Mesh directory: {mesh_dir_abs}")

    # Read original URDF
    tree = ET.parse(urdf_path)
    robot = tree.getroot()
    robot_name = robot.get('name', 'humanoid')

    # Remove package:// URIs and replace with direct mesh paths
    for mesh_elem in robot.findall('.//mesh'):
        filename = mesh_elem.get('filename', '')
        if 'package://' in filename:
            # Extract just the mesh filename
            parts = filename.split('/')
            mesh_file = parts[-1]
            # Determine if visual or collision
            if 'collision' in filename:
                new_path = f'collision/{mesh_file}'
            else:
                new_path = f'visual/{mesh_file}'
            mesh_elem.set('filename', new_path)
            print(f"  Fixed mesh path: {filename} -> {new_path}")

    # Create MuJoCo root
    mujoco = ET.Element('mujoco', model=robot_name)

    # Compiler
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

    # Visual
    visual = ET.SubElement(mujoco, 'visual')
    global_vis = ET.SubElement(visual, 'global')
    global_vis.set('offwidth', '1920')
    global_vis.set('offheight', '1080')

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

    # Process URDF content
    # Convert fixed base to floating base
    base_link = None
    for joint in robot.findall('joint[@type="fixed"]'):
        child_elem = joint.find('child')
        if child_elem is not None:
            child_link = child_elem.get('link')
            if 'TORSO' in child_link.upper() or 'BASE' in child_link.upper():
                base_link = child_link
                break

    if base_link is None:
        base_link = 'KD_B_102B_TORSO_BTM'

    print(f"\nBase link: {base_link}")

    # Create floating base body
    torso_body = ET.SubElement(worldbody, 'body')
    torso_body.set('name', 'floating_base')
    torso_body.set('pos', '0 0 0.89')

    freejoint = ET.SubElement(torso_body, 'freejoint')
    freejoint.set('name', 'root_joint')

    # Copy all links except base
    link_dict = {}
    for link in robot.findall('link'):
        link_name = link.get('name')
        if link_name != 'base':
            link_dict[link_name] = link

    # Copy joints
    joint_list = []
    for joint in robot.findall('joint'):
        if joint.get('type') != 'fixed':
            joint_list.append(joint)

    # Build kinematic tree recursively
    def add_children(parent_body, parent_link_name):
        for joint in joint_list:
            parent_elem = joint.find('parent')
            if parent_elem is not None and parent_elem.get('link') == parent_link_name:
                child_elem = joint.find('child')
                if child_elem is None:
                    continue
                child_link_name = child_elem.get('link')

                # Create child body
                child_body = ET.SubElement(parent_body, 'body')
                child_body.set('name', child_link_name)

                # Add joint origin as body pos
                origin = joint.find('origin')
                if origin is not None:
                    pos = origin.get('xyz', '0 0 0')
                    child_body.set('pos', pos)

                # Add joint
                mj_joint = ET.SubElement(child_body, 'joint')
                mj_joint.set('name', joint.get('name'))
                joint_type = joint.get('type')
                if joint_type == 'revolute' or joint_type == 'continuous':
                    mj_joint.set('type', 'hinge')
                else:
                    mj_joint.set('type', joint_type)

                axis_elem = joint.find('axis')
                if axis_elem is not None:
                    mj_joint.set('axis', axis_elem.get('xyz', '0 0 1'))

                limit_elem = joint.find('limit')
                if limit_elem is not None:
                    lower = limit_elem.get('lower', '-3.14')
                    upper = limit_elem.get('upper', '3.14')
                    mj_joint.set('range', f'{lower} {upper}')

                # Add link geometry
                if child_link_name in link_dict:
                    link = link_dict[child_link_name]

                    # Add inertial
                    inertial = link.find('inertial')
                    if inertial is not None:
                        mj_inertial = ET.SubElement(child_body, 'inertial')
                        mass_elem = inertial.find('mass')
                        if mass_elem is not None:
                            mj_inertial.set('mass', mass_elem.get('value', '1.0'))

                    # Add collision geoms
                    for collision in link.findall('collision'):
                        mesh_elem = collision.find('geometry/mesh')
                        if mesh_elem is not None:
                            geom = ET.SubElement(child_body, 'geom')
                            mesh_file = mesh_elem.get('filename', '')
                            geom.set('type', 'mesh')
                            geom.set('mesh', mesh_file.replace('.stl', ''))

                            # Add mesh to assets
                            mesh_asset = ET.SubElement(asset, 'mesh')
                            mesh_asset.set('name', mesh_file.replace('.stl', ''))
                            mesh_asset.set('file', mesh_file)

                # Recursively add children
                add_children(child_body, child_link_name)

    # Start from base link
    add_children(torso_body, base_link)

    # Add actuators
    actuator = ET.SubElement(mujoco, 'actuator')
    for joint in joint_list:
        if joint.get('type') in ['revolute', 'continuous']:
            motor = ET.SubElement(actuator, 'position')
            motor.set('name', f"{joint.get('name')}_motor")
            motor.set('joint', joint.get('name'))
            motor.set('kp', '100')

    # Write output
    tree_out = ET.ElementTree(mujoco)
    ET.indent(tree_out, space='  ')
    tree_out.write(output_mjcf, encoding='utf-8', xml_declaration=True)

    print(f"\n✓ MJCF created: {output_mjcf}")
    return output_mjcf

if __name__ == '__main__':
    urdf_path = 'src/robot_description/humanoid_description/urdf/robot.urdf'
    output_mjcf = 'src/robot_description/humanoid_description/urdf/humanoid.xml'

    if not os.path.exists(urdf_path):
        print(f"✗ URDF not found: {urdf_path}")
        exit(1)

    create_mujoco_wrapper_for_urdf(urdf_path, output_mjcf)

    print("\n" + "=" * 60)
    print("Testing MuJoCo load...")
    print("=" * 60)

    try:
        import mujoco
        model = mujoco.MjModel.from_xml_path(output_mjcf)
        print(f"✓ SUCCESS! Model loaded in MuJoCo!")
        print(f"  Bodies: {model.nbody}")
        print(f"  Joints: {model.njnt}")
        print(f"  DOFs: {model.nv}")
        print(f"  Actuators: {model.nu}")

        print(f"\n✓ Ready to use!")
        print(f"\nTest with: python3 -m mujoco.viewer --mjcf={output_mjcf}")

    except Exception as e:
        print(f"✗ Error loading in MuJoCo: {e}")
        import traceback
        traceback.print_exc()
