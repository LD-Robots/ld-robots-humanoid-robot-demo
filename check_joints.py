import mujoco

m = mujoco.MjModel.from_xml_path('src/control/lipm_walking_controller/models/robot.mjcf')
print('Joint ranges:')
for i in range(m.njnt):
    name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
    if name != 'floating_base':
        jnt_type = m.jnt_type[i]
        if jnt_type == mujoco.mjtJoint.mjJNT_HINGE:
            jnt_limited = m.jnt_limited[i]
            if jnt_limited:
                range_vals = m.jnt_range[i]
                print(f'{name}: [{range_vals[0]:.3f}, {range_vals[1]:.3f}] rad = [{range_vals[0]*57.3:.1f}, {range_vals[1]*57.3:.1f}] deg')
