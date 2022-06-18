import numpy as np
import pybullet as pb


def get_joint_states(robot, mode_leg):
    """
    Read the angular position and velocity of the actuated joints.
    """
    if mode_leg == 1:       # front legs
        # left leg
        hip_l_idx = robot.fl_hip
        knee_l_idx = robot.fl_knee
        # right leg
        hip_r_idx = robot.fr_hip
        knee_r_idx = robot.fr_knee
    elif mode_leg == 2:     # back legs
        # left leg
        hip_l_idx = robot.bl_hip
        knee_l_idx = robot.bl_knee
        # right leg
        hip_r_idx = robot.br_hip
        knee_r_idx = robot.br_knee
    # left leg
    hip_l = pb.getJointState(bodyUniqueId=robot.robot_id, jointIndex=hip_l_idx)
    knee_l = pb.getJointState(bodyUniqueId=robot.robot_id, jointIndex=knee_l_idx)
    # right leg
    hip_r = pb.getJointState(bodyUniqueId=robot.robot_id, jointIndex=hip_r_idx)
    knee_r = pb.getJointState(bodyUniqueId=robot.robot_id, jointIndex=knee_r_idx)
    # angular position and velocity of the joints
    q = np.array([hip_l[0], knee_l[0], hip_r[0], knee_r[0]])
    vq = np.array([hip_l[1], knee_l[1], hip_r[1], knee_r[1]])
    return q, vq


def get_com_crds(robot):
    """
    Read the coordinates of the robot's CoM, including x, y, z, th (pitch angle), vx, vy, wth (pitch velocity).
    """
    pos, orn = pb.getBasePositionAndOrientation(bodyUniqueId=robot.robot_id)
    x_com, y_com, z_com = pos[0], pos[1], pos[2]
    orn_euler = pb.getEulerFromQuaternion(orn)
    th_com = orn_euler[1]
    lin_vel, ang_vel = pb.getBaseVelocity(bodyUniqueId=robot.robot_id)
    vx_com, vz_com, wth_com = lin_vel[0], lin_vel[2], ang_vel[1]
    return x_com, y_com, z_com, th_com, vx_com, vz_com, wth_com


def calc_body_crds(robot, mode_leg, x, vx, z, vz, th_com, wth_com):
    """
    Calculate the relative coordinates of the CoM w.r.t. the toes.
    """
    if mode_leg == 1:       # front legs
        x_body = -(robot.l_body/2)*np.cos(th_com) - x
        vx_body = (robot.l_body/2)*np.sin(th_com)*wth_com - vx
        z_body = (robot.l_body/2)*np.sin(th_com) + z
        vz_body = (robot.l_body/2)*np.cos(th_com)*wth_com + vz
    elif mode_leg == 2:     # back legs
        x_body = (robot.l_body/2)*np.cos(th_com) - x
        vx_body = -(robot.l_body/2)*np.sin(th_com)*wth_com - vx
        z_body = -(robot.l_body/2)*np.sin(th_com) + z
        vz_body = -(robot.l_body/2)*np.cos(th_com)*wth_com + vz
    return x_body, z_body, vx_body, vz_body


def get_toe_forces(robot):
    """
    Read the forces on the toes as they contact the ground.
    """
    fl_toe_force = np.array([[0], [0]])
    fr_toe_force = np.array([[0], [0]])
    bl_toe_force = np.array([[0], [0]])
    br_toe_force = np.array([[0], [0]])
    temp_fl = pb.getContactPoints(bodyA=robot.robot_id, bodyB=robot.plane_id,
                                  linkIndexA=robot.fl_foot, linkIndexB=-1)
    temp_fr = pb.getContactPoints(bodyA=robot.robot_id, bodyB=robot.plane_id,
                                  linkIndexA=robot.fr_foot, linkIndexB=-1)
    temp_bl = pb.getContactPoints(bodyA=robot.robot_id, bodyB=robot.plane_id,
                                  linkIndexA=robot.bl_foot, linkIndexB=-1)
    temp_br = pb.getContactPoints(bodyA=robot.robot_id, bodyB=robot.plane_id,
                                  linkIndexA=robot.br_foot, linkIndexB=-1)
    if len(temp_fl) > 0:
        for item in temp_fl:
            fl_toe_force[0] = fl_toe_force[0] + item[12] * item[13][0]
            fl_toe_force[1] = fl_toe_force[1] + item[9] * item[7][2]
    if len(temp_fr) > 0:
        for item in temp_fr:
            fr_toe_force[0] = fr_toe_force[0] + item[12] * item[13][0]
            fr_toe_force[1] = fr_toe_force[1] + item[9] * item[7][2]
    if len(temp_bl) > 0:
        for item in temp_bl:
            bl_toe_force[0] = bl_toe_force[0] + item[12] * item[13][0]
            bl_toe_force[1] = bl_toe_force[1] + item[9] * item[7][2]
    if len(temp_br) > 0:
        for item in temp_br:
            br_toe_force[0] = br_toe_force[0] + item[12] * item[13][0]
            br_toe_force[1] = br_toe_force[1] + item[9] * item[7][2]
    return fl_toe_force[0], fl_toe_force[1], fr_toe_force[0], fr_toe_force[1],\
           bl_toe_force[0], bl_toe_force[1], br_toe_force[0], br_toe_force[1]


def enable_torque_sensors(robot):
    """
    Enable PyBullet torque sensors in the actuated joints.
    """
    for j in robot.act_joints:
        pb.enableJointForceTorqueSensor(bodyUniqueId=robot.robot_id, jointIndex=j)
    return
