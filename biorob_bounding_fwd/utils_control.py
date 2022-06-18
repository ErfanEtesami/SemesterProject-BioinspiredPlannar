import pybullet as pb
import numpy as np


def change_plane_dynamics(robot):
    """
    Change the dynamics (friction coefficient and restitution) of the plane.
    """
    pb.changeDynamics(bodyUniqueId=robot.plane_id,
                      linkIndex=-1,
                      lateralFriction=robot.mu_plane,
                      restitution=robot.rest_plane)


def change_robot_dynamics(robot):
    """
    Change the dynamics (friction coefficient and restitution) of the robot's links.
    """
    for j in robot.links:
        pb.changeDynamics(bodyUniqueId=robot.robot_id,
                          linkIndex=j,
                          lateralFriction=robot.mu_robot,
                          restitution=robot.rest_robot)
    return


def disable_default_control(robot):
    """
    Disable the default PyBullet velocity control in the robot's joints.
    """
    pb.setJointMotorControlArray(bodyUniqueId=robot.robot_id,
                                 jointIndices=robot.joints,
                                 controlMode=pb.VELOCITY_CONTROL,
                                 forces=np.zeros(robot.nb_joints))
    return


def check_td(robot, mode_leg):
    """
    Check whether the toes touch down the ground or not.
    """
    if mode_leg == 1:       # front legs
        temp_l = pb.getContactPoints(bodyA=robot.robot_id, bodyB=robot.plane_id,
                                     linkIndexA=robot.fl_foot, linkIndexB=-1)
        temp_r = pb.getContactPoints(bodyA=robot.robot_id, bodyB=robot.plane_id,
                                     linkIndexA=robot.fr_foot, linkIndexB=-1)
    elif mode_leg == 2:     # back legs
        temp_l = pb.getContactPoints(bodyA=robot.robot_id, bodyB=robot.plane_id,
                                     linkIndexA=robot.bl_foot, linkIndexB=-1)
        temp_r = pb.getContactPoints(bodyA=robot.robot_id, bodyB=robot.plane_id,
                                     linkIndexA=robot.br_foot, linkIndexB=-1)
    if len(temp_l) > 0 or len(temp_r) > 0:
        return True
    else:
        return False


def clamp_x_force(force_x, force_z, mu):
    """
    Clamp x force by |F_x| <= mu * |F_z|.
    """
    if abs(force_x) > mu*abs(force_z) and force_x != 0:
        force_x = mu * abs(force_z) * (force_x/abs(force_x))
    return force_x


def calc_fwd_kin_dir(robot, q, vq):
    """
    Calculate the direct forward kinematics of the leg.
    """
    # modify angles' references
    q0 = q[0] + np.pi/4
    q2 = q[1] + np.pi/2
    vq0 = vq[0]
    vq2 = vq[1]
    # calculate x and vx of the toe w.r.t. the hip joint
    x = (robot.l_thigh+robot.l_foot2)*np.cos(q0) + robot.l_calf*np.cos(q0+q2)
    vx = -(robot.l_thigh+robot.l_foot2)*np.sin(q0)*vq0 + -robot.l_calf*np.sin(q0+q2)*(vq0+vq2)
    # calculate z and vz of the toe w.r.t. the hip joint
    z = (robot.l_thigh+robot.l_foot2)*np.sin(q0) + robot.l_calf*np.sin(q0+q2)
    vz = (robot.l_thigh+robot.l_foot2)*np.cos(q0)*vq0 + robot.l_calf*np.cos(q0+q2)*(vq0+vq2)
    # calculate the leg's jacobian
    j_11 = -(robot.l_thigh+robot.l_foot2)*np.sin(q0) + -robot.l_calf*np.sin(q0+q2)
    j_12 = -robot.l_calf*np.sin(q0+q2)
    j_21 = (robot.l_thigh+robot.l_foot2)*np.cos(q0) + robot.l_calf*np.cos(q0+q2)
    j_22 = robot.l_calf*np.cos(q0+q2)
    jac = np.array([[j_11, j_12], [j_21, j_22]])
    return x, vx, z, vz, jac


def calc_fwd_kin_eqv(robot, q, vq):
    """
    Calculate the equivalent forward kinematics of the leg.
    """
    # modify angles' references
    q0 = q[0] + np.pi/4
    q2 = q[1] + np.pi/2
    q3 = q2
    vq0 = vq[0]
    vq2 = vq[1]
    vq3 = vq2
    # calculate equivalent leg length
    l = np.sqrt(robot.l_calf**2 + robot.l_foot2**2 + 2*robot.l_calf*robot.l_foot2*np.cos(q3))
    dl_dq3 = (-2*robot.l_calf*robot.l_foot2*np.sin(q3)) / (2*l)
    dl_dt = dl_dq3 * vq3
    # calculate alpha angle
    u = (robot.l_foot2/l) * np.sin(np.pi-q3)
    du_dq3 = -(robot.l_foot2/l)*np.cos(np.pi-q3) + -(robot.l_foot2/(l**2))*np.sin(np.pi-q3)*dl_dq3
    du_dt = -(robot.l_foot2/l)*np.cos(np.pi-q3)*vq3 + -(robot.l_foot2/(l**2))*np.sin(np.pi-q3)*dl_dt
    alpha = np.arcsin(u)
    dalpha_dq3 = (1/np.sqrt(1-(u**2))) * du_dq3
    dalpha_dt = (1/np.sqrt(1-(u**2))) * du_dt
    # calculate x and vx of the toe w.r.t. the hip joint
    x = robot.l_thigh*np.cos(q0) + l*np.cos(q0+q2-alpha)
    vx = -robot.l_thigh*np.sin(q0)*vq0 + dl_dt*np.cos(q0+q2-alpha) + -l*np.sin(q0+q2-alpha)*(vq0+vq2-dalpha_dt)
    # calculate z and vz of the toe w.r.t. the hip joint
    z = robot.l_thigh*np.sin(q0) + l*np.sin(q0+q2-alpha)
    vz = robot.l_thigh*np.cos(q0)*vq0 + dl_dt*np.sin(q0+q2-alpha) + l*np.cos(q0+q2-alpha)*(vq0+vq2-dalpha_dt)
    # calculate the leg's jacobian
    j_11 = -robot.l_thigh*np.sin(q0) + -l*np.sin(q0+q2-alpha)
    j_12 = dl_dq3*np.cos(q0+q2-alpha) + -l*np.sin(q0+q2-alpha)*(1-dalpha_dq3)
    j_21 = robot.l_thigh*np.cos(q0) + l*np.cos(q0+q2-alpha)
    j_22 = dl_dq3*np.sin(q0+q2-alpha) + l*np.cos(q0+q2-alpha)*(1-dalpha_dq3)
    jac = np.array([[j_11, j_12], [j_21, j_22]])
    return x, vx, z, vz, jac


def calc_fwd_kin(robot, flag_fwd_kin, q, vq):
    """
    Calculate the forward kinematics of the leg.
    """
    if flag_fwd_kin == 0:       # direct forward kinematics
        x_l, vx_l, z_l, vz_l, jac_l = calc_fwd_kin_dir(robot, [q[0], q[1]], [vq[0], vq[1]])
        x_r, vx_r, z_r, vz_r, jac_r = calc_fwd_kin_dir(robot, [q[2], q[3]], [vq[2], vq[3]])
    elif flag_fwd_kin == 1:     # equivalent forward kinematics
        x_l, vx_l, z_l, vz_l, jac_l = calc_fwd_kin_eqv(robot, [q[0], q[1]], [vq[0], vq[1]])
        x_r, vx_r, z_r, vz_r, jac_r = calc_fwd_kin_eqv(robot, [q[2], q[3]], [vq[2], vq[3]])
    return x_l, vx_l, z_l, vz_l, jac_l, x_r, vx_r, z_r, vz_r, jac_r


def calc_fb(robot, mode_control, x, x_d, vx, vx_d, z, z_d, vz, vz_d, th, th_d, wth, wth_d):
    """
    Calculate the feedback.
    """
    if mode_control == 1:       # initialization
        kp_x = robot.kp_x_init
        kd_x = robot.kd_x_init
        kp_z = robot.kp_z_init
        kd_z = robot.kd_z_init
        kp_th = robot.kp_th_init
        kd_th = robot.kd_th_init
    elif mode_control == 2:     # stance
        kp_x = robot.kp_x_st
        kd_x = robot.kd_x_st
        kp_z = robot.kp_z_st
        kd_z = robot.kd_z_st
        kp_th = robot.kp_th_st
        kd_th = robot.kd_th_st
    elif mode_control == 3:     # swing
        kp_x = robot.kp_x_sw
        kd_x = robot.kd_x_sw
        kp_z = robot.kp_z_sw
        kd_z = robot.kd_z_sw
        kp_th = robot.kp_th_sw
        kd_th = robot.kd_th_sw
    fb_x_p = -kp_x * (x - x_d)
    fb_x_d = -kd_x * (vx - vx_d)
    fb_z_p = -kp_z * (z - z_d)
    fb_z_d = -kd_z * (vz - vz_d)
    fb_th_p = -kp_th * (th - th_d)
    fb_th_d = -kd_th * (wth - wth_d)
    fb_x_pd = fb_x_p + fb_x_d
    fb_z_pd = fb_z_p + fb_z_d
    fb_th_pd = fb_th_p + fb_th_d
    return fb_x_pd, fb_x_p, fb_x_d, fb_z_pd, fb_z_p, fb_z_d, fb_th_pd, fb_th_p, fb_th_d


def calc_torques(jac_l, force_l, jac_r, force_r, torque_sat):
    """
    Calculate the joints torques (joint torques = jacobian.transpose * toe force)
    """
    torques_l = np.matmul(np.transpose(jac_l), force_l)
    torques_r = np.matmul(np.transpose(jac_r), force_r)
    torques = np.vstack((torques_l, torques_r))
    for j in range(len(torques)):
        if abs(torques[j]) > torque_sat:
            torques[j] = torque_sat * (torques[j]/abs(torques[j]))
    return torques


def apply_torques(robot, mode_leg, torques):
    """
    Apply the calculated torques to the joints.
    """
    if mode_leg == 1:       # front legs
        pb.setJointMotorControlArray(bodyUniqueId=robot.robot_id,
                                     jointIndices=robot.front_joints,
                                     controlMode=pb.TORQUE_CONTROL,
                                     forces=[torques[0].item(), 0.0, torques[1].item(), 0.0,
                                             torques[2].item(), 0.0, torques[3].item(), 0.0])
    elif mode_leg == 2:     # back legs
        pb.setJointMotorControlArray(bodyUniqueId=robot.robot_id,
                                     jointIndices=robot.back_joints,
                                     controlMode=pb.TORQUE_CONTROL,
                                     forces=[torques[0].item(), 0.0, torques[1].item(), 0.0,
                                             torques[2].item(), 0.0, torques[3].item(), 0.0])
    return


def apply_control(robot, mode_leg, mode_control,
                  x_l, z_l, vx_l, vz_l, x_r, z_r, vx_r, vz_r, jac_l, jac_r, th, wth,
                  x_body_l, x_body_r, z_body_l, z_body_r,
                  x_d, vx_d, z_d, vz_d, th_d, wth_d,
                  cnt_x_l, cnt_x_r, cnt_z_l, cnt_z_r,
                  torque_sat, afb, mu, flag_clamp_x_force):
    """
    Calculate the total toe force (contact + feedback) and the corresponding joint torques.
    Then apply the torques to the joints.
    """
    # calculate feedback
    fb_x_pd_l, fb_x_p_l, fb_x_d_l, fb_z_pd_l, fb_z_p_l, fb_z_d_l, fb_th_pd_l, fb_th_p_l, fb_th_d_l = \
        calc_fb(robot, mode_control, x_l, x_d, vx_l, vx_d, z_l, z_d, vz_l, vz_d, th, th_d, wth, wth_d)
    fb_x_pd_r, fb_x_p_r, fb_x_d_r, fb_z_pd_r, fb_z_p_r, fb_z_d_r, fb_th_pd_r, fb_th_p_r, fb_th_d_r = \
        calc_fb(robot, mode_control, x_r, x_d, vx_r, vx_d, z_r, z_d, vz_r, vz_d, th, th_d, wth, wth_d)
    fb_tot_x_l = fb_x_pd_l + fb_th_pd_l/z_body_l
    fb_tot_z_l = afb*fb_z_pd_l + fb_th_pd_l/x_body_l
    fb_tot_x_r = fb_x_pd_r + fb_th_pd_r/z_body_r
    fb_tot_z_r = afb*fb_z_pd_r + fb_th_pd_r/x_body_r
    # calculate the total force profiles in x and z directions
    action_x_l = cnt_x_l + fb_tot_x_l
    action_z_l = cnt_z_l + fb_tot_z_l
    action_x_r = cnt_x_r + fb_tot_x_r
    action_z_r = cnt_z_r + fb_tot_z_r
    # clamp the x force profile if required
    if flag_clamp_x_force == 1:
        action_x_l = clamp_x_force(action_x_l, action_z_l, mu)
        action_x_r = clamp_x_force(action_x_r, action_z_r, mu)
    # calculate the joint torques and apply them
    action_l = np.array([[action_x_l], [action_z_l]])
    action_r = np.array([[action_x_r], [action_z_r]])
    torques = calc_torques(jac_l, action_l, jac_r, action_r, torque_sat)
    apply_torques(robot, mode_leg, torques)
    return fb_x_pd_l, fb_x_p_l, fb_x_d_l, fb_z_pd_l, fb_z_p_l, fb_z_d_l, fb_th_pd_l, fb_th_p_l, fb_th_d_l, \
           fb_x_pd_r, fb_x_p_r, fb_x_d_r, fb_z_pd_r, fb_z_p_r, fb_z_d_r, fb_th_pd_r, fb_th_p_r, fb_th_d_r, \
           fb_tot_x_l, fb_tot_z_l, fb_tot_x_r, fb_tot_z_r, \
           action_l[0], action_l[1], action_r[0], action_r[1], \
           torques[0], torques[1], torques[2], torques[3]
