import pybullet as pb


class BioRob:
    def __init__(self, plane_id,
                 m, g, lengths, l_base,
                 mu_robot, rest_robot, mu_plane, rest_plane,
                 kp_x_init, kd_x_init, kp_z_init, kd_z_init, kp_th_init, kd_th_init,
                 kp_x_st, kd_x_st, kp_z_st, kd_z_st, kp_th_st, kd_th_st,
                 kp_x_sw, kd_x_sw, kp_z_sw, kd_z_sw, kp_th_sw, kd_th_sw,
                 k_st,
                 start_pos, start_orn,
                 flag_fix_base):
        # general parameters
        self.plane_id = plane_id
        self.m = m
        self.g = g
        self.l_base = l_base
        self.mu_robot = mu_robot
        self.rest_robot = rest_robot
        self.mu_plane = mu_plane
        self.rest_plane = rest_plane
        self.start_pos = start_pos
        self.start_orn = start_orn
        # control gains
        self.kp_x_init = kp_x_init
        self.kd_x_init = kd_x_init
        self.kp_z_init = kp_z_init
        self.kd_z_init = kd_z_init
        self.kp_th_init = kp_th_init
        self.kd_th_init = kd_th_init
        self.kp_x_st = kp_x_st
        self.kd_x_st = kd_x_st
        self.kp_z_st = kp_z_st
        self.kd_z_st = kd_z_st
        self.kp_th_st = kp_th_st
        self.kd_th_st = kd_th_st
        self.kp_x_sw = kp_x_sw
        self.kd_x_sw = kd_x_sw
        self.kp_z_sw = kp_z_sw
        self.kd_z_sw = kd_z_sw
        self.kp_th_sw = kp_th_sw
        self.kd_th_sw = kd_th_sw
        self.k_st = k_st
        # flags
        self.flag_fix_base = flag_fix_base
        # load robot
        self.robot_id = pb.loadURDF(fileName='./model/solo.urdf',
                                    basePosition=self.start_pos,
                                    baseOrientation=pb.getQuaternionFromEuler(self.start_orn),
                                    useFixedBase=self.flag_fix_base,
                                    flags=pb.URDF_USE_INERTIA_FROM_FILE & pb.URDF_MAINTAIN_LINK_ORDER)
        # link lengths
        self.lengths = lengths
        self.l_upper = self.lengths[0]
        self.l_lower = self.lengths[1]
        # link indices
        self.base = -1
        self.fl_upper = 0
        self.fl_lower = 1
        self.fl_toe = 2
        self.fr_upper = 3
        self.fr_lower = 4
        self.fr_toe = 5
        self.bl_upper = 6
        self.bl_lower = 7
        self.bl_toe = 8
        self.br_upper = 9
        self.br_lower = 10
        self.br_toe = 11
        # link arrays
        self.links = [self.base,
                      self.fl_upper, self.fl_lower, self.fl_toe,
                      self.fr_upper, self.fr_lower, self.fr_toe,
                      self.bl_upper, self.bl_lower, self.bl_toe,
                      self.br_upper, self.br_lower, self.br_toe]
        self.nb_links = len(self.links)
        # joint indices
        self.fl_hip = 0
        self.fl_knee = 1
        self.fl_ankle = 2
        self.fr_hip = 3
        self.fr_knee = 4
        self.fr_ankle = 5
        self.bl_hip = 6
        self.bl_knee = 7
        self.bl_ankle = 8
        self.br_hip = 9
        self.br_knee = 10
        self.br_ankle = 11
        # joint arrays
        self.joints = [self.fl_hip, self.fl_knee, self.fl_ankle,
                       self.fr_hip, self.fr_knee, self.fr_ankle,
                       self.bl_hip, self.bl_knee, self.bl_ankle,
                       self.br_hip, self.br_knee, self.br_ankle]
        self.front_revolute_joints = [self.fl_hip, self.fl_knee, self.fr_hip, self.fr_knee]
        self.back_revolute_joints = [self.bl_hip, self.bl_knee, self.br_hip, self.br_knee]
        self.act_joints = [self.fl_hip, self.fl_knee, self.fr_hip, self.fr_knee,
                           self.bl_hip, self.bl_knee, self.br_hip, self.br_knee]
        self.nb_joints = len(self.joints)
        self.nb_act_joints = len(self.act_joints)
