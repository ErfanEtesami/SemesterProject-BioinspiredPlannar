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
        self.robot_id = pb.loadURDF(fileName='./model/biorob.urdf',
                                    basePosition=self.start_pos,
                                    baseOrientation=pb.getQuaternionFromEuler(self.start_orn),
                                    useFixedBase=self.flag_fix_base,
                                    flags=pb.URDF_USE_INERTIA_FROM_FILE & pb.URDF_MAINTAIN_LINK_ORDER)
        # link lengths
        self.lengths = lengths
        self.l_thigh = self.lengths[0]
        self.l_calf = self.lengths[1]
        self.l_foot1 = self.lengths[2]
        self.l_foot2 = self.lengths[3]
        # link indices
        self.base = -1
        self.fl_thigh = 0
        self.fl_rod = 1
        self.fl_calf = 2
        self.fl_foot = 3
        self.fr_thigh = 4
        self.fr_rod = 5
        self.fr_calf = 6
        self.fr_foot = 7
        self.bl_thigh = 8
        self.bl_rod = 9
        self.bl_calf = 10
        self.bl_foot = 11
        self.br_thigh = 12
        self.br_rod = 13
        self.br_calf = 14
        self.br_foot = 15
        # link arrays
        self.links = [self.base,
                      self.fl_thigh, self.fl_rod, self.fl_calf, self.fl_foot,
                      self.fr_thigh, self.fr_rod, self.fr_calf, self.fr_foot,
                      self.bl_thigh, self.bl_rod, self.bl_calf, self.bl_foot,
                      self.br_thigh, self.br_rod, self.br_calf, self.br_foot]
        self.nb_links = len(self.links)
        # joint indices
        self.fl_hip = 0
        self.fl_thigh_rod = 1
        self.fl_knee = 2
        self.fl_ankle = 3
        self.fr_hip = 4
        self.fr_thigh_rod = 5
        self.fr_knee = 6
        self.fr_ankle = 7
        self.bl_hip = 8
        self.bl_thigh_rod = 9
        self.bl_knee = 10
        self.bl_ankle = 11
        self.br_hip = 12
        self.br_thigh_rod = 13
        self.br_knee = 14
        self.br_ankle = 15
        # joint arrays
        self.joints = [self.fl_hip, self.fl_thigh_rod, self.fl_knee, self.fl_ankle,
                       self.fr_hip, self.fr_thigh_rod, self.fr_knee, self.fr_ankle,
                       self.bl_hip, self.bl_thigh_rod, self.bl_knee, self.bl_ankle,
                       self.br_hip, self.br_thigh_rod, self.br_knee, self.br_ankle]
        self.front_joints = [self.fl_hip, self.fl_thigh_rod, self.fl_knee, self.fl_ankle,
                             self.fr_hip, self.fr_thigh_rod, self.fr_knee, self.fr_ankle]
        self.back_joints = [self.bl_hip, self.bl_thigh_rod, self.bl_knee, self.bl_ankle,
                            self.br_hip, self.br_thigh_rod, self.br_knee, self.br_ankle]
        self.act_joints = [self.fl_hip, self.fl_knee, self.fr_hip, self.fr_knee,
                           self.bl_hip, self.bl_knee, self.br_hip, self.br_knee]
        self.nb_joints = len(self.joints)
        self.nb_act_joints = len(self.act_joints)
        # define constraints
        self.cst1 = pb.createConstraint(parentBodyUniqueId=self.robot_id,
                                        parentLinkIndex=self.fl_rod,
                                        childBodyUniqueId=self.robot_id,
                                        childLinkIndex=self.fl_foot,
                                        jointType=pb.JOINT_POINT2POINT,
                                        jointAxis=[0, 0, 0],
                                        parentFramePosition=[0, self.l_calf/2, 0],
                                        childFramePosition=[0, -0.09079, -0.01014])
        self.cst2 = pb.createConstraint(parentBodyUniqueId=self.robot_id,
                                        parentLinkIndex=self.fr_rod,
                                        childBodyUniqueId=self.robot_id,
                                        childLinkIndex=self.fr_foot,
                                        jointType=pb.JOINT_POINT2POINT,
                                        jointAxis=[0, 0, 0],
                                        parentFramePosition=[0, self.l_calf/2, 0],
                                        childFramePosition=[0, -0.09079, -0.01014])
        self.cst3 = pb.createConstraint(parentBodyUniqueId=self.robot_id,
                                        parentLinkIndex=self.bl_rod,
                                        childBodyUniqueId=self.robot_id,
                                        childLinkIndex=self.bl_foot,
                                        jointType=pb.JOINT_POINT2POINT,
                                        jointAxis=[0, 0, 0],
                                        parentFramePosition=[0, self.l_calf/2, 0],
                                        childFramePosition=[0, -0.09079, -0.01014])
        self.cst4 = pb.createConstraint(parentBodyUniqueId=self.robot_id,
                                        parentLinkIndex=self.br_rod,
                                        childBodyUniqueId=self.robot_id,
                                        childLinkIndex=self.br_foot,
                                        jointType=pb.JOINT_POINT2POINT,
                                        jointAxis=[0, 0, 0],
                                        parentFramePosition=[0, self.l_calf/2, 0],
                                        childFramePosition=[0, -0.09079, -0.01014])
