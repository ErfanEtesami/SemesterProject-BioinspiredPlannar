# --- modules --- #
import pybullet as pb
import pybullet_data
import numpy as np
import time
import solo_class as solo_class
import utils_verbose as verbose
import utils_get_info as get_info
import utils_gen_curves as gen_curves
import utils_control as control
import utils_plot as plot


# --- nomenclature (sorted alphabetically) --- #
# !!! UNITS: kg, s, m, rad, m/s, rad/s, m/(s^2), N, N.m, Watt !!! #
# act: actuated
# action: total force
# afb: feedback activation
# b: back
# beta: Bezier control points
# bl: back left
# body_crds: relative coordinates of the CoM w.r.t. the toe
# br: back right
# calc: calculate
# cnt: contact force profile
# coeffs: coefficients
# com (CoM): center of mass
# config: configuration
# crds: coordinates
# crr: corrected
# d: desired or derivative
# dir: direct
# eqv: equivalent
# f: front
# fb: feedback
# fl: front left
# fr: front right
# fwd: forward
# gen: generate
# init: initialization
# ip: in place
# jac: jacobian
# kin: kinematics
# mdf: modified
# mech: mechanical
# mu: friction coefficient
# nb: number of
# org: original
# p: proportional
# poly: polynomial
# pow: power
# rest: restitution
# sm: state machine
# st: stance
# sw: swing
# t: time
# td: touch down
# tf: toe force
# th: (theta) pitch angle
# tot: total
# traj: trajectory
# wth: (omega theta) pitch angular velocity


# --- flags --- #
flag_fix_base = 0                   # 1: fix the base on air or not (0)
flag_change_robot_dynamics = 1      # 1: change dynamics of the robot or not (0)
flag_change_plane_dynamics = 1      # 1: change dynamics of the plane or not (0)
flag_use_afb = 1                    # 1: use feedback activation such that F_z > 0 or not (0)
flag_clamp_x_force = 0              # 1: clamp x force by |F_x| <= mu * |F_z| or not (0)
flag_update_t_st = 0                # 1: update stance time or not (0)
flag_record_video = 1               # 1: record video or not (0)
flag_draw_traj = 1                  # 1: draw trajectory in gui or not (0)
draw_step = 50
flag_slow_motion = 0                # 1: slow down the simulation or not (0)
t_slow_motion = 10*1e-3
# check flag inputs
verbose.check_flags([flag_fix_base, flag_change_robot_dynamics, flag_change_plane_dynamics,
                     flag_use_afb, flag_clamp_x_force, flag_update_t_st, flag_record_video,
                     flag_draw_traj, flag_slow_motion])
# --- time constants --- #
t_step = 1*1e-3             # PyBullet time step of simulation
t_init = 1                  # initialization time for applying the initial configuration
nb_periods = 10             # number of simulation gait periods
t_st = 0.110                # stance time
t_sw = 0.220                # swing time
# --- robot constants --- #
m = 1.43315091 + 4*(0.14853845+0.03070001+0.00693606)    # 2.17784899
lengths = [0.160, 0.160]                    # length of the upper and lower parts of the leg
l_base = 0.3892                             # length of the main body of the robot
l_span = 0.57*l_base                        # stroke length
g = 9.81                                    # gravity
torque_sat = 2.7                            # saturation joint torques
# --- friction and restitution --- #
mu_plane = 0.8
rest_plane = 0.5
mu_robot = 1
rest_robot = 0.5
mu = mu_plane * mu_robot
# --- start and initial configuration --- #
if flag_fix_base == 1:
    start_pos = [0, 0, 0.32+0.1]
else:
    start_pos = [0, 0, 0.32+0.02]
start_orn = [0, 0, 0]
x_init_f = -0.005
x_init_b = 0.005
z_init = 0.25
vx_init = 0
vz_init = 0
th_init = 0
wth_init = 0
# --- control gains --- #
# updating t_st
k_st = 0.1
if flag_fix_base == 0:
    # initialization
    kp_x_init = 200
    kd_x_init = 10
    kp_z_init = 200
    kd_z_init = 10
    kp_th_init = 0
    kd_th_init = 0
    # stance
    kp_x_st = 160
    kd_x_st = 14
    kp_z_st = 250
    kd_z_st = 10
    kp_th_st = 0
    kd_th_st = 0
    # swing
    kp_x_sw = 160
    kd_x_sw = 14
    kp_z_sw = 250
    kd_z_sw = 10
    kp_th_sw = 0
    kd_th_sw = 0
elif flag_fix_base == 1:
    # initialization
    kp_x_init = 200
    kd_x_init = 10
    kp_z_init = 200
    kd_z_init = 10
    kp_th_init = 0
    kd_th_init = 0
    # stance
    kp_x_st = 60
    kd_x_st = 12
    kp_z_st = 150
    kd_z_st = 10
    kp_th_st = 0
    kd_th_st = 0
    # swing
    kp_x_sw = 60
    kd_x_sw = 12
    kp_z_sw = 150
    kd_z_sw = 10
    kp_th_sw = 0
    kd_th_sw = 0
# --- temporal gait parameters --- #
v_d = 0
t_st_org = t_st
x_st_d = 0.00
z_st_d = 0.18
th_st_d = 0.00
vx_st_d = -v_d
vz_st_d = 0.00
wth_st_d = 0.00
t_st_array = np.arange(0, t_st, t_step)
t_air = (t_sw-t_st)/2
t_stride = t_st+t_sw
t_st_front = t_st
t_st_back = t_st
t_air_front = (t_sw-t_st_front)/2
t_air_back = (t_sw-t_st_back)/2
t_stride_front = t_st_front+t_sw
t_stride_back = t_st_back+t_sw
t_st_front_array = np.arange(0, t_st_front, t_step)
t_st_back_array = np.arange(0, t_st_back, t_step)
t_sw_array = np.arange(0, t_sw, t_step)
# --- generate contact profiles --- #
ratio_alpha_z = 0.5         # location of the peak of the contact force in terms of finished percentage of t_stance
ratio_cnt_xz = 0.0          # ration of the x contact force w.r.t. the z contact force
t_alpha_z_front = t_st_front*ratio_alpha_z
t_alpha_x_front = t_st_front*0.5
t_alpha_z_back = t_st_back*ratio_alpha_z
t_alpha_x_back = t_st_back*0.5
alpha_cnt_z_front = (m*g*t_stride_front)/(2*0.7*t_st_front)
alpha_cnt_z_back = (m*g*t_stride_back)/(2*0.7*t_st_back)
alpha_cnt_x_front = -alpha_cnt_z_front*ratio_cnt_xz
alpha_cnt_x_back = alpha_cnt_z_back*ratio_cnt_xz
beta_cnt_z_1 = np.array([0.0, 0.8, 1.0, 1.0])
beta_cnt_z_2 = np.array([1.0, 1.0, 0.8, 0.0])
beta_cnt_x_1 = np.array([0.0, 0.8, 1.0, 1.0])
beta_cnt_x_2 = np.array([1.0, 1.0, 0.8, 0.0])
cnt_z_front = gen_curves.gen_cnt(t_st_front, t_alpha_z_front, t_st_front_array,
                                 alpha_cnt_z_front*beta_cnt_z_1, alpha_cnt_z_front*beta_cnt_z_2)
cnt_z_back = gen_curves.gen_cnt(t_st_back, t_alpha_z_back, t_st_back_array, alpha_cnt_z_back*beta_cnt_z_1,
                                alpha_cnt_z_back*beta_cnt_z_2)
cnt_x_front = gen_curves.gen_cnt(t_st_front, t_alpha_x_front, t_st_front_array,
                                 alpha_cnt_x_front*beta_cnt_x_1, alpha_cnt_x_front*beta_cnt_x_2)
cnt_x_back = gen_curves.gen_cnt(t_st_back, t_alpha_x_back, t_st_back_array,
                                alpha_cnt_x_back*beta_cnt_x_1, alpha_cnt_x_back*beta_cnt_x_2)
plot.plot_cnt(m, g, t_st_front_array, t_st_back_array, cnt_x_front, cnt_x_back, cnt_z_front, cnt_z_back)
# --- generate feedback activation --- #
if flag_use_afb == 0:
    afb_front = np.ones(len(t_st_front_array))
    afb_back = np.ones(len(t_st_back_array))
elif flag_use_afb == 1:
    t_alpha_1_front = 0.2*t_st_front
    t_alpha_2_front = 0.8*t_st_front
    t_alpha_1_back = 0.2*t_st_back
    t_alpha_2_back = 0.8*t_st_back
    beta_afb_1 = np.array([0.0, 1.0, 1.0, 1.0])
    beta_afb_2 = np.array([1.0, 1.0, 1.0, 0.0])
    afb_front = gen_curves.gen_afb(t_st_front, t_alpha_1_front, t_alpha_2_front, t_st_front_array,
                                   1*beta_afb_1, 1*beta_afb_2)
    afb_back = gen_curves.gen_afb(t_st_back, t_alpha_1_back, t_alpha_2_back, t_st_back_array,
                                  1*beta_afb_1, 1*beta_afb_2)
    plot.plot_afb(t_st_front_array, t_st_back_array, afb_front, afb_back)
# --- generate swing original trajectory --- #
x_traj_sw_org = x_st_d * np.ones(len(t_sw_array))
z_traj_sw_org = z_st_d * np.ones(len(t_sw_array))
vx_traj_sw_org = np.gradient(x_traj_sw_org, t_step)
vz_traj_sw_org = np.gradient(z_traj_sw_org, t_step)
# --- initialize storing variables --- #
# time
time_array = np.arange(0, t_init+nb_periods*(t_st+t_sw), t_step)
time_len = len(time_array)
t_axis = np.full(time_len, np.nan)
t_st_front_axis = np.full(time_len, np.nan)
t_st_back_axis = np.full(time_len, np.nan)
# torques
torque_fl_hip_axis = np.full(time_len, np.nan)
torque_fl_knee_axis = np.full(time_len, np.nan)
torque_fr_hip_axis = np.full(time_len, np.nan)
torque_fr_knee_axis = np.full(time_len, np.nan)
torque_bl_hip_axis = np.full(time_len, np.nan)
torque_bl_knee_axis = np.full(time_len, np.nan)
torque_br_hip_axis = np.full(time_len, np.nan)
torque_br_knee_axis = np.full(time_len, np.nan)
# total forces
action_x_fl_axis = np.full(time_len, np.nan)
action_z_fl_axis = np.full(time_len, np.nan)
action_x_fr_axis = np.full(time_len, np.nan)
action_z_fr_axis = np.full(time_len, np.nan)
action_x_bl_axis = np.full(time_len, np.nan)
action_z_bl_axis = np.full(time_len, np.nan)
action_x_br_axis = np.full(time_len, np.nan)
action_z_br_axis = np.full(time_len, np.nan)
# contact profiles
cnt_x_fl_axis = np.full(time_len, np.nan)
cnt_z_fl_axis = np.full(time_len, np.nan)
cnt_x_fr_axis = np.full(time_len, np.nan)
cnt_z_fr_axis = np.full(time_len, np.nan)
cnt_x_bl_axis = np.full(time_len, np.nan)
cnt_z_bl_axis = np.full(time_len, np.nan)
cnt_x_br_axis = np.full(time_len, np.nan)
cnt_z_br_axis = np.full(time_len, np.nan)
# feedback forces
fb_tot_x_fl_axis = np.full(time_len, np.nan)
fb_x_pd_fl_axis = np.full(time_len, np.nan)
fb_x_p_fl_axis = np.full(time_len, np.nan)
fb_x_d_fl_axis = np.full(time_len, np.nan)
fb_tot_z_fl_axis = np.full(time_len, np.nan)
fb_z_pd_fl_axis = np.full(time_len, np.nan)
fb_z_p_fl_axis = np.full(time_len, np.nan)
fb_z_d_fl_axis = np.full(time_len, np.nan)
fb_th_pd_fl_axis = np.full(time_len, np.nan)
fb_th_p_fl_axis = np.full(time_len, np.nan)
fb_th_d_fl_axis = np.full(time_len, np.nan)
fb_tot_x_fr_axis = np.full(time_len, np.nan)
fb_x_pd_fr_axis = np.full(time_len, np.nan)
fb_x_p_fr_axis = np.full(time_len, np.nan)
fb_x_d_fr_axis = np.full(time_len, np.nan)
fb_tot_z_fr_axis = np.full(time_len, np.nan)
fb_z_pd_fr_axis = np.full(time_len, np.nan)
fb_z_p_fr_axis = np.full(time_len, np.nan)
fb_z_d_fr_axis = np.full(time_len, np.nan)
fb_th_pd_fr_axis = np.full(time_len, np.nan)
fb_th_p_fr_axis = np.full(time_len, np.nan)
fb_th_d_fr_axis = np.full(time_len, np.nan)
fb_tot_x_bl_axis = np.full(time_len, np.nan)
fb_x_pd_bl_axis = np.full(time_len, np.nan)
fb_x_p_bl_axis = np.full(time_len, np.nan)
fb_x_d_bl_axis = np.full(time_len, np.nan)
fb_tot_z_bl_axis = np.full(time_len, np.nan)
fb_z_pd_bl_axis = np.full(time_len, np.nan)
fb_z_p_bl_axis = np.full(time_len, np.nan)
fb_z_d_bl_axis = np.full(time_len, np.nan)
fb_th_pd_bl_axis = np.full(time_len, np.nan)
fb_th_p_bl_axis = np.full(time_len, np.nan)
fb_th_d_bl_axis = np.full(time_len, np.nan)
fb_tot_x_br_axis = np.full(time_len, np.nan)
fb_x_pd_br_axis = np.full(time_len, np.nan)
fb_x_p_br_axis = np.full(time_len, np.nan)
fb_x_d_br_axis = np.full(time_len, np.nan)
fb_tot_z_br_axis = np.full(time_len, np.nan)
fb_z_pd_br_axis = np.full(time_len, np.nan)
fb_z_p_br_axis = np.full(time_len, np.nan)
fb_z_d_br_axis = np.full(time_len, np.nan)
fb_th_pd_br_axis = np.full(time_len, np.nan)
fb_th_p_br_axis = np.full(time_len, np.nan)
fb_th_d_br_axis = np.full(time_len, np.nan)
# measured toe forces
tf_x_fl_axis = np.full(time_len, np.nan)
tf_z_fl_axis = np.full(time_len, np.nan)
tf_x_fr_axis = np.full(time_len, np.nan)
tf_z_fr_axis = np.full(time_len, np.nan)
tf_x_bl_axis = np.full(time_len, np.nan)
tf_z_bl_axis = np.full(time_len, np.nan)
tf_x_br_axis = np.full(time_len, np.nan)
tf_z_br_axis = np.full(time_len, np.nan)
# toe coordinates w.r.t. hip joint
x_fl_axis = np.full(time_len, np.nan)
x_fr_axis = np.full(time_len, np.nan)
x_bl_axis = np.full(time_len, np.nan)
x_br_axis = np.full(time_len, np.nan)
z_fl_axis = np.full(time_len, np.nan)
z_fr_axis = np.full(time_len, np.nan)
z_bl_axis = np.full(time_len, np.nan)
z_br_axis = np.full(time_len, np.nan)
vx_fl_axis = np.full(time_len, np.nan)
vx_fr_axis = np.full(time_len, np.nan)
vx_bl_axis = np.full(time_len, np.nan)
vx_br_axis = np.full(time_len, np.nan)
vz_fl_axis = np.full(time_len, np.nan)
vz_fr_axis = np.full(time_len, np.nan)
vz_bl_axis = np.full(time_len, np.nan)
vz_br_axis = np.full(time_len, np.nan)
# CoM coordinates
x_com_axis = np.full(time_len, np.nan)
y_com_axis = np.full(time_len, np.nan)
z_com_axis = np.full(time_len, np.nan)
th_com_axis = np.full(time_len, np.nan)
vx_com_axis = np.full(time_len, np.nan)
vz_com_axis = np.full(time_len, np.nan)
wth_com_axis = np.full(time_len, np.nan)
# CoM coordinates w.r.t. stance toe
x_body_fl_axis = np.full(time_len, np.nan)
x_body_fr_axis = np.full(time_len, np.nan)
x_body_bl_axis = np.full(time_len, np.nan)
x_body_br_axis = np.full(time_len, np.nan)
z_body_fl_axis = np.full(time_len, np.nan)
z_body_fr_axis = np.full(time_len, np.nan)
z_body_bl_axis = np.full(time_len, np.nan)
z_body_br_axis = np.full(time_len, np.nan)
vx_body_fl_axis = np.full(time_len, np.nan)
vx_body_fr_axis = np.full(time_len, np.nan)
vx_body_bl_axis = np.full(time_len, np.nan)
vx_body_br_axis = np.full(time_len, np.nan)
vz_body_fl_axis = np.full(time_len, np.nan)
vz_body_fr_axis = np.full(time_len, np.nan)
vz_body_bl_axis = np.full(time_len, np.nan)
vz_body_br_axis = np.full(time_len, np.nan)
# mechanical power
mech_pow_hip_fl_axis = np.full(time_len, np.nan)
mech_pow_knee_fl_axis = np.full(time_len, np.nan)
mech_pow_tot_fl_axis = np.full(time_len, np.nan)
mech_pow_hip_fr_axis = np.full(time_len, np.nan)
mech_pow_knee_fr_axis = np.full(time_len, np.nan)
mech_pow_tot_fr_axis = np.full(time_len, np.nan)
mech_pow_hip_bl_axis = np.full(time_len, np.nan)
mech_pow_knee_bl_axis = np.full(time_len, np.nan)
mech_pow_tot_bl_axis = np.full(time_len, np.nan)
mech_pow_hip_br_axis = np.full(time_len, np.nan)
mech_pow_knee_br_axis = np.full(time_len, np.nan)
mech_pow_tot_br_axis = np.full(time_len, np.nan)
# desired swing trajectories
x_sw_d_front_axis = np.full(time_len, np.nan)
x_sw_d_back_axis = np.full(time_len, np.nan)
z_sw_d_front_axis = np.full(time_len, np.nan)
z_sw_d_back_axis = np.full(time_len, np.nan)
vx_sw_d_front_axis = np.full(time_len, np.nan)
vx_sw_d_back_axis = np.full(time_len, np.nan)
vz_sw_d_front_axis = np.full(time_len, np.nan)
vz_sw_d_back_axis = np.full(time_len, np.nan)
# feedback activation
afb_front_axis = np.full(time_len, np.nan)
afb_back_axis = np.full(time_len, np.nan)
# --- setup pybullet environment --- #
pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.setGravity(0, 0, -g)
pb.setPhysicsEngineParameter(fixedTimeStep=t_step, enableFileCaching=0, numSubSteps=1)
pb.resetDebugVisualizerCamera(cameraDistance=0.4, cameraYaw=180, cameraPitch=0, cameraTargetPosition=[0, 0.4, 0.4])
plane_id = pb.loadURDF(fileName='plane.urdf')
# --- video logging setting --- #
if flag_record_video:
    verbose.record_video()
# --- load the robot --- #
robot = solo_class.Solo(plane_id=plane_id,
                        m=m, g=g, lengths=lengths, l_base=l_base,
                        mu_robot=mu_robot, rest_robot=rest_robot,
                        mu_plane=mu_plane, rest_plane=rest_plane,
                        kp_x_init=kp_x_init, kd_x_init=kd_x_init,
                        kp_z_init=kp_z_init, kd_z_init=kd_z_init,
                        kp_th_init=kp_th_init, kd_th_init=kd_th_init,
                        kp_x_st=kp_x_st, kd_x_st=kd_x_st,
                        kp_z_st=kp_z_st, kd_z_st=kd_z_st,
                        kp_th_st=kp_th_st, kd_th_st=kd_th_st,
                        kp_x_sw=kp_x_sw, kd_x_sw=kd_x_sw,
                        kp_z_sw=kp_z_sw, kd_z_sw=kd_z_sw,
                        kp_th_sw=kp_th_sw, kd_th_sw=kd_th_sw,
                        k_st=k_st,
                        start_pos=start_pos, start_orn=start_orn,
                        flag_fix_base=flag_fix_base)
# --- enable torque sensors in actuated joints --- #
get_info.enable_torque_sensors(robot)
# --- change plane and robot dynamics --- #
if flag_change_plane_dynamics == 1:
    control.change_plane_dynamics(robot)
if flag_change_robot_dynamics == 1:
    control.change_robot_dynamics(robot)
# --- print system states --- #
verbose.print_states(robot)
# --- disable default velocity control in all joints --- #
control.disable_default_control(robot)
# --- simulation steps --- #
sm_front = 1               # 1: initialization, 2: stance, 3: swing
sm_back = 1                # 1: initialization, 2: stance, 3: swing
flag_td_front = 0
flag_td_back = 0
i = 0
ii = 0
j = 0
jj = 0
t = 0
t_count = 0
flag_update_t_st_front = 0
flag_update_t_st_back = 0
flag_end_t_st_front = 0
flag_end_t_st_back = 0
t_td_front = 0
t_td_back = 0
while True:
    # --- reset values --- #
    fb_tot_x_fl = 0
    fb_x_pd_fl = 0
    fb_x_p_fl = 0
    fb_x_d_fl = 0
    fb_tot_z_fl = 0
    fb_z_pd_fl = 0
    fb_z_p_fl = 0
    fb_z_d_fl = 0
    fb_th_pd_fl = 0
    fb_th_p_fl = 0
    fb_th_d_fl = 0
    fb_tot_x_fr = 0
    fb_x_pd_fr = 0
    fb_x_p_fr = 0
    fb_x_d_fr = 0
    fb_tot_z_fr = 0
    fb_z_pd_fr = 0
    fb_z_p_fr = 0
    fb_z_d_fr = 0
    fb_th_pd_fr = 0
    fb_th_p_fr = 0
    fb_th_d_fr = 0
    fb_tot_x_bl = 0
    fb_x_pd_bl = 0
    fb_x_p_bl = 0
    fb_x_d_bl = 0
    fb_tot_z_bl = 0
    fb_z_pd_bl = 0
    fb_z_p_bl = 0
    fb_z_d_bl = 0
    fb_th_pd_bl = 0
    fb_th_p_bl = 0
    fb_th_d_bl = 0
    fb_tot_x_br = 0
    fb_x_pd_br = 0
    fb_x_p_br = 0
    fb_x_d_br = 0
    fb_tot_z_br = 0
    fb_z_pd_br = 0
    fb_z_p_br = 0
    fb_z_d_br = 0
    fb_th_pd_br = 0
    fb_th_p_br = 0
    fb_th_d_br = 0
    cnt_x_fl = 0
    cnt_z_fl = 0
    cnt_x_fr = 0
    cnt_z_fr = 0
    cnt_x_bl = 0
    cnt_z_bl = 0
    cnt_x_br = 0
    cnt_z_br = 0
    action_x_fl = 0
    action_z_fl = 0
    action_x_fr = 0
    action_z_fr = 0
    action_x_bl = 0
    action_z_bl = 0
    action_x_br = 0
    action_z_br = 0
    torque_fl_hip = 0
    torque_fl_knee = 0
    torque_fr_hip = 0
    torque_fr_knee = 0
    torque_bl_hip = 0
    torque_bl_knee = 0
    torque_br_hip = 0
    torque_br_knee = 0
    x_sw_d_f = 0
    x_sw_d_b = 0
    z_sw_d_f = 0
    z_sw_d_b = 0
    vx_sw_d_f = 0
    vx_sw_d_b = 0
    vz_sw_d_f = 0
    vz_sw_d_b = 0
    afb_f = 1
    afb_b = 1
    # --- update camera position --- #
    if flag_fix_base == 0 and t >= t_init:
        pb.resetDebugVisualizerCamera(0.8, 145, -5, pb.getBasePositionAndOrientation(robot.robot_id)[0])
    # --- extract coordinates and states --- #
    x_com, y_com, z_com, th_com, vx_com, vz_com, wth_com = get_info.get_com_crds(robot)
    q_front, vq_front = get_info.get_joint_states(robot, 1)
    q_back, vq_back = get_info.get_joint_states(robot, 2)
    x_fl, vx_fl, z_fl, vz_fl, jac_fl, x_fr, vx_fr, z_fr, vz_fr, jac_fr = \
        control.calc_fwd_kin(robot, q_front, vq_front)
    x_bl, vx_bl, z_bl, vz_bl, jac_bl, x_br, vx_br, z_br, vz_br, jac_br = \
        control.calc_fwd_kin(robot, q_back, vq_back)
    x_body_fl, z_body_fl, vx_body_fl, vz_body_fl = \
        get_info.calc_body_crds(robot, 1, x_fl, vx_fl, z_fl, vz_fl, th_com, wth_com)
    x_body_fr, z_body_fr, vx_body_fr, vz_body_fr = \
        get_info.calc_body_crds(robot, 1, x_fr, vx_fr, z_fr, vz_fr, th_com, wth_com)
    x_body_bl, z_body_bl, vx_body_bl, vz_body_bl = \
        get_info.calc_body_crds(robot, 2, x_bl, vx_bl, z_bl, vz_bl, th_com, wth_com)
    x_body_br, z_body_br, vx_body_br, vz_body_br = \
        get_info.calc_body_crds(robot, 2, x_br, vx_br, z_br, vz_br, th_com, wth_com)
    # --- apply initial configuration --- #
    if t <= t_init and sm_front == 1:               # front legs
        fb_x_pd_fl, fb_x_p_fl, fb_x_d_fl, fb_z_pd_fl, fb_z_p_fl, fb_z_d_fl, fb_th_pd_fl, fb_th_p_fl, fb_th_d_fl, \
        fb_x_pd_fr, fb_x_p_fr, fb_x_d_fr, fb_z_pd_fr, fb_z_p_fr, fb_z_d_fr, fb_th_pd_fr, fb_th_p_fr, fb_th_d_fr, \
        fb_tot_x_fl, fb_tot_z_fl, fb_tot_x_fr, fb_tot_z_fr, \
        action_x_fl, action_z_fl, action_x_fr, action_z_fr, \
        torque_fl_hip, torque_fl_knee, torque_fr_hip, torque_fr_knee = \
            control.apply_control(robot, 1, sm_front,
                  x_fl, z_fl, vx_fl, vz_fl, x_fr, z_fr, vx_fr, vz_fr, jac_fl, jac_fr, th_com, wth_com,
                  x_body_fl, x_body_fr, z_body_fl, z_body_fr,
                  x_init_f, vx_init, z_init, vz_init, th_init, wth_init,
                  cnt_x_fl, cnt_x_fr, cnt_z_fl, cnt_z_fr,
                  torque_sat, afb_f, mu, flag_clamp_x_force)
    elif t > t_init and sm_front == 1:              # front legs: switch to stance
        sm_front = 2
    if t <= t_init+t_st and sm_back == 1:           # back legs
        fb_x_pd_bl, fb_x_p_bl, fb_x_d_bl, fb_z_pd_bl, fb_z_p_bl, fb_z_d_bl, fb_th_pd_bl, fb_th_p_bl, fb_th_d_bl, \
        fb_x_pd_br, fb_x_p_br, fb_x_d_br, fb_z_pd_br, fb_z_p_br, fb_z_d_br, fb_th_pd_br, fb_th_p_br, fb_th_d_br, \
        fb_tot_x_bl, fb_tot_z_bl, fb_tot_x_br, fb_tot_z_br, \
        action_x_bl, action_z_bl, action_x_br, action_z_br, \
        torque_bl_hip, torque_bl_knee, torque_br_hip, torque_br_knee = \
            control.apply_control(robot, 2, sm_back,
                                  x_bl, z_bl, vx_bl, vz_bl, x_br, z_br, vx_br, vz_br, jac_bl, jac_br, th_com, wth_com,
                                  x_body_bl, x_body_br, z_body_bl, z_body_br,
                                  x_init_b, vx_init, z_init, vz_init, th_init, wth_init,
                                  cnt_x_bl, cnt_x_br, cnt_z_bl, cnt_z_br,
                                  torque_sat, afb_b, mu, flag_clamp_x_force)
    elif t > t_init+t_st and sm_back == 1:     # back legs: switch to stance
        sm_back = 2
    # --- state machine of front legs --- #
    if sm_front == 2:                           # stance
        cnt_x_fl = cnt_x_front[i]/2
        cnt_x_fr = cnt_x_front[i]/2
        cnt_z_fl = cnt_z_front[i]/2
        cnt_z_fr = cnt_z_front[i]/2
        afb_f = afb_front[i]
        fb_x_pd_fl, fb_x_p_fl, fb_x_d_fl, fb_z_pd_fl, fb_z_p_fl, fb_z_d_fl, fb_th_pd_fl, fb_th_p_fl, fb_th_d_fl, \
        fb_x_pd_fr, fb_x_p_fr, fb_x_d_fr, fb_z_pd_fr, fb_z_p_fr, fb_z_d_fr, fb_th_pd_fr, fb_th_p_fr, fb_th_d_fr, \
        fb_tot_x_fl, fb_tot_z_fl, fb_tot_x_fr, fb_tot_z_fr, \
        action_x_fl, action_z_fl, action_x_fr, action_z_fr, \
        torque_fl_hip, torque_fl_knee, torque_fr_hip, torque_fr_knee = \
            control.apply_control(robot, 1, sm_front,
                                  x_fl, z_fl, vx_fl, vz_fl, x_fr, z_fr, vx_fr, vz_fr, jac_fl, jac_fr, th_com, wth_com,
                                  x_body_fl, x_body_fr, z_body_fl, z_body_fr,
                                  x_st_d, vx_st_d, z_st_d, vz_st_d, th_st_d, wth_st_d,
                                  cnt_x_fl, cnt_x_fr, cnt_z_fl, cnt_z_fr,
                                  torque_sat, afb_f, mu, flag_clamp_x_force)
        i = i + 1
        if i > len(t_st_front_array)-1:         # end of stance: switch to swing
            sm_front = 3
            i = 0
            ii = 0
            flag_end_t_st_front = 1
    elif sm_front == 3:                         # swing
        x_sw_d_f = x_traj_sw_org[ii]
        vx_sw_d_f = vx_traj_sw_org[ii]
        z_sw_d_f = z_traj_sw_org[ii]
        vz_sw_d_f = vz_traj_sw_org[ii]
        th_sw_d = 0
        wth_sw_d = 0
        fb_x_pd_fl, fb_x_p_fl, fb_x_d_fl, fb_z_pd_fl, fb_z_p_fl, fb_z_d_fl, fb_th_pd_fl, fb_th_p_fl, fb_th_d_fl, \
        fb_x_pd_fr, fb_x_p_fr, fb_x_d_fr, fb_z_pd_fr, fb_z_p_fr, fb_z_d_fr, fb_th_pd_fr, fb_th_p_fr, fb_th_d_fr, \
        fb_tot_x_fl, fb_tot_z_fl, fb_tot_x_fr, fb_tot_z_fr, \
        action_x_fl, action_z_fl, action_x_fr, action_z_fr, \
        torque_fl_hip, torque_fl_knee, torque_fr_hip, torque_fr_knee = \
            control.apply_control(robot, 1, sm_front,
                                  x_fl, z_fl, vx_fl, vz_fl, x_fr, z_fr, vx_fr, vz_fr, jac_fl, jac_fr, th_com, wth_com,
                                  x_body_fl, x_body_fr, z_body_fl, z_body_fr,
                                  x_sw_d_f, vx_sw_d_f, z_sw_d_f, vz_sw_d_f, th_sw_d, wth_sw_d,
                                  cnt_x_fl, cnt_x_fr, cnt_z_fl, cnt_z_fr,
                                  torque_sat, afb_f, mu, flag_clamp_x_force)
        ii = ii + 1
        if ii > len(t_sw_array)-1:                                  # end of swing: switch to stance
            sm_front = 2
            i = 0
            ii = 0
        if flag_fix_base == 0 and control.check_td(robot, 1):       # end of swing: switch to stance
            sm_front = 2
            i = 0
            ii = 0
            flag_td_front = 1
            t_td_front = t
    # --- state machine of back legs --- #
    if sm_back == 2:                        # stance
        cnt_x_bl = cnt_x_back[j]/2
        cnt_x_br = cnt_x_back[j]/2
        cnt_z_bl = cnt_z_back[j]/2
        cnt_z_br = cnt_z_back[j]/2
        afb_b = afb_back[j]
        fb_x_pd_bl, fb_x_p_bl, fb_x_d_bl, fb_z_pd_bl, fb_z_p_bl, fb_z_d_bl, fb_th_pd_bl, fb_th_p_bl, fb_th_d_bl, \
        fb_x_pd_br, fb_x_p_br, fb_x_d_br, fb_z_pd_br, fb_z_p_br, fb_z_d_br, fb_th_pd_br, fb_th_p_br, fb_th_d_br, \
        fb_tot_x_bl, fb_tot_z_bl, fb_tot_x_br, fb_tot_z_br, \
        action_x_bl, action_z_bl, action_x_br, action_z_br, \
        torque_bl_hip, torque_bl_knee, torque_br_hip, torque_br_knee = \
            control.apply_control(robot, 2, sm_back,
                                  x_bl, z_bl, vx_bl, vz_bl, x_br, z_br, vx_br, vz_br, jac_bl, jac_br, th_com, wth_com,
                                  x_body_bl, x_body_br, z_body_bl, z_body_br,
                                  x_st_d, vx_st_d, z_st_d, vz_st_d, th_st_d, wth_st_d,
                                  cnt_x_bl, cnt_x_br, cnt_z_bl, cnt_z_br,
                                  torque_sat, afb_b, mu, flag_clamp_x_force)
        j = j + 1
        if j > len(t_st_back_array)-1:       # end of stance: switch to swing
            sm_back = 3
            j = 0
            jj = 0
            flag_end_t_st_back = 1
    elif sm_back == 3:                      # swing
        x_sw_d_b = x_traj_sw_org[jj]
        vx_sw_d_b = vx_traj_sw_org[jj]
        z_sw_d_b = z_traj_sw_org[jj]
        vz_sw_d_b = vz_traj_sw_org[jj]
        th_sw_d = 0
        wth_sw_d = 0
        fb_x_pd_bl, fb_x_p_bl, fb_x_d_bl, fb_z_pd_bl, fb_z_p_bl, fb_z_d_bl, fb_th_pd_bl, fb_th_p_bl, fb_th_d_bl, \
        fb_x_pd_br, fb_x_p_br, fb_x_d_br, fb_z_pd_br, fb_z_p_br, fb_z_d_br, fb_th_pd_br, fb_th_p_br, fb_th_d_br, \
        fb_tot_x_bl, fb_tot_z_bl, fb_tot_x_br, fb_tot_z_br, \
        action_x_bl, action_z_bl, action_x_br, action_z_br, \
        torque_bl_hip, torque_bl_knee, torque_br_hip, torque_br_knee = \
            control.apply_control(robot, 2, sm_back,
                                  x_bl, z_bl, vx_bl, vz_bl, x_br, z_br, vx_br, vz_br, jac_bl, jac_br, th_com, wth_com,
                                  x_body_bl, x_body_br, z_body_bl, z_body_br,
                                  x_sw_d_b, vx_sw_d_b, z_sw_d_b, vz_sw_d_b, th_sw_d, wth_sw_d,
                                  cnt_x_bl, cnt_x_br, cnt_z_bl, cnt_z_br,
                                  torque_sat, afb_b, mu, flag_clamp_x_force)
        jj = jj + 1
        if jj > len(t_sw_array)-1:                                  # end of swing: switch to stance
            sm_back = 2
            j = 0
            jj = 0
        if flag_fix_base == 0 and control.check_td(robot, 2):       # end of swing: switch to stance
            sm_back = 2
            j = 0
            jj = 0
            flag_td_back = 1
            t_td_back = t
    # --- update t_st --- #
    if flag_update_t_st == 1 and flag_td_front == 1 and flag_td_back == 1:
        t_diff = abs(t_td_front-t_td_back)
        t_st = t_st_org - k_st*(t_diff-t_stride/2)
        t_air = (t_sw-t_st)/2
        t_stride = t_st+t_sw
        flag_td_front = 0
        flag_td_back = 0
    if flag_update_t_st == 1 and sm_front == 3:
        t_st_front = t_st
        t_air_front = t_air
        t_stride_front = t_stride
        t_st_front_array = np.arange(0, t_st_front, t_step)
        t_alpha_z_front = t_st_front*ratio_alpha_z
        t_alpha_x_front = t_st_front*0.5
        alpha_cnt_z_front = (m*g*t_stride_front)/(2*0.7*t_st_front)
        alpha_cnt_x_front = -alpha_cnt_z_front*ratio_cnt_xz
        cnt_z_front = gen_curves.gen_cnt(t_st_front, t_alpha_z_front, t_st_front_array,
                                         alpha_cnt_z_front*beta_cnt_z_1, alpha_cnt_z_front*beta_cnt_z_2)
        cnt_x_front = gen_curves.gen_cnt(t_st_front, t_alpha_x_front, t_st_front_array,
                                         alpha_cnt_x_front*beta_cnt_x_1, alpha_cnt_x_front*beta_cnt_x_2)
        if flag_use_afb == 0:
            afb_front = np.ones(len(t_st_front_array))
        elif flag_use_afb == 1:
            t_alpha_1_front = 0.2*t_st_front
            t_alpha_2_front = 0.8*t_st_front
            afb_front = gen_curves.gen_afb(t_st_front, t_alpha_1_front, t_alpha_2_front, t_st_front_array,
                                           1*beta_afb_1, 1*beta_afb_2)
    if flag_update_t_st == 1 and sm_back == 3:
        t_st_back = t_st
        t_air_back = t_air
        t_stride_back = t_stride
        t_st_back_array = np.arange(0, t_st_back, t_step)
        t_alpha_z_back = t_st_back*ratio_alpha_z
        t_alpha_x_back = t_st_back*0.5
        alpha_cnt_z_back = (m*g*t_stride_back)/(2*0.7*t_st_back)
        alpha_cnt_x_back = alpha_cnt_z_back*ratio_cnt_xz
        cnt_z_back = gen_curves.gen_cnt(t_st_back, t_alpha_z_back, t_st_back_array,
                                        alpha_cnt_z_back*beta_cnt_z_1, alpha_cnt_z_back*beta_cnt_z_2)
        cnt_x_back = gen_curves.gen_cnt(t_st_back, t_alpha_x_back, t_st_back_array,
                                        alpha_cnt_x_back*beta_cnt_x_1, alpha_cnt_x_back*beta_cnt_x_2)
        if flag_use_afb == 0:
            afb_back = np.ones(len(t_st_back_array))
        elif flag_use_afb == 1:
            t_alpha_1_back = 0.2*t_st_back
            t_alpha_2_back = 0.8*t_st_back
            afb_back = gen_curves.gen_afb(t_st_back, t_alpha_1_back, t_alpha_2_back, t_st_back_array,
                                          1*beta_afb_1, 1*beta_afb_2)
    # --- read measured foot forces --- #
    tf_x_fl_axis[t_count], tf_z_fl_axis[t_count], tf_x_fr_axis[t_count], tf_z_fr_axis[t_count], \
    tf_x_bl_axis[t_count], tf_z_bl_axis[t_count], tf_x_br_axis[t_count], tf_z_br_axis[t_count] = \
        get_info.get_toe_forces(robot)
    # --- calculate mechanical power --- #
    mech_pow_hip_fl_axis[t_count] = np.abs(torque_fl_hip * vq_front[0])
    mech_pow_knee_fl_axis[t_count] = np.abs(torque_fl_knee * vq_front[1])
    mech_pow_tot_fl_axis[t_count] = mech_pow_hip_fl_axis[t_count] + mech_pow_knee_fl_axis[t_count]
    mech_pow_hip_fr_axis[t_count] = np.abs(torque_fr_hip * vq_front[2])
    mech_pow_knee_fr_axis[t_count] = np.abs(torque_fr_knee * vq_front[3])
    mech_pow_tot_fr_axis[t_count] = mech_pow_hip_fr_axis[t_count] + mech_pow_knee_fr_axis[t_count]
    mech_pow_hip_bl_axis[t_count] = np.abs(torque_bl_hip * vq_back[0])
    mech_pow_knee_bl_axis[t_count] = np.abs(torque_bl_knee * vq_back[1])
    mech_pow_tot_bl_axis[t_count] = mech_pow_hip_bl_axis[t_count] + mech_pow_knee_bl_axis[t_count]
    mech_pow_hip_br_axis[t_count] = np.abs(torque_br_hip * vq_back[2])
    mech_pow_knee_br_axis[t_count] = np.abs(torque_br_knee * vq_back[3])
    mech_pow_tot_br_axis[t_count] = mech_pow_hip_br_axis[t_count] + mech_pow_knee_br_axis[t_count]
    # --- store current data --- #
    t_axis[t_count] = t
    t_st_front_axis[t_count] = t_st_front
    t_st_back_axis[t_count] = t_st_back
    x_com_axis[t_count] = x_com
    y_com_axis[t_count] = y_com
    z_com_axis[t_count] = z_com
    th_com_axis[t_count] = th_com
    vx_com_axis[t_count] = vx_com
    vz_com_axis[t_count] = vz_com
    wth_com_axis[t_count] = wth_com
    fb_tot_x_fl_axis[t_count] = fb_tot_x_fl
    fb_x_pd_fl_axis[t_count] = fb_x_pd_fl
    fb_x_p_fl_axis[t_count] = fb_x_p_fl
    fb_x_d_fl_axis[t_count] = fb_x_d_fl
    fb_tot_z_fl_axis[t_count] = fb_tot_z_fl
    fb_z_pd_fl_axis[t_count] = fb_z_pd_fl
    fb_z_p_fl_axis[t_count] = fb_z_p_fl
    fb_z_d_fl_axis[t_count] = fb_z_d_fl
    fb_th_pd_fl_axis[t_count] = fb_th_pd_fl
    fb_th_p_fl_axis[t_count] = fb_th_p_fl
    fb_th_d_fl_axis[t_count] = fb_th_d_fl
    fb_tot_x_fr_axis[t_count] = fb_tot_x_fr
    fb_x_pd_fr_axis[t_count] = fb_x_pd_fr
    fb_x_p_fr_axis[t_count] = fb_x_p_fr
    fb_x_d_fr_axis[t_count] = fb_x_d_fr
    fb_tot_z_fr_axis[t_count] = fb_tot_z_fr
    fb_z_pd_fr_axis[t_count] = fb_z_pd_fr
    fb_z_p_fr_axis[t_count] = fb_z_p_fr
    fb_z_d_fr_axis[t_count] = fb_z_d_fr
    fb_th_pd_fr_axis[t_count] = fb_th_pd_fr
    fb_th_p_fr_axis[t_count] = fb_th_p_fr
    fb_th_d_fr_axis[t_count] = fb_th_d_fr
    fb_tot_x_bl_axis[t_count] = fb_tot_x_bl
    fb_x_pd_bl_axis[t_count] = fb_x_pd_bl
    fb_x_p_bl_axis[t_count] = fb_x_p_bl
    fb_x_d_bl_axis[t_count] = fb_x_d_bl
    fb_tot_z_bl_axis[t_count] = fb_tot_z_bl
    fb_z_pd_bl_axis[t_count] = fb_z_pd_bl
    fb_z_p_bl_axis[t_count] = fb_z_p_bl
    fb_z_d_bl_axis[t_count] = fb_z_d_bl
    fb_th_pd_bl_axis[t_count] = fb_th_pd_bl
    fb_th_p_bl_axis[t_count] = fb_th_p_bl
    fb_th_d_bl_axis[t_count] = fb_th_d_bl
    fb_tot_x_br_axis[t_count] = fb_tot_x_br
    fb_x_pd_br_axis[t_count] = fb_x_pd_br
    fb_x_p_br_axis[t_count] = fb_x_p_br
    fb_x_d_br_axis[t_count] = fb_x_d_br
    fb_tot_z_br_axis[t_count] = fb_tot_z_br
    fb_z_pd_br_axis[t_count] = fb_z_pd_br
    fb_z_p_br_axis[t_count] = fb_z_p_br
    fb_z_d_br_axis[t_count] = fb_z_d_br
    fb_th_pd_br_axis[t_count] = fb_th_pd_br
    fb_th_p_br_axis[t_count] = fb_th_p_br
    fb_th_d_br_axis[t_count] = fb_th_d_br
    torque_fl_hip_axis[t_count] = torque_fl_hip
    torque_fl_knee_axis[t_count] = torque_fl_knee
    torque_fr_hip_axis[t_count] = torque_fr_hip
    torque_fr_knee_axis[t_count] = torque_fr_knee
    torque_bl_hip_axis[t_count] = torque_bl_hip
    torque_bl_knee_axis[t_count] = torque_bl_knee
    torque_br_hip_axis[t_count] = torque_br_hip
    torque_br_knee_axis[t_count] = torque_br_knee
    cnt_x_fl_axis[t_count] = cnt_x_fl
    cnt_z_fl_axis[t_count] = cnt_z_fl
    cnt_x_fr_axis[t_count] = cnt_x_fr
    cnt_z_fr_axis[t_count] = cnt_z_fr
    cnt_x_bl_axis[t_count] = cnt_x_bl
    cnt_z_bl_axis[t_count] = cnt_z_bl
    cnt_x_br_axis[t_count] = cnt_x_br
    cnt_z_br_axis[t_count] = cnt_z_br
    action_x_fl_axis[t_count] = action_x_fl
    action_z_fl_axis[t_count] = action_z_fl
    action_x_fr_axis[t_count] = action_x_fr
    action_z_fr_axis[t_count] = action_z_fr
    action_x_bl_axis[t_count] = action_x_bl
    action_z_bl_axis[t_count] = action_z_bl
    action_x_br_axis[t_count] = action_x_br
    action_z_br_axis[t_count] = action_z_br
    x_fl_axis[t_count] = x_fl
    x_fr_axis[t_count] = x_fr
    x_bl_axis[t_count] = x_bl
    x_br_axis[t_count] = x_br
    z_fl_axis[t_count] = z_fl
    z_fr_axis[t_count] = z_fr
    z_bl_axis[t_count] = z_bl
    z_br_axis[t_count] = z_br
    vx_fl_axis[t_count] = vx_fl
    vx_fr_axis[t_count] = vx_fr
    vx_bl_axis[t_count] = vx_bl
    vx_br_axis[t_count] = vx_br
    vz_fl_axis[t_count] = vz_fl
    vz_fr_axis[t_count] = vz_fr
    vz_bl_axis[t_count] = vz_bl
    vz_br_axis[t_count] = vz_br
    x_body_fl_axis[t_count] = x_body_fl
    x_body_fr_axis[t_count] = x_body_fr
    x_body_bl_axis[t_count] = x_body_bl
    x_body_br_axis[t_count] = x_body_br
    z_body_fl_axis[t_count] = z_body_fl
    z_body_fr_axis[t_count] = z_body_fr
    z_body_bl_axis[t_count] = z_body_bl
    z_body_br_axis[t_count] = z_body_br
    vx_body_fl_axis[t_count] = vx_body_fl
    vx_body_fr_axis[t_count] = vx_body_fr
    vx_body_bl_axis[t_count] = vx_body_bl
    vx_body_br_axis[t_count] = vx_body_br
    vz_body_fl_axis[t_count] = vz_body_fl
    vz_body_fr_axis[t_count] = vz_body_fr
    vz_body_bl_axis[t_count] = vz_body_bl
    vz_body_br_axis[t_count] = vz_body_br
    x_sw_d_front_axis[t_count] = x_sw_d_f
    x_sw_d_back_axis[t_count] = x_sw_d_b
    z_sw_d_front_axis[t_count] = z_sw_d_f
    z_sw_d_back_axis[t_count] = z_sw_d_b
    vx_sw_d_front_axis[t_count] = vx_sw_d_f
    vx_sw_d_back_axis[t_count] = vx_sw_d_b
    vz_sw_d_front_axis[t_count] = vz_sw_d_f
    vz_sw_d_back_axis[t_count] = vz_sw_d_b
    afb_front_axis[t_count] = afb_f
    afb_back_axis[t_count] = afb_b
    # --- drawing model foot trajectory --- #
    if flag_draw_traj == 1 and t_count > 0 and t_count % draw_step == 0:
        verbose.draw_traj(x_fl_axis[t_count-draw_step], x_fl_axis[t_count],
                          z_fl_axis[t_count-draw_step], z_fl_axis[t_count],
                          x_fr_axis[t_count-draw_step], x_fr_axis[t_count],
                          z_fr_axis[t_count-draw_step], z_fr_axis[t_count],
                          x_bl_axis[t_count-draw_step], x_bl_axis[t_count],
                          z_bl_axis[t_count-draw_step], z_bl_axis[t_count],
                          x_br_axis[t_count-draw_step], x_br_axis[t_count],
                          z_br_axis[t_count-draw_step], z_br_axis[t_count],
                          x_com_axis[t_count-draw_step], x_com_axis[t_count],
                          y_com_axis[t_count-draw_step], y_com_axis[t_count],
                          z_com_axis[t_count-draw_step], z_com_axis[t_count],
                          flag_fix_base)
    # --- next step --- #
    pb.stepSimulation()
    if flag_slow_motion == 1:
        time.sleep(t_slow_motion)
    else:
        time.sleep(t_step)
    t = t + t_step
    t_count = t_count + 1
    if t >= time_array[-1]:
        break
# --- end of simulation --- #
pb.disconnect()
for item in [t_axis, t_st_front_axis, t_st_back_axis,
             x_com_axis, y_com_axis, z_com_axis, th_com_axis, vx_com_axis, vz_com_axis, wth_com_axis,
             fb_tot_x_fl_axis, fb_tot_x_fr_axis, fb_tot_x_bl_axis, fb_tot_x_br_axis,
             fb_tot_z_fl_axis, fb_tot_z_fr_axis, fb_tot_z_bl_axis, fb_tot_z_br_axis,
             fb_x_pd_fl_axis, fb_x_pd_fr_axis, fb_x_pd_bl_axis, fb_x_pd_br_axis,
             fb_z_pd_fl_axis, fb_z_pd_fr_axis, fb_z_pd_bl_axis, fb_z_pd_br_axis,
             fb_th_pd_fl_axis, fb_th_pd_fr_axis, fb_th_pd_bl_axis, fb_th_pd_br_axis,
             fb_x_p_fl_axis, fb_x_p_fr_axis, fb_x_p_bl_axis, fb_x_p_br_axis,
             fb_z_p_fl_axis, fb_z_p_fr_axis, fb_z_p_bl_axis, fb_z_p_br_axis,
             fb_th_p_fl_axis, fb_th_p_fr_axis, fb_th_p_bl_axis, fb_th_p_br_axis,
             fb_x_d_fl_axis, fb_x_d_fr_axis, fb_x_d_bl_axis, fb_x_d_br_axis,
             fb_z_d_fl_axis, fb_z_d_fr_axis, fb_z_d_bl_axis, fb_z_d_br_axis,
             fb_th_d_fl_axis, fb_th_d_fr_axis, fb_th_d_bl_axis, fb_th_d_br_axis,
             torque_fl_hip_axis, torque_fl_knee_axis, torque_fr_hip_axis, torque_fr_knee_axis,
             torque_bl_hip_axis, torque_bl_knee_axis, torque_br_hip_axis, torque_br_knee_axis,
             cnt_x_fl_axis, cnt_z_fl_axis, cnt_x_fr_axis, cnt_z_fr_axis,
             cnt_x_bl_axis, cnt_z_bl_axis, cnt_x_br_axis, cnt_z_br_axis,
             action_x_fl_axis, action_z_fl_axis, action_x_fr_axis, action_z_fr_axis,
             action_x_bl_axis, action_z_bl_axis, action_x_br_axis, action_z_br_axis,
             tf_x_fl_axis, tf_z_fl_axis, tf_x_fr_axis, tf_z_fr_axis,
             tf_z_bl_axis, tf_z_bl_axis, tf_x_br_axis, tf_z_br_axis,
             x_fl_axis, x_fr_axis, x_bl_axis, x_br_axis,
             z_fl_axis, z_fr_axis, z_bl_axis, z_br_axis,
             vx_fl_axis, vx_fr_axis, vx_bl_axis, vx_br_axis,
             vz_fl_axis, vz_fr_axis, vz_bl_axis, vz_br_axis,
             x_body_fl_axis, x_body_fr_axis, x_body_bl_axis, x_body_br_axis,
             z_body_fl_axis, z_body_fr_axis, z_body_bl_axis, z_body_br_axis,
             vx_body_fl_axis, vx_body_fr_axis, vx_body_bl_axis, vx_body_br_axis,
             vz_body_fl_axis, vz_body_fr_axis, vz_body_bl_axis, vz_body_br_axis,
             mech_pow_hip_fl_axis, mech_pow_knee_fl_axis, mech_pow_tot_fl_axis,
             mech_pow_hip_fr_axis, mech_pow_knee_fr_axis, mech_pow_tot_fr_axis,
             mech_pow_hip_bl_axis, mech_pow_knee_bl_axis, mech_pow_tot_bl_axis,
             mech_pow_hip_br_axis, mech_pow_knee_br_axis, mech_pow_tot_br_axis,
             x_sw_d_front_axis, x_sw_d_back_axis, z_sw_d_front_axis, z_sw_d_back_axis,
             vx_sw_d_front_axis, vx_sw_d_back_axis, vz_sw_d_front_axis, vz_sw_d_back_axis,
             afb_front_axis, afb_back_axis]:
    item = item[~np.isnan(item)]
# --- generate final plots --- #
plot.plot_full(m, g, v_d, torque_sat, t_axis,
              x_com_axis, y_com_axis, z_com_axis, th_com_axis, vx_com_axis, vz_com_axis, wth_com_axis,
              fb_tot_x_fl_axis, fb_tot_z_fl_axis, fb_x_pd_fl_axis, fb_z_pd_fl_axis, fb_th_pd_fl_axis,
              fb_x_p_fl_axis, fb_x_d_fl_axis, fb_z_p_fl_axis, fb_z_d_fl_axis, fb_th_p_fl_axis, fb_th_d_fl_axis,
              fb_tot_x_fr_axis, fb_tot_z_fr_axis, fb_x_pd_fr_axis, fb_z_pd_fr_axis, fb_th_pd_fr_axis,
              fb_x_p_fr_axis, fb_x_d_fr_axis, fb_z_p_fr_axis, fb_z_d_fr_axis, fb_th_p_fr_axis, fb_th_d_fr_axis,
              fb_tot_x_bl_axis, fb_tot_z_bl_axis, fb_x_pd_bl_axis, fb_z_pd_bl_axis, fb_th_pd_bl_axis,
              fb_x_p_bl_axis, fb_x_d_bl_axis, fb_z_p_bl_axis, fb_z_d_bl_axis, fb_th_p_bl_axis, fb_th_d_bl_axis,
              fb_tot_x_br_axis, fb_tot_z_br_axis, fb_x_pd_br_axis, fb_z_pd_br_axis, fb_th_pd_br_axis,
              fb_x_p_br_axis, fb_x_d_br_axis, fb_z_p_br_axis, fb_z_d_br_axis, fb_th_p_br_axis, fb_th_d_br_axis,
              torque_fl_hip_axis, torque_fl_knee_axis, torque_fr_hip_axis, torque_fr_knee_axis,
              torque_bl_hip_axis, torque_bl_knee_axis, torque_br_hip_axis, torque_br_knee_axis,
              cnt_x_fl_axis, cnt_z_fl_axis, cnt_x_fr_axis, cnt_z_fr_axis,
              cnt_x_bl_axis, cnt_z_bl_axis, cnt_x_br_axis, cnt_z_br_axis,
              action_x_fl_axis, action_z_fl_axis, action_x_fr_axis, action_z_fr_axis,
              action_x_bl_axis, action_z_bl_axis, action_x_br_axis, action_z_br_axis,
              tf_x_fl_axis, tf_z_fl_axis, tf_x_fr_axis, tf_z_fr_axis,
              tf_x_bl_axis, tf_z_bl_axis, tf_x_br_axis, tf_z_br_axis,
              x_fl_axis, x_fr_axis, x_bl_axis, x_br_axis,
              z_fl_axis, z_fr_axis, z_bl_axis, z_br_axis,
              vx_fl_axis, vx_fr_axis, vx_bl_axis, vx_br_axis,
              vz_fl_axis, vz_fr_axis, vz_bl_axis, vz_br_axis,
              mech_pow_hip_fl_axis, mech_pow_knee_fl_axis, mech_pow_tot_fl_axis,
              mech_pow_hip_fr_axis, mech_pow_knee_fr_axis, mech_pow_tot_fr_axis,
              mech_pow_hip_bl_axis, mech_pow_knee_bl_axis, mech_pow_tot_bl_axis,
              mech_pow_hip_br_axis, mech_pow_knee_br_axis, mech_pow_tot_br_axis,
              x_traj_sw_org, z_traj_sw_org,
              t_st_front_axis, t_st_back_axis)
