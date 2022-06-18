import pybullet as pb


def record_video():
    pb.startStateLogging(pb.STATE_LOGGING_VIDEO_MP4,
                         fileName='./videos/biorob_bounding_fwd.mp4')
    return


def print_states(robot):
    print()
    print("******* Plane State *******")
    temp = pb.getDynamicsInfo(bodyUniqueId=robot.plane_id, linkIndex=-1)
    print(f'{f"Link Index: {-1}":<14}, Link Mass: {temp[0]:.6f}, Link Lateral Friction: {temp[1]}, '
          f'Link Restitution: {temp[5]}, Link Rolling Friction: {temp[6]}, Link Spinning Friction: {temp[7]}, '
          f'Contact Damping: {temp[8]}, Contact Stiffness: {temp[9]}')
    print("******* Joint States *******")
    for j in robot.joints:
        temp = pb.getJointInfo(bodyUniqueId=robot.robot_id, jointIndex=j)
        print(f'{f"Joint Name: {temp[1]}":<28}, {f"Joint Index: {temp[0]}":<15}, Joint Damping: {temp[6]}, '
              f'Joint Friction: {temp[7]}, Joint Lower Limit: {temp[8]:.6f}, joint Upper Limit: {temp[9]:.6f}, '
              f'Joint Max Force: {temp[10]}, Joint Max Velocity: {temp[11]}', temp[13])
    print("******* Link States *******")
    for j in robot.links:
        temp = pb.getDynamicsInfo(bodyUniqueId=robot.robot_id, linkIndex=j)
        print(f'{f"Link Index: {j}":<14}, Link Mass: {temp[0]:.6f}, Link Lateral Friction: {temp[1]}, '
              f'Link Restitution: {temp[5]}, Link Rolling Friction: {temp[6]}, Link Spinning Friction: {temp[7]}, '
              f'Contact Damping: {temp[8]}, Contact Stiffness: {temp[9]}')
    print()
    return


def draw_traj(x_fl_1, x_fl_2, z_fl_1, z_fl_2, x_fr_1, x_fr_2, z_fr_1, z_fr_2,
              x_bl_1, x_bl_2, z_bl_1, z_bl_2, x_br_1, x_br_2, z_br_1, z_br_2,
              x_com_1, x_com_2, y_com_1, y_com_2, z_com_1, z_com_2, flag_fix_base):
    if flag_fix_base == 1:
        # relative coordinates of the hip joint w.r.t. the center of the base
        y = (87.5 + 14 + 37.45) / 1000
        x = (214 - 19.5) / 1000
        # front left foot
        pb.addUserDebugLine([x_fl_1+x_com_2+x, y_com_2+y, z_com_2-z_fl_1],
                            [x_fl_2+x_com_2+x, y_com_2+y, z_com_2-z_fl_2],
                            [0, 0, 0])  # black
        # front right foot
        pb.addUserDebugLine([x_fr_1+x_com_2+x, y_com_2-y, z_com_2-z_fr_1],
                            [x_fr_2+x_com_2+x, y_com_2-y, z_com_2-z_fr_2],
                            [1, 0, 0])  # red
        # back left foot
        pb.addUserDebugLine([x_bl_1+x_com_2-x, y_com_2+y, z_com_2-z_bl_1],
                            [x_bl_2+x_com_2-x, y_com_2+y, z_com_2-z_bl_2],
                            [0, 1, 0])  # green
        # back right foot
        pb.addUserDebugLine([x_br_1+x_com_2-x, y_com_2-y, z_com_2-z_br_1],
                            [x_br_2+x_com_2-x, y_com_2-y, z_com_2-z_br_2],
                            [0, 0, 1])  # blue
    # CoM
    pb.addUserDebugLine([x_com_1, y_com_1, z_com_1],
                        [x_com_2, y_com_2, z_com_2],
                        [1, 1, 0])      # yellow
    return


def check_flags(flags):
    for item in flags:
        if item not in [0, 1]:
            raise Exception("Please check flag inputs based on the comments given accordingly!")
    return
