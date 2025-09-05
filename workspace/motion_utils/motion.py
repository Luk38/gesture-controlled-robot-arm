import numpy as np
import time
from deoxys.utils import transform_utils

def osc_move(current_pose, target_pose, simulation):
    if simulation:
        target_pos, target_quat, grasp = target_pose
        current_pos, current_quat = current_pose
    elif not simulation:
        target_pos, target_quat, grasp = target_pose
        target_pos = target_pos.reshape((3, 1))
        current_pos = current_pose[:3, 3:]
        current_rot = current_pose[:3, :3]
        current_quat = transform_utils.mat2quat(current_rot)
    if np.dot(target_quat, current_quat) < 0.0:
        current_quat = -current_quat
    quat_diff = transform_utils.quat_distance(target_quat, current_quat)
    axis_angle_diff = transform_utils.quat2axisangle(quat_diff)

    action_pos = (target_pos - current_pos).flatten() 
    print("target_pos:", target_pos)
    print("current_pos:", current_pos)
    print("action_pos:", action_pos)
    action_axis_angle = axis_angle_diff.flatten()
    # print("action_axis_angle:", action_axis_angle)
    # action_pos = np.clip(action_pos, -1, 1)
    action_axis_angle = np.clip(action_axis_angle, -0.3, 0.3)

    # gripper
    if grasp == 0:
        grasp = np.array([-1.0])

    #action_pos.tolist()
    #action_axis_angle.tolist()
    #np.array([0.0, 0, 0]).tolist()
    action = action_pos.tolist() + action_axis_angle.tolist() + grasp.tolist()
    #print("action:", action)
    return action