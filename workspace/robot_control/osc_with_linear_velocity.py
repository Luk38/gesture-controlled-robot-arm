import numpy as np
import time
from receive_hand_positions import receive_hand_positions
#import robosuite as suite
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import transform_utils
from deoxys.utils.config_utils import get_default_controller_config
from deoxys.experimental.motion_utils import reset_joints_to


# Simulation or real robot mode
simulation = False # Set to False for real robot mode

# Real robot
if not simulation:
    # Scales for the robot's end-effector
    X_POS_SCALE = 0.009  
    Y_POS_SCALE = 0.004  
    Z_POS_SCALE = 0.004  
    X_ROT_SCALE = 1  
    Y_ROT_SCALE = -1   
    Z_ROT_SCALE = -1  
    X_OFFSET = 0.6  
    Y_OFFSET = -0.02  
    Z_OFFSET = -0.4  
    X_OFFSET = 30
    Y_OFFSET = 0
    Z_OFFSET = -220 

# Simulation
elif simulation:
    X_POS_SCALE = 0.02  
    Y_POS_SCALE = 0.03  
    Z_POS_SCALE = 0.006 
    X_ROT_SCALE = -1  
    Y_ROT_SCALE = 1   
    Z_ROT_SCALE = -1 
    X_OFFSET = 0 
    Y_OFFSET = 0 
    Z_OFFSET = 0 

    # Simulation parameters
    MAX_FR = 60 # Maximum frame rate
    CONTROL_FREQ = 60 # Control frequency in Hz


def get_target_pose(hand_data):
    """Convert hand tracking data to target pose for the robot."""
    target_pos = np.array([
        (hand_data['z'] * X_POS_SCALE) + X_OFFSET,
        (hand_data['x'] * Y_POS_SCALE) + Y_OFFSET,
        (hand_data['y'] * Z_POS_SCALE) + Z_OFFSET,
    ])
    target_quat = np.array([
        hand_data['orientation']['w'],
        hand_data['orientation']['y'] * X_ROT_SCALE,
        hand_data['orientation']['x'] * Y_ROT_SCALE,
        hand_data['orientation']['z'] * Z_ROT_SCALE
    ])
    grasp = np.array([hand_data['pinch_strength']])
    return target_pos, target_quat, grasp

def osc_move(current_pose, target_pose, raw_lin):
    """Compute the action for the robot using Operational Space Control (OSC) with linear velocity."""
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
    action_axis_angle = axis_angle_diff.flatten()
    # action_pos = np.clip(action_pos, -1, 1)
    action_axis_angle = np.clip(action_axis_angle, -0.3, 0.3)
    # gripper
    if grasp <= 0.4:
        grasp = np.array([-1.0])

    action_pos = raw_lin.flatten()
    action_pos = np.where(np.abs(action_pos) < 0.01, 0, action_pos)
    action_axis_angle = np.where(np.abs(action_axis_angle) < 0.01, 0, action_axis_angle)

    #action_pos.tolist()
    #action_axis_angle.tolist()
    #np.array([0.0, 0, 0]).tolist()
    action = action_pos.tolist() + action_axis_angle.tolist() + grasp.tolist()
    return action

def main():

    robot_interface = FrankaInterface("config/charmander.yml"
                                        , use_visualizer=False)
    controller_type = "OSC_POSE"
    controller_cfg = get_default_controller_config(controller_type)

    reset_joint_positions = [
        0.09162008114028396,
        -0.19826458111314524,
        -0.01990020486871322,
        -2.4732269941140346,
        -0.01307073642274261,
        2.30396583422025,
        0.8480939705504309,
        ]

    reset_joints_to(robot_interface, reset_joint_positions)
    try:
        while True:
            # current pose
            current_pose = robot_interface.last_eef_pose
            # Hand tracking data              
            hand_data = receive_hand_positions()
            scale_lin = 0.005
            raw_lin = np.array([
                scale_lin * (hand_data['z'] + X_OFFSET),
                scale_lin * (hand_data['x'] + Y_OFFSET),
                scale_lin * (hand_data['y'] + Z_OFFSET),
            ])
            if hand_data['grab_strength'] > 0.8:
                action = [0.0, 0, 0, 0, 0, 0] + [-1]
            else:
                target_pose = get_target_pose(hand_data)
                action = osc_move(current_pose, target_pose, raw_lin)

            robot_interface.control(controller_type=controller_type,
                                    action=action,
                                    controller_cfg=controller_cfg,
                                    )
    except KeyboardInterrupt:
        print("Program stopped by user.")
    finally:
        # Send termination command
        robot_interface.control(
            controller_type=controller_type,
            action=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0] + [1.0],
            controller_cfg=controller_cfg,
            termination=True,
        )
        print("Closing robot interface.")
        robot_interface.close()



if __name__ == "__main__":
    main()