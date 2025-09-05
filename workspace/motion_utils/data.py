import numpy as np

# Simulation or real robot mode
simulation = False # Set to False for real robot mode

# Real robot
if not simulation:
    # Scales for the robot's end-effector
    X_POS_SCALE = 0.009  # Scale for robots x position
    Y_POS_SCALE = 0.004  # Scale for robots y position
    Z_POS_SCALE = 0.004  # Scale for robots z position
    X_ROT_SCALE = -1  # Scale for robots x rotation
    Y_ROT_SCALE = -1  # Scale for robots y rotation   
    Z_ROT_SCALE = -1  # Scale for robots z rotation
    X_OFFSET = 0.6  # x-axis offset for the robot
    Y_OFFSET = -0.02  # y-axis offset for the robot
    Z_OFFSET = -0.4  # z-axis offset for the robot

# Simulation
elif simulation:
    X_POS_SCALE = 0.02  # Scale for robots x position
    Y_POS_SCALE = 0.03  # Scale for robots y position
    Z_POS_SCALE = 0.006  # Scale for robots z position
    X_ROT_SCALE = -1  # Scale for robots x rotation
    Y_ROT_SCALE = 1  # Scale for robots y rotation   
    Z_ROT_SCALE = -1  # Scale for robots z rotation
    X_OFFSET = 0  # x-axis offset for the robot
    Y_OFFSET = 0  # y-axis offset for the robot
    Z_OFFSET = 0  # z-axis offset for the robot

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
    #print("target_pos_z:", target_pos[2])
    return target_pos, target_quat, grasp
