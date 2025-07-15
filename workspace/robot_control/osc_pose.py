import numpy as np
import time
from receive_hand_positions import hand_position_stream
import robosuite as suite
import sys
sys.path.append(r"C:\Users\Lukas\deoxys_control\deoxys")
#from deoxys.franka_interface import FrankaInterface
from deoxys.utils import transform_utils
#from deoxys.utils.config_utils import get_default_controller_config

# Constants
# Scales for the robot's end-effector
X_POS_SCALE = 0.02  # Scale for robots x position
Y_POS_SCALE = 0.02  # Scale for robots y position
Z_POS_SCALE = 0.006  # Scale for robots z position
X_ROT_SCALE = -1  # Scale for robots x rotation
Y_ROT_SCALE = 1  # Scale for robots y rotation   
Z_ROT_SCALE = -1  # Scale for robots z rotation

# Simulation parameters
MAX_FR = 60 # Maximum frame rate
CONTROL_FREQ = 240  # Control frequency in Hz

# Simulation or real robot mode
simulation = True  # Set to False for real robot mode

def get_target_pose(hand_data):
    """Convert hand tracking data to target pose for the robot."""
    target_pos = np.array([
        hand_data['z'] * X_POS_SCALE,
        hand_data['x'] * Y_POS_SCALE,
        hand_data['y'] * Z_POS_SCALE,
    ])
    target_quat = np.array([
        hand_data['orientation']['w'],
        hand_data['orientation']['y'] * X_ROT_SCALE,
        hand_data['orientation']['x'] * Y_ROT_SCALE,
        hand_data['orientation']['z'] * Z_ROT_SCALE
    ])
    grasp = np.array([hand_data['pinch_strength']])
    return target_pos, target_quat, grasp

def osc_move(current_pose, target_pose):
    target_pos, target_quat, grasp = target_pose
    current_pos, current_quat = current_pose
    if np.dot(target_quat, current_quat) < 0.0:
        current_quat = -current_quat
    quat_diff = transform_utils.quat_distance(target_quat, current_quat)
    axis_angle_diff = transform_utils.quat2axisangle(quat_diff)

    action_pos = (target_pos - current_pos).flatten()
    action_axis_angle = axis_angle_diff.flatten()
    # action_pos = np.clip(action_pos, -1.0, 1.0)
    # action_axis_angle = np.clip(action_axis_angle, -0.2, 0.2)

    # gripper
    if grasp == 0:
        grasp = np.array([-1.0])

    #action_axis_angle.tolist()
    #np.array([0.0, 0, 0]).tolist()
    action = action_pos.tolist() + action_axis_angle.tolist() + grasp.tolist()
    return action

def main():
    if simulation:
        hand_stream = hand_position_stream()
        env = suite.make(
            env_name="Lift",
            robots="Panda",
            has_renderer=True,
            ignore_done=True,
            control_freq=CONTROL_FREQ,
            has_offscreen_renderer=False,
            use_camera_obs=False,
        )
        obs = env.reset()
        while True:
            start = time.time()

            # Hand tracking data (immer das neueste Paket)
            hand_data = next(hand_stream)
            target_pose = get_target_pose(hand_data)

            # current pose
            current_pos = obs["robot0_eef_pos"]
            current_quat = obs["robot0_eef_quat"]

            action = osc_move((current_pos, current_quat), target_pose)

            obs, reward, done, info = env.step(action)
            env.render()

            # elapsed = time.time() - start
            # diff = 1 / MAX_FR - elapsed
            # if diff > 0:
            #     time.sleep(diff)

    # Run Program on Real Robot
    elif (not simulation):
        robot_interface = FrankaInterface("/deoxys/config/charmander.yml"
                                          , use_visualizer=False)
        controller_type = "OSC_POSE"
        controller_cfg = get_default_controller_config(controller_type)

        robot_interface = FrankaInterface("/deoxys/config/charmander.yml"
                                          , use_visualizer=False)
        
        while True:
            # Hand tracking data
            hand_data = receive_hand_positions()
            target_pose = get_target_pose(hand_data)

            # current pose
            current_quat, current_pos = robot_interface.last_eef_quat_and_pos

            robot_interface.control(controller_type=controller_type,
                                    action=osc_move((current_pos, current_quat), target_pose),
                                    controller_cfg=controller_cfg,
                                    )

if __name__ == "__main__":
    main()