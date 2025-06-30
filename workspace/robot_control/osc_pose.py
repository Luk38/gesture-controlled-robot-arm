import numpy as np
import time
from receive_hand_positions import receive_hand_positions
import robosuite as suite
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import transform_utils
from deoxys.utils.config_utils import get_default_controller_config

# Constants
# Scales for the robot's end-effector
X_POS_SCALE = 0.01  # Scale for robots x position
Y_POS_SCALE = 0.01  # Scale for robots y position
Z_POS_SCALE = 0.015  # Scale for robots z position
X_ROT_SCALE = 10.0  # Scale for robots x rotation
Y_ROT_SCALE = 10.0  # Scale for robots y rotation   
Z_ROT_SCALE = 10.0  # Scale for robots z rotation

# Simulation parameters
MAX_FR = 60 # Maximum frame rate
CONTROL_FREQ = 60  # Control frequency in Hz

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
        hand_data['orientation']['z'] * X_ROT_SCALE,
        hand_data['orientation']['x'] * Y_ROT_SCALE,
        hand_data['orientation']['y'] * Z_ROT_SCALE,
        hand_data['orientation']['w']
    ])
    return target_pos, target_quat

def osc_move(robot_interface, target_pose):
    target_pos, target_quat = target_pose
    current_rot, current_pos = robot_interface.last_eef_rot_and_pos
    # Current Pose abfrage hier für  Simulation oder realen Roboter extra machen
    # Für Simulation: current_pos = obs["robot0_eef_pos"] usw.
    current_pose = robot_interface.last_eef_pose
    current_pos = current_pose[:3, 3:]
    current_rot = current_pose[:3, :3]
    current_quat = transform_utils.mat2quat(current_rot)
    if np.dot(target_quat, current_quat) < 0.0:
        current_quat = -current_quat
    quat_diff = transform_utils.quat_distance(target_quat, current_quat)
    axis_angle_diff = transform_utils.quat2axisangle(quat_diff)

    action_pos = (target_pos - current_pos).flatten()
    action_axis_angle = axis_angle_diff.flatten()
    # action_pos = np.clip(action_pos, -1.0, 1.0)
    # action_axis_angle = np.clip(action_axis_angle, -0.5, 0.5)

    action = action_pos.tolist() + action_axis_angle.tolist() + [-1.0]
    return action

def main():
    controller_type = "OSC_POSE"
    controller_cfg = get_default_controller_config(controller_type)
    robot_interface = FrankaInterface("config/charmander.yml"
                                      , use_visualizer=False)
    
    # Run Program in Simulation
    if(simulation):

        # create environment instance
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

            # Hand tracking data
            hand_data = receive_hand_positions()
            target_pose = get_target_pose(hand_data)

            # calculate delta position
            # current_pos = obs["robot0_eef_pos"]
            # target_pos = np.array([hand_data['z']*X_POS_SCALE,
            #                     hand_data['x']*Y_POS_SCALE,
            #                     hand_data['y']*Z_POS_SCALE])
            # delta_pos = target_pos - current_pos

            # # calculate delta rotation
            # hand_ori = hand_data['orientation']
            # current_rot = obs["robot0_eef_quat"]
            # target_rot = np.array([hand_ori['z']*X_ROT_SCALE,
            #                     hand_ori['x']*Y_ROT_SCALE,
            #                     hand_ori['y']*Z_ROT_SCALE,])
            # delta_rot = target_rot - current_rot[:3]

            # # gripper
            # gripper = np.array([hand_data['pinch_strength']])
            # if gripper == 0:
            #     gripper = np.array([-1.0])

            # Action: [dx, dy, dz, droll, dpitch, dyaw, gripper]
            # action = np.concatenate([delta_pos, np.zeros(3), gripper])

            action  = osc_move(robot_interface, target_pose)

            obs, reward, done, info = env.step(action)
            env.render()

            elapsed = time.time() - start
            diff = 1 / MAX_FR - elapsed
            if diff > 0:
                time.sleep(diff)

    # Run Program on Real Robot
    elif (not simulation):

        controller_type = "OSC_POSE"
        controller_cfg = get_default_controller_config(controller_type)

        robot_interface = FrankaInterface("config/charmander.yml"
                                          , use_visualizer=False)
        
        while True:
            # Hand tracking data
            hand_data = receive_hand_positions()
            target_pose = get_target_pose(hand_data)

            robot_interface.control(controller_type=controller_type,
                                    action=osc_move(robot_interface, target_pose),
                                    controller_cfg=controller_cfg,
                                    )
                                    
        robot_interface.close()



if __name__ == "__main__":
    main()