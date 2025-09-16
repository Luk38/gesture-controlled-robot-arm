import numpy as np
import time
from receive_hand_positions import receive_hand_positions
import robosuite as suite
#import sys
#sys.path.append(r"C:\Users\Lukas\deoxys_control\deoxys")
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import transform_utils
from deoxys.utils.config_utils import get_default_controller_config, verify_controller_config
from deoxys.experimental.motion_utils import reset_joints_to


# Simulation or real robot mode
simulation = False # Set to False for real robot mode

# Limits for hand tracker
INNER_X_LIMIT = 50.0
INNER_Z_MIN = -100.0
INNER_Z_MAX = 45.0

# Real robot
if not simulation:
    # Scales for the robot's end-effector
    X_POS_SCALE = -0.01  # Scale for robots x position
    Y_POS_SCALE = 0.004  # Scale for robots y position
    Z_POS_SCALE = 0.004  # Scale for robots z position
    X_ROT_SCALE = -1  # Scale for robots x rotation
    Y_ROT_SCALE = 1  # Scale for robots y rotation   
    Z_ROT_SCALE = -1  # Scale for robots z rotation
    Z_OFFSET = -0.4  #z-axis offset for the robot

    CARTESIAN_VELOCITY = "CARTESIAN_VELOCITY"
    CARTESIAN_VELOCITY_CFG = get_default_controller_config(CARTESIAN_VELOCITY)
    OSC_POSE = "OSC_POSE"
    OSC_POSE_CFG = get_default_controller_config(OSC_POSE)

# Simulation
elif simulation:
    X_POS_SCALE = -0.02  # Scale for robots x position
    Y_POS_SCALE = 0.03  # Scale for robots y position
    Z_POS_SCALE = 0.006  # Scale for robots z position
    X_ROT_SCALE = -1  # Scale for robots x rotation
    Y_ROT_SCALE = 1  # Scale for robots y rotation   
    Z_ROT_SCALE = -1  # Scale for robots z rotation

    # Simulation parameters
    MAX_FR = 60 # Maximum frame rate
    CONTROL_FREQ = 60 # Control frequency in Hz

def check_hand_data(hand_data):
    """
    Check if the hand is in the inner or outer area.
    Args:
        hand_data (dict): Hand tracking data.
    Returns:
        str: "inner" if the hand is in the inner area, "outer" otherwise.
    """
    if -INNER_X_LIMIT < hand_data['x'] < INNER_X_LIMIT and INNER_Z_MIN < hand_data['z'] < INNER_Z_MAX:
        return "inner"
    else:
        return "outer"

def get_target_pose(hand_data):
    """Convert hand tracking data to target pose for the robot."""
    target_pos = np.array([
        hand_data['z'] * X_POS_SCALE,
        hand_data['x'] * Y_POS_SCALE,
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

def osc_move(current_pose, target_pose):
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
    action_axis_angle = axis_angle_diff.flatten()
    # action_pos = np.clip(action_pos, -1, 1)
    action_axis_angle = np.clip(action_axis_angle, -0.25, 0.25)

    # gripper
    if grasp == 0:
        grasp = np.array([-1.0])

    #action_pos.tolist()
    #action_axis_angle.tolist()
    #np.array([0.0, 0, 0]).tolist()
    action = action_pos.tolist() + action_axis_angle.tolist() + grasp.tolist()
    #print("action:", action)
    return action

def velocity_move(hand_data):
    v_max = 0.045
    scale = 0.001
    if hand_data['x'] < -INNER_X_LIMIT:
        vx = (hand_data['x'] + INNER_X_LIMIT) * scale
    elif hand_data['x'] > INNER_X_LIMIT:
        vx = (hand_data['x'] - INNER_X_LIMIT) * scale
    else:
        vx = 0

    if hand_data['z'] < INNER_Z_MIN:
        vz = (hand_data['z'] - INNER_Z_MIN) * scale
    elif hand_data['z'] > INNER_Z_MAX:
        vz = (hand_data['z'] - INNER_Z_MAX) * scale
    else:
        vz = 0

    vx = np.clip(vx, -v_max, v_max)
    vz = np.clip(vz, -v_max, v_max)

    action = [-vz, -vx, 0.0, 0.0, 0.0, 0.0] + [-1]
    return action

def inner_area(robot_interface, target_pose):
    # current pose
    current_pose = robot_interface.last_eef_pose

    robot_interface.control(controller_type=OSC_POSE,
                            action=osc_move(current_pose, target_pose),
                            controller_cfg=OSC_POSE_CFG,
                            )
    
def outer_area(robot_interface, hand_data):
    robot_interface.control(controller_type=CARTESIAN_VELOCITY,
                            action=velocity_move(hand_data),
                            controller_cfg=CARTESIAN_VELOCITY_CFG,
                            )


def main():
    if simulation:
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
        try:
            while True:
                start = time.time()

                # Hand tracking data
                hand_data = receive_hand_positions()
                target_pose = get_target_pose(hand_data)

                # current pose
                current_pos = obs["robot0_eef_pos"]
                current_quat = obs["robot0_eef_quat"]

                action = osc_move((current_pos, current_quat), target_pose)

                obs, reward, done, info = env.step(action)
                env.render()

                elapsed = time.time() - start
                diff = 1 / MAX_FR - elapsed
                if diff > 0:
                    time.sleep(diff)
        except KeyboardInterrupt:
            print("Simulation stopped by user.")
        finally:
            print("Closing simulation environment.")
            env.close()

    # Run Program on Real Robot
    elif (not simulation):
        robot_interface = FrankaInterface("config/charmander.yml"
                                          , use_visualizer=False)

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
                hand_data = receive_hand_positions()
                target_pose = get_target_pose(hand_data)
                area = check_hand_data(hand_data)
                if area == "inner":
                    inner_area(robot_interface, target_pose)
                if area == "outer":
                    outer_area(robot_interface, hand_data)
        except KeyboardInterrupt:
            print("Program stopped by user.")
        finally:
            print("Closing robot interface.")
            robot_interface.close()



if __name__ == "__main__":
    main()