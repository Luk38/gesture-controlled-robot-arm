import numpy as np
import time
from receive_hand_positions import receive_hand_positions
import robosuite as suite
#import sys
#sys.path.append(r"C:\Users\Lukas\deoxys_control\deoxys")
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import transform_utils
from deoxys.utils.config_utils import get_default_controller_config
from deoxys.experimental.motion_utils import reset_joints_to

# Real robot Constants
X_OFFSET = 50
Y_OFFSET = 0
Z_OFFSET = -200 

X_ROT_SCALE = 0.1
Y_ROT_SCALE = 0.02
Z_ROT_SCALE = 0.02

global cx 
global cy
global cz

def smooth_velocity(current_velocity, next_velocity, alpha=0.1):
    v_smoothed = alpha * next_velocity + (1 - alpha) * current_velocity # exponential smoothing
    
    return v_smoothed

def acceleration_limiter(current_velocity, next_velocity, max_acceleration=0.01):
    delta_v = next_velocity - current_velocity
    delta_v = np.clip(delta_v, -max_acceleration, max_acceleration)
    
    return current_velocity + delta_v

# jerk limiter hier noch einbauen

def velocity_move(hand_data):
    global cx, cy, cz
    v_max = 0.09
    scale = 0.0004 * 2

    vx = scale * (hand_data['z'] + X_OFFSET)
    vy = scale * (hand_data['x'] + Y_OFFSET)
    vz = scale * (hand_data['y'] + Z_OFFSET)

    vx = smooth_velocity(cx, vx)
    vy = smooth_velocity(cy, vy)
    vz = smooth_velocity(cz, vz)

    vx = acceleration_limiter(cx, vx)
    vy = acceleration_limiter(cy, vy)
    vz = acceleration_limiter(cz, vz)

    vx = np.clip(vx, -v_max, v_max)
    vy = np.clip(vy, -v_max, v_max)
    vz = np.clip(vz, -v_max, v_max)

    cx = vx
    cy = vy
    cz = vz

    # hand_quat = np.array([
    #     hand_data['orientation']['w'],
    #     hand_data['orientation']['y'],
    #     hand_data['orientation']['x'],
    #     hand_data['orientation']['z']
    # ])
    # axis_angle = transform_utils.quat2axisangle(hand_quat)
    # print(axis_angle)
    # rx = (axis_angle[0] - 3) * X_ROT_SCALE
    # ry = (axis_angle[2]) * Y_ROT_SCALE
    # rz = (axis_angle[1]) * Z_ROT_SCALE

    # rx = np.clip(rx, -v_max, v_max)
    # ry = np.clip(ry, -v_max, v_max)
    # rz = np.clip(rz, -v_max, v_max)

    gripper = hand_data['pinch_strength']
    if gripper == 0:
        gripper = -1.0

    action = [vx, vy, vz, 0, 0, 0] + [gripper]
    #print("Action:", action)
    return action

def main():
    global cx, cy, cz
    robot_interface = FrankaInterface("config/charmander.yml"
                                          , use_visualizer=False)
    controller_type = "CARTESIAN_VELOCITY"
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

    cx = cy = cz = 0.0

    for _ in range(5):
        cx += 0.001
        cy += 0.001
        cz += 0.001
        robot_interface.control(controller_type=controller_type,
                                action=[cx, cy, cz, 0, 0, 0] + [-1],
                                controller_cfg=controller_cfg,
                                )
            
    try:
        while True:
            # Hand tracking data
            hand_data = receive_hand_positions()
            if hand_data['grab_strength'] > 0.8:
                cx *= 0.5
                cy *= 0.5
                cz *= 0.5
                action = [cx, cy, cz, 0, 0, 0] + [-1]
            else:
                if hand_data is None:
                    action = [cx, cy, cz, 0, 0, 0] + [-1]
                else:
                    action = velocity_move(hand_data)

            robot_interface.control(controller_type=controller_type,
                                    action=action,
                                    controller_cfg=controller_cfg,
                                    )
            #print('Velocities:', cx, cy, cz)
    except KeyboardInterrupt:
        print("Program stopped by user.")
    finally:
        print("Closing robot interface.")
        robot_interface.close()


if __name__ == "__main__":
    main()