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

def velocity_move(hand_data):
    v_max = 0.1
    scale = 0.0004
    vx = scale * (hand_data['z'] + X_OFFSET)
    vy = scale * (hand_data['x'] + Y_OFFSET)
    vz = scale * (hand_data['y'] + Z_OFFSET)

    vx = np.clip(vx, -v_max, v_max)
    vy = np.clip(vy, -v_max, v_max)
    vz = np.clip(vz, -v_max, v_max)

    hand_quat = np.array([
        hand_data['orientation']['w'],
        hand_data['orientation']['y'],
        hand_data['orientation']['x'],
        hand_data['orientation']['z']
    ])
    axis_angle = transform_utils.quat2axisangle(hand_quat)
    print(axis_angle)
    rx = (axis_angle[0] - 3) * X_ROT_SCALE
    ry = (axis_angle[2]) * Y_ROT_SCALE
    rz = (axis_angle[1]) * Z_ROT_SCALE

    rx = np.clip(rx, -v_max, v_max)
    ry = np.clip(ry, -v_max, v_max)
    rz = np.clip(rz, -v_max, v_max)

    action = [vx, vy, vz, -rx, ry, rz] + [-1]
    #print("Action:", action)
    return action

def main():
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
    try:
        vx = 0.0
        for _ in range(20):
            vx += 0.001
            robot_interface.control(controller_type=controller_type,
                                    action=[vx, vx, -vx, 0, 0, 0] + [-1],
                                    controller_cfg=controller_cfg,
                                    )
        while True:
            # Hand tracking data
            hand_data = receive_hand_positions()

            robot_interface.control(controller_type=controller_type,
                                    action=velocity_move(hand_data),
                                    controller_cfg=controller_cfg,
                                    )
    except KeyboardInterrupt:
        print("Program stopped by user.")
    finally:
        print("Closing robot interface.")
        robot_interface.close()



if __name__ == "__main__":
    main()