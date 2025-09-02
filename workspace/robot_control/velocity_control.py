import numpy as np
import threading, time
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

DT = 0.01

# Global variables
global cx 
global cy
global cz
cx = cy = cz = 0.0

global hand_data
hand_data = None

lock = threading.Lock()

def smooth_velocity(current_velocity, next_velocity, alpha=0.2):
    v_smoothed = alpha * next_velocity + (1 - alpha) * current_velocity # exponential smoothing
    
    return v_smoothed

def acceleration_limiter(current_velocity, next_velocity, max_acceleration=0.01):
    delta_v = next_velocity - current_velocity
    delta_v = np.clip(delta_v, -max_acceleration, max_acceleration)
    
    return current_velocity + delta_v

def jerk_limiter(current_acceleration, next_acceleration, max_jerk=0.01):
    delta_a = next_acceleration - current_acceleration
    delta_a = np.clip(delta_a, -max_jerk*DT, max_jerk*DT)
    
    return current_acceleration + delta_a

def velocity_move(hand_data):
    global cx, cy, cz
    v_max = 0.1
    scale = 0.0004 

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

    # Für zu kleine Bewegungen
    # kein Ruckeln
    if vx < 0.01 and vx > -0.01:
        vx = 0
    if vy < 0.01 and vy > -0.01:
        vy = 0
    if vz < 0.01 and vz > -0.01:
        vz = 0

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

    # max_acceleration = 0.2
    # max_delta_v = max_acceleration * DT
    # vx = acceleration_limiter(cx, vx)
    # vy = acceleration_limiter(cy, vy)
    # vz = acceleration_limiter(cz, vz)

    # ax = (vx - cx) / DT
    # ay = (vy - cy) / DT
    # az = (vz - cz) / DT
    # ax = jerk_limiter(0, ax)
    # ay = jerk_limiter(0, ay)
    # az = jerk_limiter(0, az)
    # vx = cx + ax * DT
    # vy = cy + ay * DT
    # vz = cz + az * DT

    # vx = np.clip(vx, -v_max, v_max)
    # vy = np.clip(vy, -v_max, v_max)
    # vz = np.clip(vz, -v_max, v_max)

    # cx = vx
    # cy = vy
    # cz = vz

    gripper = hand_data['pinch_strength']
    if gripper == 0:
        gripper = -1.0

    action = [vx, vy, vz, 0, 0, 0] + [gripper]
    #print("Action:", action)
    return action

def receiver_loop(stop_event):
    global hand_data
    while not stop_event.is_set():
        data = receive_hand_positions()
        with lock:
            hand_data = data
    print("receiver loop closed")

def main():
    global cx, cy, cz
    global hand_data

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

    # anfahren
    for _ in range(5):
        cx += 0.001
        cy += 0.001
        cz += 0.001
        robot_interface.control(controller_type=controller_type,
                                action=[cx, cy, cz, 0, 0, 0] + [-1],
                                controller_cfg=controller_cfg,
                                )
        
    stop_event = threading.Event()
    receive_loop = threading.Thread(target=receiver_loop, args=(stop_event,))
    receive_loop.start()

    try:
        while True:
            with lock:
                hd = hand_data
            if hd is None:
                action = [cx, cy, cz, 0, 0, 0] + [-1]
            else: 
                if hd['grab_strength'] > 0.8:
                    cx *= 0.5
                    cy *= 0.5
                    cz *= 0.5
                    action = [cx, cy, cz, 0, 0, 0] + [-1]
                else:
                    action = velocity_move(hd)

            robot_interface.control(controller_type=controller_type,
                                    action=action,
                                    controller_cfg=controller_cfg,
                                    )
            print('Velocities:', cx, cy, cz)
    except KeyboardInterrupt:
        print("Program stopped by user.")
    finally:
        print('closing receive loop')
        stop_event.set()
        receive_loop.join()
        hand_data = None

        print("Closing robot interface.")
        robot_interface.close()


if __name__ == "__main__":
    main()