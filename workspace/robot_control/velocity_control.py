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

X_ROT_SCALE = -0.1
Y_ROT_SCALE = 0.04
Z_ROT_SCALE = -0.02

DT = 0.05

# Global variables
global translation
translation = np.array([0.0, 0.0, 0.0])
global rotation
rotation = np.array([0.0, 0.0, 0.0])
global velocities
velocities = translation.tolist() + rotation.tolist()

global cx 
global cy
global cz
cx = cy = cz = 0.0
global crx
global cry
global crz
crx = cry = crz = 0.0

global v_current
v_current = np.array([cx, cy, cz])
global a_current
a_current = np.array([0.0, 0.0, 0.0])

global hand_data
hand_data = None

lock = threading.Lock()

def smooth_velocity(current_velocity, next_velocity, alpha=0.5):
    v_smoothed = alpha * next_velocity + (1 - alpha) * current_velocity # exponential smoothing
    
    return v_smoothed

def acceleration_limiter(current_velocity, next_velocity, max_acceleration = 0.02):
    delta_v = next_velocity - current_velocity
    delta_v = np.clip(delta_v, -max_acceleration, max_acceleration)

    return current_velocity + delta_v

def jerk_limiter(current_acceleration, next_acceleration, max_jerk=0.04):
    delta_a = next_acceleration - current_acceleration
    delta_a = np.clip(delta_a, -max_jerk*DT, max_jerk*DT)
    
    return current_acceleration + delta_a

def velocity_move(hand_data):
    global translation, rotation, velocities

    global cx, cy, cz
    global crx, cry, crz
    #global v_current, a_current
    v_max = 0.1
    scale = 0.0005

    # exponential smoothing + acceleration limiter
    
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
    if np.abs(vx) < 0.001:
        vx = 0
    if np.abs(vy) < 0.001:
        vy = 0
    if np.abs(vz) < 0.001:
        vz = 0

    cx = vx
    cy = vy
    cz = vz

    # Jerk limiter
    # v = (vx, vy, vz)
    # v_current = np.array([cx, cy, cz])
    # a_current = np.array([0.0, 0.0, 0.0])

    # v_cmd = np.array([
    #     scale * (hand_data['z'] + X_OFFSET),
    #     scale * (hand_data['x'] + Y_OFFSET),
    #     scale * (hand_data['y'] + Z_OFFSET)
    # ])

    # a_des = (v_cmd - v_current) / DT

    # a_cmd = jerk_limiter(a_current, a_des)
    # a_cmd = np.clip(a_cmd, -0.2, 0.2)


    # v_current += a_cmd * DT

    # a_current = a_cmd

    # vx = v_current[0]
    # vy = v_current[1]
    # vz = v_current[2]

    hand_quat = np.array([
        hand_data['orientation']['w'],
        hand_data['orientation']['y'],
        hand_data['orientation']['x'],
        hand_data['orientation']['z']
    ])

    axis_angle = transform_utils.quat2axisangle(hand_quat)
    #print(axis_angle)
    scale_rot = 10
    rx = (axis_angle[0] - 3) * X_ROT_SCALE * scale_rot
    ry = (axis_angle[2]) * Y_ROT_SCALE * scale_rot
    rz = (axis_angle[1] + 0.01) * Z_ROT_SCALE * scale_rot

    print('raw', rx, ry, rz)

    if np.abs(rx) < 0.01 * scale_rot:
        rx = 0
    if np.abs(ry) < 0.01 * scale_rot:
        ry = 0
    if np.abs(rz) < 0.01 * scale_rot:
        rz = 0
    print('close range limited', rx, ry, rz)

    rx = smooth_velocity(crx, rx, 0.5)
    ry = smooth_velocity(cry, ry, 0.5)
    rz = smooth_velocity(crz, rz, 0.5)

    print('smoothed', rx, ry, rz)

    rx = acceleration_limiter(crx, rx, 0.05)
    ry = acceleration_limiter(cry, ry, 0.05)
    rz = acceleration_limiter(crz, rz, 0.05)

    print('acc limited',rx, ry, rz)

    crx = rx
    cry = ry
    crz = rz

    gripper = hand_data['pinch_strength']
    if gripper == 0:
        gripper = -1.0

    action = [vx, vy, vz, rx, ry, rz] + [gripper]
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
    global crx, cry, crz
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

    last = time.perf_counter()

    try:
        while True:
            now = time.perf_counter()
            dt = now - last
            #print('dt: ', dt)
            freq = 1.0 / dt if dt > 0 else float('inf')
            last = now
            quat, pos = robot_interface.last_eef_quat_and_pos
            axis_angle = transform_utils.quat2axisangle(quat)
            with lock:
                hd = hand_data
            if hd is None:
                action = [cx, cy, cz, 0, 0, 0] + [-1]
            else: 
                if hd['grab_strength'] > 0.8:
                    cx *= 0.5
                    cy *= 0.5
                    cz *= 0.5
                    cx *= 0.001
                    cy *= 0.001
                    cz *= 0.001
                    action = [cx, cy, cz, 0, 0, 0] + [-1]
                else:
                    #print(axis_angle)
                    action = velocity_move(hd)

            robot_interface.control(controller_type=controller_type,
                                    action=action,
                                    controller_cfg=controller_cfg,
                                    )

            #print('Velocities:', crx, cry, crz)
            last = now
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