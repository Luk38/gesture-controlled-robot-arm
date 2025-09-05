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

# State Vektoren
vel_lin = np.zeros(3)   # [vx, vy, vz]
vel_rot = np.zeros(3)   # [rx, ry, rz]

hand_data = None
lock = threading.Lock()

def smooth_velocity(current, nxt, alpha=0.5):
    return alpha * nxt + (1 - alpha) * current

def acceleration_limiter(current, nxt, max_acceleration=0.02):
    delta = nxt - current
    delta = np.clip(delta, -max_acceleration, max_acceleration)
    return current + delta

def jerk_limiter(current_acc, next_acc, max_jerk=0.04):
    delta_a = next_acc - current_acc
    delta_a = np.clip(delta_a, -max_jerk * DT, max_jerk * DT)
    return current_acc + delta_a

def velocity_move(hd, current_pose):
    global vel_lin, vel_rot

    v_max = 0.1
    scale_lin = 0.0005

    # Raw linear desired from hand
    raw_lin = np.array([
        scale_lin * (hd['z'] + X_OFFSET),
        scale_lin * (hd['x'] + Y_OFFSET),
        scale_lin * (hd['y'] + Z_OFFSET),
    ])
    v_smoothed = smooth_velocity(vel_lin, raw_lin, 0.5)
    v_limited = acceleration_limiter(vel_lin, v_smoothed, 0.02)
    v_limited = np.clip(v_limited, -v_max, v_max)

    v_limited = np.where(np.abs(v_limited) < 0.001, 0.0, v_limited)
    vel_lin[:] = v_limited

    # Rotation
    axis_inv = -1
    hand_quat = np.array([
        hd['orientation']['w'],
        hd['orientation']['y'] * axis_inv,
        hd['orientation']['x'] * axis_inv,
        hd['orientation']['z'] * axis_inv
    ])
    current_rot = current_pose[:3, :3]
    current_quat = transform_utils.mat2quat(current_rot)
    if np.dot(hand_quat, current_quat) < 0.0:
        current_quat = -current_quat
    quat_diff = transform_utils.quat_distance(hand_quat, current_quat)
    axis_angle_diff = transform_utils.quat2axisangle(quat_diff)
    action_axis_angle = axis_angle_diff.flatten()

    vel_smoothed_rot = smooth_velocity(vel_rot, action_axis_angle, 0.5)
    vel_limited_rot = acceleration_limiter(vel_smoothed_rot, action_axis_angle, 0.04)
    vel_rot = np.clip(vel_limited_rot, -0.1, 0.1)
    vel_rot = np.where(np.abs(vel_rot) < 0.001, 0.0, vel_rot)

    gripper = hd['pinch_strength']
    if gripper == 0:
        gripper = -1.0

    #vel_rot = np.zeros(3)
    vel_lin = np.zeros(3)
    action = vel_lin.tolist() + vel_rot.tolist() + [gripper]
    return action

def receiver_loop(stop_event):
    global hand_data
    while not stop_event.is_set():
        data = receive_hand_positions()
        with lock:
            hand_data = data
    print("receiver loop closed")

def main():
    global vel_lin, vel_rot, hand_data

    robot_interface = FrankaInterface("config/charmander.yml", use_visualizer=False)
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

    # Soft start
    for _ in range(5):
        vel_lin += np.array([0.001, 0.001, 0.001])
        vel_rot += np.array([0.001, 0.001, 0.001])
        robot_interface.control(
            controller_type=controller_type,
            action=vel_lin.tolist() + vel_rot.tolist() + [-1],
            controller_cfg=controller_cfg
        )

    stop_event = threading.Event()
    receive_thread = threading.Thread(target=receiver_loop, args=(stop_event,))
    receive_thread.start()

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
                action = vel_lin.tolist() + vel_rot.tolist() + [-1]
            else:
                if hd['grab_strength'] > 0.8:
                    vel_lin *= 0.001
                    vel_rot[:] *= 0.001
                    action = vel_lin.tolist() + vel_rot.tolist() + [-1]
                else:
                    #print(axis_angle)
                    action = velocity_move(hd, current_pose=robot_interface.last_eef_pose)

            robot_interface.control(
                controller_type=controller_type,
                action=action,
                controller_cfg=controller_cfg
            )
    except KeyboardInterrupt:
        print("Program stopped by user.")
    finally:
        print("closing receive loop")
        stop_event.set()
        receive_thread.join()
        hand_data = None
        print("Closing robot interface.")
        robot_interface.close()

if __name__ == "__main__":
    main()