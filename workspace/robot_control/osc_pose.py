import numpy as np
import robosuite as suite
import time
from receiveHandPositions import receive_hand_positions

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

def main():
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

        # calculate delta position
        current_pos = obs["robot0_eef_pos"]
        target_pos = np.array([hand_data['z']*X_POS_SCALE,
                            hand_data['x']*Y_POS_SCALE,
                            hand_data['y']*Z_POS_SCALE])
        delta_pos = target_pos - current_pos

        # calculate delta rotation
        hand_ori = hand_data['orientation']
        current_rot = obs["robot0_eef_quat"]
        target_rot = np.array([hand_ori['z']*X_ROT_SCALE,
                               hand_ori['x']*Y_ROT_SCALE,
                               hand_ori['y']*Z_ROT_SCALE,])
        delta_rot = target_rot - current_rot[:3]

        # gripper
        gripper = np.array([hand_data['pinch_strength']])
        if gripper == 0:
            gripper = np.array([-1.0])

        # Action: [dx, dy, dz, droll, dpitch, dyaw, gripper]
        action = np.concatenate([delta_pos, np.zeros(3), gripper])

        obs, reward, done, info = env.step(action)
        env.render()

        elapsed = time.time() - start
        diff = 1 / MAX_FR - elapsed
        if diff > 0:
            time.sleep(diff)

if __name__ == "__main__":
    main()