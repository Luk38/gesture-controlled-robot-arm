import numpy as np
import robosuite as suite
import time
from receiveHandPositions import receive_hand_positions
    
# create environment instance
env = suite.make(
    env_name="Lift", 
    robots="Panda",  
    has_renderer=True,
    ignore_done=True,
    control_freq=60,
    has_offscreen_renderer=False,
    use_camera_obs=False,
)
env.reset()

obs = env.reset()

MAX_FR = 60 # Maximum frame rate
while True:
    start = time.time()

    hand_data = receive_hand_positions()

    # calculate delta position
    current_pos = obs["robot0_eef_pos"]
    target_pos = np.array([hand_data['z']/100, hand_data['x']/100, hand_data['y']/150])
    delta_pos = target_pos - current_pos

    # calculate delta rotation
    # hand_ori = hand_data['orientation']
    # current_rot = obs["robot0_eef_quat"]
    # target_rot = np.array([hand_ori['z']/10, hand_ori['x']/10, hand_ori['y']/10])
    # delta_rot = target_rot - current_rot[:3]

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
