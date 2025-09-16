from deoxys.franka_interface import FrankaInterface
from deoxys.utils.config_utils import get_default_controller_config
import numpy as np

robot_interface = FrankaInterface("config/charmander.yml"
                                          , use_visualizer=False)
controller_type = "OSC_POSE"
controller_cfg = get_default_controller_config(controller_type)
try:
    while True:
        # current pose
        current_quat, current_pos = robot_interface.last_eef_quat_and_pos

        robot_interface.control(controller_type=controller_type,
                                        action=np.array([0.0, 0.0, 0.0]).tolist() + np.array([0.1, 0, 0]).tolist() + np.array([0]).tolist(),
                                        controller_cfg=controller_cfg,
                                    )
        print("current_pose:", current_quat)
except KeyboardInterrupt:
        print("Program stopped by user.")
finally:
        print("Closing robot interface.")
        robot_interface.close()

robot_interface.close()