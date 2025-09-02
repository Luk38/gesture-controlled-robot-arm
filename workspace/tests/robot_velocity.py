from deoxys.franka_interface import FrankaInterface
from deoxys.utils.config_utils import get_default_controller_config
import numpy as np

robot_interface = FrankaInterface("config/charmander.yml"
                                          , use_visualizer=False)
controller_type = "CARTESIAN_VELOCITY"
controller_cfg = get_default_controller_config(controller_type)
try:
        vx = 0.0
        for _ in range(100):
                vx += 0.01
                vx = np.clip(vx, -0.1, 0.1)
                action = [0.0, 0.0, 0.0, vx, 0, 0] + [-1]
                robot_interface.control(controller_type=controller_type,
                                        action=action,
                                        controller_cfg=controller_cfg
                                        )
                print("action:", action)
                rot, pos = robot_interface.last_eef_quat_and_pos
                #print("current position:", pos)
except KeyboardInterrupt:
        print("Program stopped by user.")
finally:
        print("Closing robot interface.")
        robot_interface.close()