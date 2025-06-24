from robosuite.controllers.composite.composite_controller import CompositeController, register_composite_controller
from deoxys.franka_interface import FrankaInterface

@register_composite_controller
class DeoxysController(CompositeController):
    name = "DEOXYS_OSC_POSE"

    def __init__(self, sim, robot_model, grippers):
            super().__init__(sim, robot_model, grippers)
            self._init_joint_action_policy()

    def _init_joint_action_policy(self):
        self.joint_action_policy = DeoxysOSCPosePolicy()

class DeoxysOSCPosePolicy:
    def __init__(self):
        pass

    def __call__(self, action):
        return action