#!/usr/bin/env python3

import numpy as np
import rclpy

from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState
from std_msgs.msg import Bool

class PerceptionController(BaseHeadingController):
    def __init__(self) -> None:
        super().__init__("perception_controller")
        self.declare_parameter("kp", 2.0)
        self.declare_parameter("active", True)
        self.image_detected = False

        self.image_sub = self.create_subscription(
            Bool, "/detector_bool",
            self.image_callback,
            10
        )

    @property
    def kp(self) -> float:
        return self.get_parameter("kp").value


    @property
    def active(self) -> float:
        return self.get_parameter("active").value


    def compute_control_with_goal(self, current_state: TurtleBotState, desired_state: TurtleBotState) -> TurtleBotControl:
        err= wrap_angle(desired_state.theta - current_state.theta)
        instructions=TurtleBotControl()

        if self.image_detected:
            instructions.omega=0.0
        else:
            instructions.omega=0.2


        return instructions


    def image_callback(self, msg: bool) -> None:
        if msg.data:
            self.image_detected = True

if __name__ == "__main__":
    rclpy.init()
    node = PerceptionController()
    rclpy.spin(node)
    rclpy.shutdown()
