#!/usr/bin/env python3

import numpy as np
import rclpy
from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl
from asl_tb3_msgs.msg import TurtleBotState


class HeadingController(BaseHeadingController):
    def __init__(self) -> None:
        super().__init__()
        self.kp = 2.0

    def compute_control_with_goal(self, curr_state: TurtleBotState, des_state: TurtleBotState) -> TurtleBotControl:
        curr_theta = curr_state.theta
        goal_theta = des_state.theta

        wrapped_err = wrap_angle(goal_theta - curr_theta)

        omega = wrapped_err * self.kp

        return_msg = TurtleBotControl()

        return_msg.omega = omega

        return return_msg



if __name__ == "__main__":
    rclpy.init()      
    node = HeadingController()  
    rclpy.spin(node) 
    rclpy.shutdown()    
