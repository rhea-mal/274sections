#!/usr/bin/env python3
import rclpy
from asl_tb3_lib.control import BaseController
from asl_tb3_msgs.msg import TurtleBotControl
from std_msgs.msg import Bool

class PerceptionController(BaseController):
    def __init__(self) -> None:
        super().__init__("perception_controller")
        self.declare_parameter("active", True)
        self.time_at_stop = None
        self.sub = self.create_subscription(Bool, "/detector_bool", self.stop_robot, 10)
        self.last_time_true = 0.0

    @property
    def active(self)->bool:
        return self.get_parameter("active").value
    

    def stop_robot(self, msg:Bool):
        # self.get_logger().info(f"{msg}")
        current_time = self.get_clock().now().nanoseconds / 1e9
        if msg.data and current_time - self.last_time_true > 2:
            self.set_parameters([rclpy.Parameter("active", value=False)])



    def compute_control(self) -> TurtleBotControl:

        current_time = self.get_clock().now().nanoseconds / 1e9

        return_msg = TurtleBotControl()
        if self.active:
            return_msg.omega = 0.5
            
        else:
            if not self.time_at_stop:
                self.time_at_stop = current_time + 5
            if current_time < self.time_at_stop:
                return_msg.omega=0.0
            else:
                self.set_parameters([rclpy.Parameter("active", value=True)])
                return_msg.omega=0.5
                self.time_at_stop = None
                self.last_time_true = current_time          

        return return_msg



if __name__ == "__main__":
    rclpy.init()      
    node = PerceptionController()  
    rclpy.spin(node) 
    rclpy.shutdown()    
