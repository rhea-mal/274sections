#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# import the message type to use
from std_msgs.msg import Int64, Bool


class Timer(Node):
    def __init__(self) -> None:
				# initialize base class (must happen before everything else)
        super().__init__("cmd_vel")

				# create publisher with: self.create_publisher(<msg type>, <topic>, <qos>)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        self.sub = self.create_subscription(Bool, "/kill", self.sub_kill, 10)
        
        # create a timer with: self.create_timer(<second>, <callback>)
        self.timer = self.create_timer(0.2, self.pub_twist)


    def pub_twist(self):
        msg = Twist()  # 0 initialize everything by default
        msg.linear.x = 1.0  # set this to be the linear velocity
        msg.angular.z = 1.0 # set this to be the angular velocity
        self.pub.publish(msg)

    def sub_kill(self, msg: Bool):
        if msg.data:
            override = Twist() 
            override.linear.x = 0.0  # set this to be the linear velocity
            override.angular.z = 0.0 # set this to be the angular velocity
            self.pub.publish(override)
            self.timer.cancel()
    
if __name__ == "__main__":
    rclpy.init()        # initialize ROS2 context (must run before any other rclpy call)
    node = Timer()  # instantiate the heartbeat node
    rclpy.spin(node)    # Use ROS2 built-in schedular for executing the node
    rclpy.shutdown()    # cleanly shutdown ROS2 context