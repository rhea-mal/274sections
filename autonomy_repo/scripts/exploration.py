#!/usr/bin/env python3

import numpy as np
import rclpy
from scipy.interpolate import splev
from asl_tb3_msgs.msg import TurtleBotControl
from asl_tb3_msgs.msg import TurtleBotState
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from asl_tb3_lib . navigation import BaseNavigator
from asl_tb3_lib . navigation import TrajectoryPlan
from asl_tb3_lib . math_utils import wrap_angle
from asl_tb3_lib . tf_utils import quaternion_to_yaw
from asl_tb3_lib.grids import snap_to_grid, StochOccupancyGrid2D
import scipy
import typing as T
import matplotlib.pyplot as plt
from scipy.signal import convolve2d
import sys
from rclpy.node import Node








class Exploration(Node):
    def __init__(self, name='Exporation') -> None:
        super().__init__(name)


        self.curr_state = None
        self.occupancy = None
        self.nav_success = None
        self.exporation_finished = False
        
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.state_sub = self.create_subscription(TurtleBotState, "/state", self.state_callback, 10)
        self.nav_success_sub = self.create_subscription(Bool, "/nav_success", self.nav_success_callback, 10)
        self.detector_bool_sub = self.create_subscription(Bool, "/detector_bool", self.detector_bool_callback, 10)

        self.next_pt_pub = self.create_publisher(TurtleBotState, "/cmd_nav", 10)
        self.nav_success_pub = self.create_publisher(Bool, "/nav_success", 10)

        self.active=True
        self.detect_time=0.0
        self.last_time_true = -2.0
        self.first_publish = False



    def map_callback(self, msg: OccupancyGrid) -> None:
        if self.nav_success == None:
            self.nav_success_pub.publish(Bool(data=True))
        self.occupancy = StochOccupancyGrid2D(
            resolution=msg.info.resolution,
            size_xy=np.array([msg.info.width, msg.info.height]),
            origin_xy=np.array([msg.info.origin.position.x, msg.info.origin.position.y]),
            window_size=9,
            probs=msg.data
        )


    def state_callback(self, state: TurtleBotState):
        self.curr_state = state
    

    def nav_success_callback(self, msg: Bool):
        self.nav_success = msg.data
        if self.nav_success and self.active:
            self.go_to_closest_frontier()


    def detector_bool_callback(self, msg: Bool):
        self.detector_bool = msg.data
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.detector_bool and current_time - self.last_time_true > 2:
            if self.active:
                self.active = False
                self.detect_time = current_time
                self.first_publish = True
        
        if current_time < self.detect_time + 5:
            self.next_pt_pub.publish(self.curr_state)
        else: 
            self.active = True
            if self.first_publish:
                self.nav_success_pub.publish(Bool(data=True))
                self.first_publish = False
                self.last_time_true = current_time



        


    def go_to_closest_frontier(self):
        
        if self.occupancy is not None and self.curr_state is not None and not self.exporation_finished:
            frontier_states, closest_state = self.explore(self.occupancy, self.curr_state)

            
            if not self.exporation_finished:
                message = TurtleBotState()
                message.x = closest_state[0]
                message.y = closest_state[1]
                message.theta = 0.0

                self.next_pt_pub.publish(message)


    def explore(self, occupancy, current_state):
        """ returns potential states to explore
        Args:
            occupancy (StochasticOccupancyGrid2D): Represents the known, unknown, occupied, and unoccupied states. See class in first section of notebook.

        Returns:
            frontier_states (np.ndarray): state-vectors in (x, y) coordinates of potential states to explore. Shape is (N, 2), where N is the number of possible states to explore.

        HINTS:
        - Function `convolve2d` may be helpful in producing the number of unknown, and number of occupied states in a window of a specified cell
        - Note the distinction between physical states and grid cells. Most operations can be done on grid cells, and converted to physical states at the end of the function with `occupancy.grid2state()`
        """

        window_size = 13    # defines the window side-length for neighborhood of cells to consider for heuristics
        ########################### Code starts here ###########################

        unknown_thresh = 0.2
        unoccupied_thresh = 0.3

        unknown_cells = occupancy.probs == -1
        occupied_cells = occupancy.probs >= occupancy.thresh
        unoccupied_cells = (occupancy.probs < occupancy.thresh) & (occupancy.probs != -1)

        kernel = np.ones((window_size, window_size))
        unknown_counts = convolve2d(unknown_cells, kernel, mode='same', boundary='fill', fillvalue=1)
        occupied_counts = convolve2d(occupied_cells, kernel, mode='same', boundary='fill', fillvalue=1)
        unoccupied_counts = convolve2d(unoccupied_cells, kernel, mode='same', boundary='fill', fillvalue=1)
        total_cells = window_size ** 2

        valid_frontier = (
            (unknown_counts >= unknown_thresh * total_cells) &
            (occupied_counts == 0) &
            (unoccupied_counts >= unoccupied_thresh * total_cells)
        )

        frontier_states = np.array([
            occupancy.grid2state(np.array([y, x]))
            for x, y in zip(*np.where(valid_frontier))
        ])


        current_state = [current_state.x, current_state.y]

        if frontier_states.shape[0] == 0:
            self.exporation_finished = True
            return frontier_states, current_state
        

        min_state = frontier_states[np.argmin(np.linalg.norm(frontier_states - current_state, axis=1))]


        ########################### Code ends here ###########################
        return frontier_states, min_state


if __name__ == "__main__":
    rclpy.init()      
    node = Exploration()  
    rclpy.spin(node) 
    rclpy.shutdown()    
