#!/usr/bin/env python3

import numpy as np
import rclpy
from scipy.interpolate import splev
from asl_tb3_msgs.msg import TurtleBotControl
from asl_tb3_msgs.msg import TurtleBotState
from asl_tb3_lib . navigation import BaseNavigator
from asl_tb3_lib . navigation import TrajectoryPlan
from asl_tb3_lib . math_utils import wrap_angle
from asl_tb3_lib . tf_utils import quaternion_to_yaw
from asl_tb3_lib.grids import snap_to_grid, StochOccupancyGrid2D
import scipy



class AStar(object):
    """Represents a motion planning problem to be solved using A*"""

    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy, resolution=1):
        self.statespace_lo = statespace_lo         # state space lower bound (e.g., [-5, -5])
        self.statespace_hi = statespace_hi         # state space upper bound (e.g., [5, 5])
        self.occupancy = occupancy                 # occupancy grid (a DetOccupancyGrid2D object)
        self.resolution = resolution               # resolution of the discretization of state space (cell/m)
        self.x_offset = x_init                     
        self.x_init = self.snap_to_grid(x_init)    # initial state
        self.x_goal = self.snap_to_grid(x_goal)    # goal state

        self.closed_set = set()    # the set containing the states that have been visited
        self.open_set = set()      # the set containing the states that are condidate for future expension

        self.est_cost_through = {}  # dictionary of the estimated cost from start to goal passing through state (often called f score)
        self.cost_to_arrive = {}    # dictionary of the cost-to-arrive at state from start (often called g score)
        self.came_from = {}         # dictionary keeping track of each state's parent to reconstruct the path

        self.open_set.add(self.x_init)
        self.cost_to_arrive[self.x_init] = 0
        self.est_cost_through[self.x_init] = self.distance(self.x_init,self.x_goal)

        self.path = None        # the final path as a list of states

    def is_free(self, x):
        """
        Checks if a give state x is free, meaning it is inside the bounds of the map and
        is not inside any obstacle.
        Inputs:
            x: state tuple
        Output:
            Boolean True/False
        Hint: self.occupancy is a DetOccupancyGrid2D object, take a look at its methods for what might be
              useful here
        """
        ########## Code starts here ##########

        return self.occupancy.is_free(np.array(x)) and \
               self.statespace_lo[0] <= x[0] <= self.statespace_hi[0] and \
               self.statespace_lo[1] <= x[1] <= self.statespace_hi[1]
        ########## Code ends here ##########

    def distance(self, x1, x2):
        """
        Computes the Euclidean distance between two states.
        Inputs:
            x1: First state tuple
            x2: Second state tuple
        Output:
            Float Euclidean distance

        HINT: This should take one line. Tuples can be converted to numpy arrays using np.array().
        """
        ########## Code starts here ##########
        return np.linalg.norm(np.array(x1) - np.array(x2), 2)
        ########## Code ends here ##########

    def snap_to_grid(self, x):
        """ Returns the closest point on a discrete state grid
        Input:
            x: tuple state
        Output:
            A tuple that represents the closest point to x on the discrete state grid
        """
        return (
            self.resolution * round((x[0] - self.x_offset[0]) / self.resolution) + self.x_offset[0],
            self.resolution * round((x[1] - self.x_offset[1]) / self.resolution) + self.x_offset[1],
        )

    def get_neighbors(self, x):
        """
        Gets the FREE neighbor states of a given state x. Assumes a motion model
        where we can move up, down, left, right, or along the diagonals by an
        amount equal to self.resolution.
        Input:
            x: tuple state
        Ouput:
            List of neighbors that are free, as a list of TUPLES

        HINTS: Use self.is_free to check whether a given state is indeed free.
               Use self.snap_to_grid (see above) to ensure that the neighbors
               you compute are actually on the discrete grid, i.e., if you were
               to compute neighbors by adding/subtracting self.resolution from x,
               numerical errors could creep in over the course of many additions
               and cause grid point equality checks to fail. To remedy this, you
               should make sure that every neighbor is snapped to the grid as it
               is computed.
        """
        neighbors = []
        ########## Code starts here ##########
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if i != 0 or j != 0:
                    neighbor = self.snap_to_grid((x[0] + i*self.resolution, x[1] + j*self.resolution))
                    if self.is_free(neighbor):
                        neighbors.append(neighbor)
        ########## Code ends here ##########
        return neighbors

    def find_best_est_cost_through(self):
        """
        Gets the state in open_set that has the lowest est_cost_through
        Output: A tuple, the state found in open_set that has the lowest est_cost_through
        """
        return min(self.open_set, key=lambda x: self.est_cost_through[x])

    def reconstruct_path(self):
        """
        Use the came_from map to reconstruct a path from the initial location to
        the goal location
        Output:
            A list of tuples, which is a list of the states that go from start to goal
        """
        path = [self.x_goal]
        current = path[-1]
        while current != self.x_init:
            path.append(self.came_from[current])
            current = path[-1]
        return list(reversed(path))


    def solve(self):
        """
        Solves the planning problem using the A* search algorithm. It places
        the solution as a list of tuples (each representing a state) that go
        from self.x_init to self.x_goal inside the variable self.path
        Input:
            None
        Output:
            Boolean, True if a solution from x_init to x_goal was found

        HINTS:  We're representing the open and closed sets using python's built-in
                set() class. This allows easily adding and removing items using
                .add(item) and .remove(item) respectively, as well as checking for
                set membership efficiently using the syntax "if item in set".
        """
        ########## Code starts here ##########
        solution_found = False

        print(len(self.open_set))

        while len(self.open_set) > 0:
            x_curr = self.find_best_est_cost_through()

            if x_curr == self.x_goal:
                solution_found = True
                break
            
            self.open_set.remove(x_curr)
            self.closed_set.add(x_curr)

            curr_neighbors = self.get_neighbors(x_curr)

            for neighbor in curr_neighbors:
                if neighbor in self.closed_set:
                    continue

                tent_cost_to_arrive = self.cost_to_arrive[x_curr] + self.distance(x_curr, neighbor)

                if neighbor not in self.open_set:
                    self.open_set.add(neighbor) 
                elif tent_cost_to_arrive > self.cost_to_arrive[neighbor]:
                    continue

                self.came_from[neighbor] = x_curr
                print('neighbor key', neighbor, 'x_curr value', x_curr)
                self.cost_to_arrive[neighbor] = tent_cost_to_arrive
                self.est_cost_through[neighbor] = tent_cost_to_arrive + self.distance(neighbor, self.x_goal)
        
        self.path = self.reconstruct_path()

        return solution_found
    


def compute_smooth_plan(path, v_desired=0.15, spline_alpha=0.05, resolution=1) -> TrajectoryPlan:

    ts = np.linspace(0, path.shape[0]*resolution/v_desired, path.shape[0])
    path_x_spline = scipy.interpolate.splrep(x=ts, y=path[:, 0], s=spline_alpha)
    path_y_spline = scipy.interpolate.splrep(x=ts, y=path[:, 1], s=spline_alpha)
    
    return TrajectoryPlan(
        path=path,
        path_x_spline=path_x_spline,
        path_y_spline=path_y_spline,
        duration=ts[-1],
    )



class Navigator(BaseNavigator):
    def __init__(self, kpx=1.75, kpy=1.75, kdx=1.75, kdy=1.75, V_PREV_THRESH=0.001) -> None:
        super().__init__()
        self.kp = 2.0

        self.kpx = kpx
        self.kpy = kpy
        self.kdx = kdx
        self.kdy = kdy

        self.V_PREV_THRESH = V_PREV_THRESH


        self.V_prev = 0.
        self.om_prev = 0.
        self.t_prev = 0.



    def compute_heading_control(self, curr_state: TurtleBotState, des_state: TurtleBotState) -> TurtleBotControl:
        curr_theta = curr_state.theta
        goal_theta = des_state.theta

        wrapped_err = wrap_angle(goal_theta - curr_theta)

        omega = wrapped_err * self.kp

        return_msg = TurtleBotControl()

        return_msg.omega = omega

        return return_msg
    

    def compute_trajectory_tracking_control(self,
        state: TurtleBotState,
        plan: TrajectoryPlan,
        t: float,
    ) -> TurtleBotControl:
        """ Compute control target using a trajectory tracking controller

        Args:
            state (TurtleBotState): current robot state
            plan (TrajectoryPlan): planned trajectory
            t (float): current timestep

        Returns:
            TurtleBotControl: control command
        """

        dt = t - self.t_prev

        x_d = splev(t, plan.path_x_spline, der=0)
        y_d = splev(t, plan.path_y_spline, der=0)

        xd_d = splev(t, plan.path_x_spline, der=1)
        yd_d = splev(t, plan.path_y_spline, der=1)

        xdd_d = splev(t, plan.path_x_spline, der=2)
        ydd_d = splev(t, plan.path_y_spline, der=2)

        x = state.x
        y = state.y
        th = state.theta

        if t == 0:
            self.V_prev = np.sqrt(xd_d**2 + yd_d**2)

        if abs(self.V_prev) < self.V_PREV_THRESH:
            self.V_prev = self.V_PREV_THRESH

        u1 = xdd_d + self.kpx*(x_d-x) + self.kdx*(xd_d - self.V_prev*np.cos(th))
        u2 = ydd_d + self.kpy*(y_d-y) + self.kdy*(yd_d - self.V_prev*np.sin(th))

        b = np.array([u1, u2])

        J = np.array([[np.cos(th), -self.V_prev*np.sin(th)],
                      [np.sin(th),  self.V_prev*np.cos(th)]])
        
        aw = np.linalg.solve(J, b)

        V = self.V_prev + dt*aw[0]
        om = aw[1]

        if abs(V) < self.V_PREV_THRESH:
            V = self.V_PREV_THRESH

        self.t_prev = t
        self.V_prev = V
        self.om_prev = om

        control = TurtleBotControl()
        control.v = V
        control.omega = om

        return control
    

    def compute_trajectory_plan(self,
        state: TurtleBotState,
        goal: TurtleBotState,
        occupancy: StochOccupancyGrid2D,
        resolution: float,
        horizon: float):
        """ Compute a trajectory plan using A* and cubic spline fitting

        Args:
            state (TurtleBotState): state
            goal (TurtleBotState): goal
            occupancy (StochOccupancyGrid2D): occupancy
            resolution (float): resolution
            horizon (float): horizon

        Returns:
            T.Optional[TrajectoryPlan]:
        """

        x_init = (state.x, state.y)
        x_goal = (goal.x, goal.y)


        statespace_lo = (state.x-horizon, state.y-horizon)
        statespace_hi = (state.x+horizon, state.y+horizon)

        astar = AStar(statespace_lo, statespace_hi, x_init, x_goal, occupancy, resolution)

        solution = astar.solve()

        if not solution or (len(astar.path) < 4):
            print("No path found")
            return None
        
        self.V_prev = 0.
        self.om_prev = 0.
        self.t_prev = 0.

        plan = compute_smooth_plan(np.array(astar.path), v_desired = 0.15, spline_alpha = 0.15, resolution=resolution)

        return plan



if __name__ == "__main__":
    rclpy.init()      
    node = Navigator()  
    rclpy.spin(node) 
    rclpy.shutdown()    
