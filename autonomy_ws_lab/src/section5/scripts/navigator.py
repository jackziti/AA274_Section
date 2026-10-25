#!/usr/bin/env python3

import typing as T

#Import math classes
import numpy as np
from numpy import linalg
from scipy.interpolate import splev, splrep

#Import from asl_tb3_lib
from asl_tb3_lib.navigation import BaseNavigator
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_lib.tf_utils import quaternion_to_yaw

#Import for ROS2
import rclpy                    # ROS2 client library
from rclpy.node import Node     # ROS2 node baseclass

class AStar(object):
    """Represents a motion planning problem to be solved using A*"""

    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy, resolution=1,
                 heuristic="l2", heuristic_weight=1.0):
        self.statespace_lo = statespace_lo         # state space lower bound (e.g., [-5, -5])
        self.statespace_hi = statespace_hi         # state space upper bound (e.g., [5, 5])
        self.occupancy = occupancy                 # occupancy grid (a DetOccupancyGrid2D object)
        self.resolution = resolution               # resolution of the discretization of state space (cell/m)
        self.heuristic_type = heuristic            # norm used by the heuristic: "l1", "l2", or "linf"
        self.heuristic_weight = heuristic_weight   # weight w >= 1 on the heuristic (w > 1 gives weighted A*)
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
        self.est_cost_through[self.x_init] = self.heuristic(self.x_init)

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
        for dim in range(len(x)):
            if x[dim] < self.statespace_lo[dim] or x[dim] > self.statespace_hi[dim]:
                return False
        # StochOccupancyGrid2D expects a numpy array, not a tuple
        return self.occupancy.is_free(np.asarray(x))
        ########## Code ends here ##########

    def distance(self, x1, x2):
        """
        Computes the Euclidean distance between two states. This is the cost of
        traveling along the edge between two neighboring states, and it is used
        for the cost-to-arrive. It should NOT depend on self.heuristic_type.
        Inputs:
            x1: First state tuple
            x2: Second state tuple
        Output:
            Float Euclidean distance

        HINT: This should take one line. Tuples can be converted to numpy arrays using np.array().
        """
        ########## Code starts here ##########
        return float(np.linalg.norm(np.array(x1) - np.array(x2)))
        ########## Code ends here ##########

    def heuristic(self, x):
        """
        Computes the weighted heuristic estimate of the cost-to-go from state x
        to self.x_goal, i.e. w * ||x - x_goal||, where w = self.heuristic_weight
        and the norm is selected by self.heuristic_type:
            "l1":   sum of absolute coordinate differences
            "l2":   Euclidean norm
            "linf": largest absolute coordinate difference
        Input:
            x: state tuple
        Output:
            Float heuristic value

        HINT: np.linalg.norm accepts an `ord` argument. Raise a ValueError for
              an unrecognized self.heuristic_type.
        """
        ########## Code starts here ##########
        ords = {"l1": 1, "l2": 2, "linf": np.inf}
        diff = np.array(x) - np.array(self.x_goal)
        return self.heuristic_weight * float(np.linalg.norm(diff, ord=ords[self.heuristic_type]))
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
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                neighbor = self.snap_to_grid((x[0] + dx * self.resolution, x[1] + dy * self.resolution))
                if self.is_free(neighbor):
                    neighbors.append(neighbor)
        return neighbors
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

                Use self.distance for edge costs (cost-to-arrive) and
                self.heuristic for the estimated cost-to-go, as in Algorithm 1
                of the handout.
        """
        ########## Code starts here ##########
        while len(self.open_set) > 0:
            x_current = self.find_best_est_cost_through()
            if x_current == self.x_goal:
                self.path = self.reconstruct_path()
                return True
            self.open_set.remove(x_current)
            self.closed_set.add(x_current)
            for x_neigh in self.get_neighbors(x_current):
                if x_neigh in self.closed_set:
                    continue
                tentative_cost_to_arrive = self.cost_to_arrive[x_current] + self.distance(x_current, x_neigh)
                if x_neigh not in self.open_set:
                    self.open_set.add(x_neigh)
                elif tentative_cost_to_arrive > self.cost_to_arrive[x_neigh]:
                    continue
                self.came_from[x_neigh] = x_current
                self.cost_to_arrive[x_neigh] = tentative_cost_to_arrive
                self.est_cost_through[x_neigh] = tentative_cost_to_arrive + self.heuristic(x_neigh)
        return False
        ########## Code ends here ##########


class Navigator(BaseNavigator):
    def __init__(self) -> None:
        # give it a default node name
        super().__init__("navigator")
        self.kp=2.0

        # trajectory tracking (migrated from HW2 trajectory_tracking.py)
        self.V_PREV_THRES = 0.0001
        self.kpx = 2.0
        self.kpy = 2.0
        self.kdx = 2.0
        self.kdy = 2.0
        self.V_prev = 0.
        self.om_prev = 0.
        self.t_prev = 0.

        # trajectory smoothing (from compute_smooth_plan in sim_astar.ipynb)
        self.v_desired = 0.15
        self.spline_alpha = 0.1

    def reset(self) -> None:
        self.V_prev = 0.
        self.om_prev = 0.
        self.t_prev = 0.

    def compute_heading_control(self, current_state: TurtleBotState, desired_state: TurtleBotState) -> TurtleBotControl:
        err= wrap_angle(desired_state.theta - current_state.theta)
        instructions=TurtleBotControl()
        instructions.omega=self.kp*err
        return instructions

    def compute_trajectory_tracking_control(self, state: TurtleBotState, plan: TrajectoryPlan, t: float) -> TurtleBotControl:
        """ Compute control target using a trajectory tracking controller

        Args:
            state (TurtleBotState): current robot state
            plan (TrajectoryPlan): planned trajectory
            t (float): current timestep

        Returns:
            TurtleBotControl: control command
        """
        dt = t - self.t_prev
        x_d = float(splev(t, plan.path_x_spline, der=0))
        xd_d = float(splev(t, plan.path_x_spline, der=1))
        xdd_d = float(splev(t, plan.path_x_spline, der=2))
        y_d = float(splev(t, plan.path_y_spline, der=0))
        yd_d = float(splev(t, plan.path_y_spline, der=1))
        ydd_d = float(splev(t, plan.path_y_spline, der=2))

        # avoid singularity
        if abs(self.V_prev) < self.V_PREV_THRES:
            self.V_prev = self.V_PREV_THRES

        th = state.th
        xd = self.V_prev*np.cos(th)
        yd = self.V_prev*np.sin(th)

        # compute virtual controls
        x = state.x
        y = state.y
        u = np.array([xdd_d + self.kpx*(x_d-x) + self.kdx*(xd_d-xd),
                      ydd_d + self.kpy*(y_d-y) + self.kdy*(yd_d-yd)])

        # compute real controls
        J = np.array([[np.cos(th), -self.V_prev*np.sin(th)],
                          [np.sin(th), self.V_prev*np.cos(th)]])
        a, om = linalg.solve(J, u)
        V = self.V_prev + a*dt

        # apply control limits
        V = np.clip(V, -self.V_max, self.V_max)
        om = np.clip(om, -self.om_max, self.om_max)

        # save the commands that were applied and the time
        self.t_prev = t
        self.V_prev = V
        self.om_prev = om

        return V, om

    def compute_trajectory_plan(self, state: TurtleBotState, goal: TurtleBotState, occupancy: StochOccupancyGrid2D,
                                resolution: float, horizon: float) -> T.Optional[TrajectoryPlan]:
        # A* path planning
        astar = AStar((state.x - horizon, state.y - horizon), (state.x + horizon, state.y + horizon),
                      (state.x, state.y), (goal.x, goal.y), occupancy, resolution=resolution)
        if not astar.solve() or len(astar.path) < 4:
            return None

        # reset tracking controller history
        self.reset()

        # path time computation and trajectory smoothing (from compute_smooth_plan)
        path = np.asarray(astar.path)
        ts = [0]
        for i in range(1, len(path)):
            distance=np.linalg.norm(path[i]-path[i-1])
            ts.append(ts[-1]+distance/self.v_desired)
        path_x_spline = splrep(ts, path[:,0], k=3, s=self.spline_alpha)
        path_y_spline = splrep(ts, path[:,1], k=3, s=self.spline_alpha)

        return TrajectoryPlan(
            path=path,
            path_x_spline=path_x_spline,
            path_y_spline=path_y_spline,
            duration=ts[-1],
        )


if __name__ == "__main__":
    rclpy.init()            # initialize ROS client library
    node = Navigator()      # create the node instance
    rclpy.spin(node)        # call ROS2 default scheduler
    rclpy.shutdown()        # clean up after node exits