#!/usr/bin/env python3

import typing as T

import numpy as np
import rclpy
import scipy.interpolate
from itertools import product
from numpy import linalg
from scipy.interpolate import splev

from asl_tb3_lib.navigation import BaseNavigator, TrajectoryPlan
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_lib.grids import StochOccupancyGrid2D
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState

# copied from HW1 P1_astar.py (plot_path / plot_tree removed since they need matplotlib)
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
        x_size = len(x)
        for i in range(x_size):
            if x[i] <= self.statespace_hi[i] and x[i] >= self.statespace_lo[i] and self.occupancy.is_free(x)==True: #not sure about the condition over self.occupancy
                continue
            else:
                return False
        return True

    def distance(self, x1, x2):
        return np.linalg.norm(np.array(x1) - np.array(x2))

    def heuristic(self, x):
        if self.heuristic_type=="l1":
         return self.heuristic_weight*np.linalg.norm(np.array(x)-np.array(self.x_goal),ord=1)
        elif self.heuristic_type=="l2":
         return self.heuristic_weight*np.linalg.norm(np.array(x)-np.array(self.x_goal), ord=2)
        elif self.heuristic_type=="linf":
         return self.heuristic_weight*np.linalg.norm(np.array(x)-np.array(self.x_goal), ord=np.inf)
        else:
         raise ValueError("Unrecognized heuristic type")

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
        neighbors = []
        n=len(x)
        test_neighbors_add = np.array(list(product([-1,0,1], repeat=n))) #the product function is from otertools, i am just doing that if we have 3 dimensions, that would be easier to create the matrix to test all the possibilities
        for elem in test_neighbors_add:
            if np.array_equal(elem, [0 for i in range(n)]):
                continue
            else:
                test_neighbor=tuple(np.array(x)+np.array(elem)*self.resolution)
                test_neighbor=self.snap_to_grid(test_neighbor)
                if self.is_free(test_neighbor):
                    neighbors.append(test_neighbor)
                else:
                    continue
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
        self.open_set.add(self.x_init)
        while len(self.open_set) > 0:
            current=self.find_best_est_cost_through()
            if current==self.x_goal:
                self.path=self.reconstruct_path()
                return True
            self.open_set.remove(current)
            self.closed_set.add(current)
            for neighbor in self.get_neighbors(current):
                if neighbor in self.closed_set:
                    continue
                tentative_cost_to_arrive=self.cost_to_arrive.get(current)+self.distance(current,neighbor)
                if neighbor not in self.open_set:
                    self.open_set.add(neighbor)
                elif tentative_cost_to_arrive > self.cost_to_arrive.get(neighbor):
                    continue
                self.came_from[neighbor]=current
                self.cost_to_arrive[neighbor]=tentative_cost_to_arrive
                self.est_cost_through[neighbor]=self.cost_to_arrive.get(neighbor)+self.heuristic(neighbor)
        return False

class Navigator(BaseNavigator):
    def __init__(self) -> None:
        super().__init__()
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

    def compute_heading_control(self, current_state: TurtleBotState, desired_state: TurtleBotState) -> TurtleBotControl:
        err= wrap_angle(desired_state.theta - current_state.theta)
        instructions=TurtleBotControl()
        instructions.omega=self.kp*err
        return instructions

    def compute_trajectory_tracking_control(self, state: TurtleBotState, plan: TrajectoryPlan, t: float) -> TurtleBotControl:
        x, y, th = state.x, state.y, state.theta

        dt = t - self.t_prev
        x_d = splev(t, plan.path_x_spline, der=0)
        xd_d = splev(t, plan.path_x_spline, der=1)
        xdd_d = splev(t, plan.path_x_spline, der=2)
        y_d = splev(t, plan.path_y_spline, der=0)
        yd_d = splev(t, plan.path_y_spline, der=1)
        ydd_d = splev(t, plan.path_y_spline, der=2)

        # avoid singularity
        if abs(self.V_prev) < self.V_PREV_THRES:
            self.V_prev = self.V_PREV_THRES

        xd = self.V_prev*np.cos(th)
        yd = self.V_prev*np.sin(th)

        # compute virtual controls
        u = np.array([xdd_d + self.kpx*(x_d-x) + self.kdx*(xd_d-xd),
                      ydd_d + self.kpy*(y_d-y) + self.kdy*(yd_d-yd)])

        # compute real controls
        J = np.array([[np.cos(th), -self.V_prev*np.sin(th)],
                          [np.sin(th), self.V_prev*np.cos(th)]])
        a, om = linalg.solve(J, u)
        V = self.V_prev + a*dt

        # save the commands that were applied and the time
        self.t_prev = t
        self.V_prev = V
        self.om_prev = om

        control = TurtleBotControl()
        control.v = float(V)
        control.omega = float(om)
        return control

    def compute_trajectory_plan(self, state: TurtleBotState, goal: TurtleBotState, occupancy: StochOccupancyGrid2D,
                                resolution: float, horizon: float) -> T.Optional[TrajectoryPlan]:
        # A* path planning
        astar = AStar((state.x - horizon, state.y - horizon), (state.x + horizon, state.y + horizon),
                      (state.x, state.y), (goal.x, goal.y), occupancy, resolution=resolution)
        if not astar.solve() or len(astar.path) < 4:
            return None

        # reset tracking controller history
        self.V_prev = 0.
        self.om_prev = 0.
        self.t_prev = 0.

        # path time computation and trajectory smoothing (from compute_smooth_plan)
        path = np.asarray(astar.path)
        ts = [0]
        for i in range(1, len(path)):
            distance=np.linalg.norm(path[i]-path[i-1])
            ts.append(ts[-1]+distance/self.v_desired)
        path_x_spline = scipy.interpolate.splrep(ts, path[:,0], s=self.spline_alpha)
        path_y_spline = scipy.interpolate.splrep(ts, path[:,1], s=self.spline_alpha)

        return TrajectoryPlan(
            path=path,
            path_x_spline=path_x_spline,
            path_y_spline=path_y_spline,
            duration=ts[-1],
        )

if __name__ == "__main__":
    rclpy.init()
    node = Navigator()
    rclpy.spin(node)
    rclpy.shutdown()
    