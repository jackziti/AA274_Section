#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from asl_tb3_lib.grids import StochOccupancyGrid2D
from asl_tb3_msgs.msg import TurtleBotState
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
import scipy.signal

def window_counts(mask, window_size):
    """
    Number of True cells of mask in the window_size x window_size window centered on every cell.
    Cells outside the map are not counted.
    Inputs:
        mask: (ny, nx) boolean array
        window_size: odd integer
    Output:
        (ny, nx) array of counts

    HINT: this is a convolution with a window of ones (scipy.signal.convolve2d with mode="same").
    """
    ########## Code starts here ##########
    window=np.ones((window_size,window_size))
    counts=scipy.signal.convolve2d(mask.astype(int),window, mode="same")
    ########## Code ends here ##########
    return counts


def explore(occupancy, window_size=11, min_unknown=0.25, min_free=0.35, occupied_thresh=50):
    """
    Frontier cells of an occupancy grid.
    A cell of occupancy.probs is unknown if its value is -1, occupied if it is >= occupied_thresh,
    and free otherwise. A cell is a frontier cell if, in the window_size x window_size window
    centered on it (cells outside the map are not counted, but the fractions are still of
    window_size^2),
        - at least min_unknown of the window_size^2 cells are unknown,
        - none of the cells are occupied, and
        - at least min_free of the window_size^2 cells are free.
    The cell itself may be free or unknown. Use the window_size argument, not occupancy.window_size.
    Output:
        frontier_states: (N, 2) array of (x, y) states of the frontier cells, in row-major order
        (increasing ky, then increasing kx, which is the order np.nonzero returns)

    HINT: work with grid indices and convert to states at the end with occupancy.grid2state.
          probs is indexed as probs[ky, kx], while grid2state expects (kx, ky).
    """
    probs = occupancy.probs
    ########## Code starts here ##########
    unkown_counts=window_counts(probs==-1,window_size)
    occupied_counts=window_counts(probs>=occupied_thresh, window_size)
    free_counts=window_counts((-1<probs) & (probs <occupied_thresh),window_size)
    frontier=(unkown_counts>=min_unknown*window_size**2) & (occupied_counts==0) & (free_counts>=min_free*window_size**2)
    ky,kx=np.nonzero(frontier) #get the index of non zero (here non False) values of frontier
    frontier_states=occupancy.grid2state(np.column_stack((kx,ky)))
    ########## Code ends here ##########
    return frontier_states


def closest_frontier(frontier_states, current_state):
    """
    Output:
        (state, distance): the frontier state closest to current_state in Euclidean distance,
        and that distance. If several states are equally close, return the first one.
        Returns (None, np.inf) if there are no frontier states.
    """
    ########## Code starts here ##########
    n=len(frontier_states)
    if n==0:
        return (None, np.inf)
    else:    
        distance=(current_state[0]-frontier_states[0][0])**2+(current_state[1]-frontier_states[0][1])**2
        index=0
        for i in range(1,len(frontier_states)):
            if (current_state[0]-frontier_states[i][0])**2+(current_state[1]-frontier_states[i][1])**2<distance:
                distance=(current_state[0]-frontier_states[i][0])**2+(current_state[1]-frontier_states[i][1])**2
                index=i
        return (frontier_states[index],np.sqrt(distance))
    ########## Code ends here ##########

class FrontierExplorer(Node):
    def __init__(self, node_name: str = "frontier") -> None:
        super().__init__(node_name)

        self.state = None
        self.goal = None
        self.occupancy = None
        self.failed_goals=[]
        self.declare_parameter("min_goal_distance", 0.4)

        self.timer=self.create_timer(1.0, self.timer_callback)
        self.state_sub = self.create_subscription(TurtleBotState, "/state", self.state_callback, 10)
        self.nav_succes_sub=self.create_subscription(Bool, "/nav_success", self.nav_success_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)

        self.nav_pub = self.create_publisher(TurtleBotState, "/cmd_nav", 10)

    def map_callback(self, msg: OccupancyGrid) -> None:
        """ Callback triggered when the map is updated
        Args:
        msg (OccupancyGrid): updated map message
        """
        self.occupancy = StochOccupancyGrid2D(
            resolution=msg.info.resolution,
            size_xy=np.array([msg.info.width, msg.info.height]),
            origin_xy=np.array([msg.info.origin.position.x, msg.info.origin.position.y]),
            window_size=9,
            probs=msg.data,
        )

    def timer_callback(self):
        if self.goal is None:
            self.send_next_goal()
    
    def state_callback(self, msg:TurtleBotState):
        self.state=msg #because only x, y and theta in msg as it's a TurtleBotState

    def nav_success_callback(self, msg: Bool):
        # Ignore results received before this explorer has sent its own goal.
        if self.goal is None:
            return
        if msg.data==False:
            self.failed_goals.append(np.array([self.goal.x, self.goal.y]))
        # Keep selection on the timer so new map updates can arrive first.
        self.goal = None

    def send_next_goal(self):
        if self.goal is not None or self.state is None or self.occupancy is None:
            return
        if self.nav_pub.get_subscription_count() == 0:
            return
        else:
            current_state=np.array([self.state.x,self.state.y])
            frontier_states=explore(self.occupancy)
            if len(frontier_states) == 0:
                self.get_logger().info("No frontiers yet; waiting for map updates")
                return

            # The homework heuristic allows unknown centers. Navigation goals
            # must instead be known-free and far enough away to form a path.
            indices = np.rint(
                (frontier_states - self.occupancy.origin_xy)
                / self.occupancy.resolution
            ).astype(int)
            values = self.occupancy.probs[indices[:, 1], indices[:, 0]]
            distances = np.linalg.norm(frontier_states - current_state, axis=1)
            min_distance = self.get_parameter("min_goal_distance").value
            keep = (distances >= min_distance) & (values >= 0) & (values < 50)
            frontier_states = frontier_states[keep]
            # remove frontier states within 0.5 m of any previously failed goal
            for failed_goal in self.failed_goals:
                distance_missed_point=np.linalg.norm(frontier_states-failed_goal, axis=1)
                frontier_states=frontier_states[distance_missed_point>=0.5]
            # Match the navigator's obstacle clearance check before publishing.
            frontier_states = np.asarray([
                point for point in frontier_states if self.occupancy.is_free(point)
            ]).reshape(-1, 2)
            goal_state, distance=closest_frontier(frontier_states,current_state)
            if goal_state is None:
                self.get_logger().info("No eligible frontier goals; waiting for map updates")
            else:
                self.goal=TurtleBotState(x=float(goal_state[0]), y=float(goal_state[1]), theta=self.state.theta)
                self.nav_pub.publish(self.goal)


def main():
    rclpy.init()
    node=FrontierExplorer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
