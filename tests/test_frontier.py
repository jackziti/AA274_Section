"""Offline regression checks; ROS messaging is replaced by explicit test doubles.

Run: python -m unittest discover -s tests -v
These checks do not replace a ROS build or simulation.
"""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

import numpy as np


class State:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x, self.y, self.theta = x, y, theta


class Publisher:
    def __init__(self):
        self.messages = []
        self.subscribers = 1

    def publish(self, msg):
        self.messages.append(msg)

    def get_subscription_count(self):
        return self.subscribers


class Timer:
    def __init__(self):
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class FakeNode:
    def __init__(self, name):
        self.parameters = {}

    def declare_parameter(self, name, value):
        self.parameters[name] = value

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, *args):
        return Publisher()

    def create_subscription(self, *args):
        return args

    def create_timer(self, *args):
        return Timer()

    def get_logger(self):
        return types.SimpleNamespace(info=lambda text: None)


class Grid:
    def __init__(self):
        self.resolution = 0.1
        self.origin_xy = np.array([-1.0, -1.0])
        self.probs = np.zeros((50, 50))

    def grid2state(self, xy):
        return xy * self.resolution + self.origin_xy

    def is_free(self, point):
        # Individual clearance decisions are controlled explicitly in the test.
        return True


def load_frontier():
    names = ['rclpy', 'rclpy.node', 'nav_msgs', 'nav_msgs.msg',
             'std_msgs', 'std_msgs.msg', 'asl_tb3_msgs', 'asl_tb3_msgs.msg',
             'asl_tb3_lib', 'asl_tb3_lib.grids']
    stubs = {name: types.ModuleType(name) for name in names}
    stubs['rclpy.node'].Node = FakeNode
    stubs['nav_msgs.msg'].OccupancyGrid = object
    stubs['std_msgs.msg'].Bool = object
    stubs['asl_tb3_msgs.msg'].TurtleBotState = State
    stubs['asl_tb3_lib.grids'].StochOccupancyGrid2D = Grid
    path = Path(__file__).resolve().parents[1] / 'autonomy_repo/scripts/frontier.py'
    spec = importlib.util.spec_from_file_location('frontier_under_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, stubs):
        spec.loader.exec_module(module)
    return module


frontier = load_frontier()


class FrontierTests(unittest.TestCase):
    def node(self):
        node = frontier.FrontierExplorer()
        node.state = State()
        node.occupancy = Grid()
        return node

    def result(self, success):
        return types.SimpleNamespace(data=success)

    def test_feedback_before_own_goal_is_ignored(self):
        node = self.node()
        node.nav_success_callback(self.result(False))
        node.nav_success_callback(self.result(True))
        self.assertIsNone(node.goal)
        self.assertEqual(node.failed_goals, [])
        self.assertEqual(node.nav_pub.messages, [])

    def test_one_goal_at_a_time_without_cancelling_timer(self):
        node = self.node()
        with patch.object(frontier, 'explore', return_value=np.array([[1., 0.]])):
            node.timer_callback()
            node.timer_callback()
            node.send_next_goal()
        self.assertEqual(len(node.nav_pub.messages), 1)
        self.assertFalse(node.timer.cancelled)

    def test_success_clears_goal_and_next_tick_selects_another(self):
        node = self.node()
        with patch.object(frontier, 'explore', return_value=np.array([[1., 0.], [2., 0.]])):
            node.timer_callback()
            node.state = State(x=1.)
            node.nav_success_callback(self.result(True))
            self.assertIsNone(node.goal)
            self.assertEqual(len(node.nav_pub.messages), 1)
            node.timer_callback()
        self.assertEqual(node.nav_pub.messages[-1].x, 2.)

    def test_map_can_restart_exploration_after_no_next_frontier(self):
        node = self.node()
        with patch.object(frontier, 'explore', return_value=np.array([[1., 0.]])):
            node.timer_callback()
            node.timer_callback()
        node.nav_success_callback(self.result(True))
        with patch.object(frontier, 'explore', return_value=np.empty((0, 2))):
            node.timer_callback()
        self.assertIsNone(node.goal)
        self.assertFalse(node.timer.cancelled)
        with patch.object(frontier, 'explore', return_value=np.array([[2., 0.]])):
            node.timer_callback()
        self.assertEqual(node.nav_pub.messages[-1].x, 2.)

    def test_failure_skips_neighborhood_on_next_tick(self):
        node = self.node()
        candidates = np.array([[1., 0.], [1.1, 0.], [2., 0.]])
        with patch.object(frontier, 'explore', return_value=candidates):
            node.timer_callback()
            node.nav_success_callback(self.result(False))
            node.timer_callback()
        np.testing.assert_allclose(node.failed_goals, [[1., 0.]])
        self.assertEqual(node.nav_pub.messages[-1].x, 2.)

    def test_ignores_current_position_and_short_path_goals(self):
        node = self.node()
        candidates = np.array([[0., 0.], [.2, 0.], [1., 0.]])
        with patch.object(frontier, 'explore', return_value=candidates):
            node.timer_callback()
        self.assertEqual(node.goal.x, 1.)

    def test_goal_distance_parameter_is_used(self):
        node = self.node()
        node.parameters['min_goal_distance'] = 1.5
        with patch.object(frontier, 'explore', return_value=np.array([[1., 0.], [2., 0.]])):
            node.timer_callback()
        self.assertEqual(node.goal.x, 2.)

    def test_unknown_occupied_and_clearance_failures_are_skipped(self):
        node = self.node()
        node.occupancy.probs[10, 15] = -1   # world (0.5, 0)
        node.occupancy.probs[10, 20] = 100  # world (1.0, 0)
        node.occupancy.is_free = lambda point: point[0] != 1.5
        candidates = np.array([[.5, 0.], [1., 0.], [1.5, 0.], [2., 0.]])
        with patch.object(frontier, 'explore', return_value=candidates):
            node.timer_callback()
        self.assertEqual(node.goal.x, 2.)

    def test_waits_for_inputs_and_navigator_subscription(self):
        node = frontier.FrontierExplorer()
        with patch.object(frontier, 'explore') as explore:
            node.timer_callback()
            node.state, node.occupancy = State(), Grid()
            node.nav_pub.subscribers = 0
            node.timer_callback()
            explore.assert_not_called()

    def test_nearest_frontier_ties_and_empty_input(self):
        point, distance = frontier.closest_frontier(np.array([[1, 0], [-1, 0]]), [0, 0])
        np.testing.assert_array_equal(point, [1, 0])
        self.assertEqual(distance, 1.)
        self.assertEqual(frontier.closest_frontier(np.empty((0, 2)), [0, 0]),
                         (None, np.inf))

    def test_frontier_counts_and_coordinate_conversion(self):
        grid = Grid()
        grid.probs = np.zeros((7, 7))
        grid.probs[:, 4:] = -1
        actual = frontier.explore(grid, window_size=3)
        expected = []
        for ky in range(7):
            for kx in range(7):
                window = grid.probs[max(0, ky-1):ky+2, max(0, kx-1):kx+2]
                if (np.sum(window == -1) >= .25 * 9
                        and not np.any(window >= 50)
                        and np.sum((window >= 0) & (window < 50)) >= .35 * 9):
                    expected.append(grid.grid2state(np.array([kx, ky])))
        np.testing.assert_allclose(actual, expected)


if __name__ == '__main__':
    unittest.main()
