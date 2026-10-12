#!/usr/bin/env python3
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

# import the message type to use
from std_msgs.msg import Int64, Bool


class Heartbeat(Node):
    def __init__(self) -> None:
        # initialize base class (must happen before everything else)
        super().__init__("heartbeat")

        # a counter
        self.counter = 0

        # create publisher with: self.create_publisher(<msg type>, <topic>, <qos>)
        #self.hb_pub = self.create_publisher(Int64, "/heartbeat", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # create a timer with: self.create_timer(<second>, <callback>)
        self.hb_timer = self.create_timer(0.2, self.hb_callback)

        # create subscription with: self.create_subscription(<msg type>, <topic>, <callback>, <qos>)
        #self.motor_sub = self.create_subscription(Bool, "/health/motor", self.health_callback, 10)
        self.cmd_kill = self.create_subscription(Bool, "/kill", self.kill_switch, 10)


    def hb_callback(self) -> None:
        """
        Heartbeat callback triggered by the timer
        """
        # construct heartbeat message
        #msg = Int64()
        #msg.data = self.counter
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0

        # publish heartbeat counter
        #self.hb_pub.publish(msg)
        self.cmd_vel_pub.publish(msg)

        # increment counter
        #self.hb_counter += 1

        print(msg.linear.x, msg.angular.z)


    def kill_switch(self, msg: Bool) -> None:
        """
        Provides a kill switch triggered by subscription
        """
        if msg:
            msg1 = Twist()
            msg1.linear.x = 0.0
            msg1.angular.z = 0.0
            self.cmd_vel_pub.publish(msg1)
            print(msg1.linear.x, msg1.linear.x)
            self.hb_timer.cancel()





    def health_callback(self, msg: Bool) -> None:
        """
        Sensor health callback triggered by subscription
        """
        if not msg.data:
            self.get_logger().fatal("Heartbeat stopped")
            self.hb_timer.cancel()


if __name__ == "__main__":
    rclpy.init()        # initialize ROS2 context (must run before any other rclpy call)
    node = Heartbeat()  # instantiate the heartbeat node
    rclpy.spin(node)    # Use ROS2 built-in schedular for executing the node
    rclpy.shutdown()    # cleanly shutdown ROS2 context
