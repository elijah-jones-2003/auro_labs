#
# Copyright (c) 2024 University of York and others
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
# 
# SPDX-License-Identifier: EPL-2.0
#
# Contributors:
#   * Pedro Ribeiro - initial contributor
#
# Example of the implementation of an Finite State Machine closely following the
# RoboChart state machine 'TurtleBot3FSMRC' in the RoboChart project named
# 'turtlebot-robochart-square' as found in the folder 'robochart'. To view and
# edit the model you can import it in RoboTool.
#

import random
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles, QoSProfile, QoSHistoryPolicy

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
# from tf_transformations import euler_from_quaternion

from enum import Enum
import math

# Enumeration for keeping track of current state
class State(Enum):
    INITIAL = 0
    FORWARD = 1
    TURNING_LEFT = 2
    TURNING_RIGHT = 3

# Enumeration for keeping track of execution of a state's action
class Action(Enum):
    ENTRY = 0
    INTERRUPTABLE = 1
    EXIT = 2

# Constants
lvel = 0.3 # linear velocity
avel = 1.5 # angular velocity

CENTER = 0
LEFT = 1
RIGHT = 2

class TurtleBot3FSMRC(Node):
    def __init__(self):
        super().__init__('turtlebot3_fsm')

        # Initial node
        self.state = State.INITIAL
        self.action = Action.ENTRY
        self.scan_data_ = [0.0] * 3

        # Clock
        self.clock_C = 0

        qos = QoSProfile(history = QoSHistoryPolicy.KEEP_LAST,
                         depth = 10)

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value)

        # self.odom_sub_ = self.create_subscription(
        #     Odometry,
        #     '/odom',
        #     self.odom_callback,
        #     qos)
        
        # Output
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Periodic callback for executing FSM
        self.timer_period = 0.1 # 100 milliseconds = 10 Hz for the control loop
        self.time_unit = 1 # 1 second is the value chosen for one 'time unit'
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def increment_clocks(self):
        self.clock_C += 1

    def reset_clock_C(self):
        self.clock_C = 0

    def since_clock_C(self):
        return self.timer_period * self.clock_C / self.time_unit

    # def odom_callback(self, msg):
    #     q = [msg.pose.pose.orientation.x,
    #          msg.pose.pose.orientation.y,
    #          msg.pose.pose.orientation.z,
    #          msg.pose.pose.orientation.w]
        
    #     (roll, pitch, yaw) = euler_from_quaternion(q)
    #     self.robot_pose_ = yaw

        self.get_logger().info(f"robot_pose_: {self.robot_pose_}")


    def scan_callback(self, msg):
        scan_angle = [0, 30, 330]

        for num in range(0, 3):
            if math.isinf(msg.ranges[scan_angle[num]]):
                self.scan_data_[num] = msg.range_max
            else:
                self.scan_data_[num] = msg.ranges[scan_angle[num]]

        self.get_logger().info(f"scan_data_: {self.scan_data_}")
    
    def cmd_vel(self, twist):
        self.publisher.publish(twist)
        self.get_logger().info(f"Output on cmd_vel: {twist}")

    def control_loop(self):

        self.increment_clocks()
        escape_range = math.radians(30)
        check_forward_dist = 0.7
        check_side_dist = 0.6

        match self.state:
            # Initial Junction
            case State.INITIAL:
                self.reset_clock_C()
                self.state = State.FORWARD

            # State Forward
            case State.FORWARD:
                match self.action:
                    case Action.ENTRY:
                        # Entry action
                        self.get_logger().info("Entering state: 'FORWARD'")
                        self.cmd_vel(Twist(linear=Vector3(x=lvel, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)))

                        self.action = Action.INTERRUPTABLE
                    case Action.INTERRUPTABLE:
                        if self.scan_data_[CENTER] > check_forward_dist:
                            if self.scan_data_[LEFT] < check_side_dist:
                                self.state = State.TURNING_RIGHT
                            elif self.scan_data_[RIGHT] < check_side_dist:
                                self.state = State.TURNING_LEFT
                        if self.scan_data_[CENTER] < check_forward_dist:
                            if (random.randint(0,1) == 0):
                                self.state = State.TURNING_LEFT
                            else:
                                self.state = State.TURNING_RIGHT
                        # Check transition's guard from Forward -> Turning.
                        if self.since_clock_C() > 10:
                            # No exit action present
                            self.get_logger().info("Transition from 'FORWARD' to 'TURNING")
                            # Transition's action
                            self.reset_clock_C()
                            self.action = Action.ENTRY
                            if (random.randint(0,1) == 0):
                                self.state = State.TURNING_LEFT
                            else:
                                self.state = State.TURNING_RIGHT
                    case _:
                        pass

            # State Turning
            case State.TURNING_LEFT:
                match self.action:
                    case Action.ENTRY:
                        # Entry action
                        self.get_logger().info("Entering state: 'TURNING LEFT'")
                        self.cmd_vel(Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-avel)))

                        self.action = Action.INTERRUPTABLE
                    case Action.INTERRUPTABLE:
                        # Check transition's guard from Turning -> Forward.
                        if self.since_clock_C() > random.random():
                            # No exit action present
                            self.get_logger().info("Transition from 'TURNING LEFT' to 'FORWARD")
                            # Transition's action
                            self.reset_clock_C()
                            self.action = Action.ENTRY
                            self.state = State.FORWARD
                    case _:
                        pass

            case State.TURNING_RIGHT:
                match self.action:
                    case Action.ENTRY:
                        # Entry action
                        self.get_logger().info("Entering state: 'TURNING RIGHT'")
                        self.cmd_vel(Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=avel)))

                        self.action = Action.INTERRUPTABLE
                    case Action.INTERRUPTABLE:
                        # Check transition's guard from Turning -> Forward.
                        if self.since_clock_C() > random.random():
                            # No exit action present
                            self.get_logger().info("Transition from 'TURNING RIGHT' to 'FORWARD")
                            # Transition's action
                            self.reset_clock_C()
                            self.action = Action.ENTRY
                            self.state = State.FORWARD
                    case _:
                        pass
            case _:
                pass

def main(args=None):
    rclpy.init(args=args)

    turtlebot3_fsm_rc = TurtleBot3FSMRC()

    rclpy.spin(turtlebot3_fsm_rc)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtlebot3_fsm_rc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()