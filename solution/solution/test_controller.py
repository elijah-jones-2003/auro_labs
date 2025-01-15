import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from assessment_interfaces.msg import RobotList, ItemList, ZoneList, Task, TaskList 

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration

from auro_interfaces.msg import StringWithPose
from auro_interfaces.srv import ItemRequest

import math
import random
from geometry_msgs.msg import Twist, Point, PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry

import angles
from enum import Enum
from tf_transformations import euler_from_quaternion

# Zone locations
ZONE_1 = Point(x = -3.5, y = 2.5)
ZONE_2 = Point(x = -3.5, y = -2.5)
ZONE_3 = Point(x = 2.5, y = -2.5)
ZONE_4 = Point(x = 2.5, y = 2.5)

# Map Edges
MIN_X = -3.5
MAX_X = 2.5
MIN_Y = -2.5
MAX_Y = 2.5

# Task constants
PICK_UP = 0
DROP_OFF = 1
    
class State(Enum):
    IDLE = 0
    COLLECT_ITEM = 1
    DEPOSIT_ITEM = 2
    NAVIGATING = 3


class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller')
        # Get robot ID 
        self.robot_id = self.get_namespace().strip('/')
        self.get_logger().info("Robot ID:" + self.robot_id)
        # Create the initial pose
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'map'
        self.pose.header.stamp = self.get_clock().now().to_msg()
        # TODO: Set initial pose dynamically
        self.pose.pose.position.x = -3.5
        self.pose.pose.position.y = 0.0
        self.pose.pose.orientation.z = 0.0 

        self.state = State.IDLE
        self.current_task = None
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'

        self.navigator = BasicNavigator()
        
        self.navigator.setInitialPose(self.pose)
        self.navigator.waitUntilNav2Active()

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Services
        client_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()

        self.pick_up_service = self.create_client(ItemRequest, '/pick_up_item', callback_group=client_callback_group)
        self.offload_service = self.create_client(ItemRequest, '/offload_item', callback_group=client_callback_group)

        # Publishers
        self.marker_publisher = self.create_publisher(StringWithPose, 'robot_marker', 10)
        self.task_complete_publisher = self.create_publisher(Task, 'task_complete', 10)

        # Subscibers
        self.task_subscriber = self.create_subscription(TaskList, '/task_list', self.task_list_callback, 10, callback_group=timer_callback_group)
        # self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10, callback_group=timer_callback_group)
        # self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10, callback_group=timer_callback_group)
        
        
    def task_list_callback(self, msg):
        tasks = msg.data
        self.current_task = tasks[0]
        if len(msg.data) > 0:
            for task in tasks:
                if task.robot_id == self.robot_id:
                    print("I got a task")
                    self.current_task = task
                    break
        else:
            self.current_task = None

    def control_loop(self):

        marker_input = StringWithPose()
        marker_input.text = str(self.state)
        marker_input.pose = self.pose.pose 
        self.marker_publisher.publish(marker_input)

        match self.state:
            case State.IDLE:
                # Check for valid task
                if self.current_task != None:
                    self.goal_pose.pose.position = self.current_task.destination 
                    self.state = State.NAVIGATING                 
                else:
                    self.get_logger().info("No task assigned. Exploring randomly.")
                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    # Create a random goal pose within the boundaries of the map
                    self.goal_pose.pose.position.x = random.uniform(MIN_X, MAX_X)
                    self.goal_pose.pose.position.y = random.uniform(MIN_Y, MAX_Y)
                    self.get_logger().info(f"Random goal: {self.goal_pose.pose.position.x}, {self.goal_pose.pose.position.y}")

                    # Navigate to the goal position
                    try:
                        self.navigator.goToPose(self.goal_pose)
                    except Exception as e:
                        self.get_logger().error(f"Failed to navigate to goal: {e}")

                    while not self.navigator.isTaskComplete():
                        feedback = self.navigator.getFeedback()
                        if self.current_task != None:
                            self.get_logger().info("Task recieved, canceling exploration")
                            self.navigator.cancelTask()
                        if Duration.from_msg(feedback.navigation_time) > Duration(seconds = 30):
                            self.get_logger().info("Navigation took too long... cancelling")
                            self.navigator.cancelTask()
                
            case State.COLLECT_ITEM:
                # Pick up item
                rqt = ItemRequest.Request()
                rqt.robot_id = self.robot_id
                try:
                    future = self.pick_up_service.call_async(rqt)
                    self.executor.spin_until_future_complete(future)
                    response = future.result()
                    if response.success:
                        self.get_logger().info('Item picked up.')
                        self.task_complete_publisher.publish(self.current_task)
                        self.current_task = None
                        self.state = State.IDLE
                    else:
                        self.get_logger().info('Unable to pick up item: ' + response.message)
                        self.state = State.IDLE
                except Exception as e:
                    self.get_logger().info('Exception ' + str(e))

            case State.DEPOSIT_ITEM:
                # Drop item
                rqt = ItemRequest.Request()
                rqt.robot_id = self.robot_id
                try:
                    future = self.offload_service.call_async(rqt)
                    self.executor.spin_until_future_complete(future)
                    response = future.result()
                    if response.success:
                        self.get_logger().info('Item dropped.')
                        self.task_complete_publisher.publish(self.current_task)
                        self.current_task = None
                        self.state = State.IDLE
                    else:
                        self.get_logger().info('Unable to drop item: ' + response.message)
                        self.state = State.IDLE
                except Exception as e:
                    self.get_logger().info('Exception ' + str(e))

            case State.NAVIGATING:
                # Navigate to the goal position
                try:
                    self.navigator.goToPose(self.goal_pose)
                except Exception as e:
                    self.get_logger().error(f"Failed to navigate to goal: {e}")
                    self.state = State.IDLE  # Return to idle if navigation fails

                # Wait for the navigator to complete the task
                while not self.navigator.isTaskComplete():
                    feedback = self.navigator.getFeedback()
                    if feedback:
                        eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                        self.get_logger().info(f"Estimated time of arrival: {eta:.0f} seconds.")
                
                # Once the navigator has finished, complete the task or catch failures
                result = self.navigator.getResult()
                match result:
                    case TaskResult.SUCCEEDED:
                        self.get_logger().info(f"Arrived at destination")
                        if not self.current_task == None: 
                            if self.current_task.action == PICK_UP:
                                self.state = State.COLLECT_ITEM
                            elif self.current_task.action == DROP_OFF:
                                self.state = State.DEPOSIT_ITEM
                            else:
                                raise ValueError("Invalid task action. 0 for collecting and 1 for depositing")
                        else:
                            self.state = State.IDLE

                    case TaskResult.CANCELED:
                        self.get_logger().info(f"Goal was canceled!")                       
                        self.state = State.IDLE

                    case TaskResult.FAILED:
                        self.get_logger().info(f"Goal failed!")
                        self.state = State.IDLE

                    case _:
                        self.get_logger().info(f"Goal has an invalid return status!")
                        self.state = State.IDLE



    def destroy_node(self):
        super().destroy_node()

def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = RobotControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()