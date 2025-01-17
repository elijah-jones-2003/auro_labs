import sys

# ROS2 Libraries 
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.qos import QoSPresetProfiles

# Messages and services
from assessment_interfaces.msg import RobotList, ItemList, ZoneList
from auro_interfaces.msg import StringWithPose
from auro_interfaces.srv import ItemRequest
from solution_interfaces.msg import Task, TaskList
from solution_interfaces.srv import TaskComplete
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, PoseStamped

# Navigation
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry

# Misc
import random
import math
import angles
from enum import Enum
from tf_transformations import euler_from_quaternion

# Map Edges
MIN_X = -3.5
MAX_X = 2.5
MIN_Y = -2.5
MAX_Y = 2.5

# Task constants
PICK_UP = 0
DROP_OFF = 1

# Scan constants
SCAN_THRESHOLD = 0.3
SCAN_FRONT = 0
SCAN_LEFT = 1
SCAN_BACK = 2
SCAN_RIGHT = 3

# Robot controller states    
class State(Enum):
    IDLE = 0
    PICK_UP_ITEM = 1
    OFFLOAD_ITEM = 2
    NAVIGATING = 3
    REPORT_TASK_COMPLETE = 4
    OBSTACLE_AVOIDANCE = 5

class RobotControllerNode(Node):
    """
    A ROS2 Node for controlling a robot in a multi-robot environment, based on tasks delegated by the task manager 
    """
    def __init__(self):
        """
        Initializes the RobotControllerNode
        """
        super().__init__('robot_controller')
        # Get robot ID 
        self.robot_id = self.get_namespace().strip('/')
        self.get_logger().info("Robot ID:" + self.robot_id)
        # Create the initial pose
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'map'
        self.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Get initial pose
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.pose.pose.position.x = self.get_parameter('x').value
        self.pose.pose.position.y = self.get_parameter('y').value
        self.yaw = self.get_parameter('yaw').value

        self.state = State.IDLE
        self.current_task = None
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'

        # Initialise a nav2 BasicNavigator
        self.navigator = BasicNavigator()
        self.navigator.setInitialPose(self.pose)
        self.navigator.waitUntilNav2Active()

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Collision detection
        self.scan_triggered = [False] * 4
        self.robot_in_way = False

        # Services
        client_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()

        self.pick_up_service = self.create_client(ItemRequest, '/pick_up_item', callback_group=client_callback_group)
        self.offload_service = self.create_client(ItemRequest, '/offload_item', callback_group=client_callback_group)
        self.task_complete_service = self.create_client(TaskComplete, '/task_complete', callback_group=client_callback_group)

        # Publishers
        self.marker_publisher = self.create_publisher(StringWithPose, 'robot_marker', 10)
        self.task_complete_publisher = self.create_publisher(Task, 'task_complete', 10)   

        # Subscibers
        self.task_subscriber = self.create_subscription(TaskList, '/task_list', self.task_list_callback, 10, callback_group=timer_callback_group)
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10, callback_group=timer_callback_group)
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, QoSPresetProfiles.SENSOR_DATA.value, callback_group=timer_callback_group)
        self.robots_subscriber = self.create_subscription(RobotList, '/robots', self.robots_callback, 10, callback_group=timer_callback_group)
               
    def task_list_callback(self, msg):
        """
        Callback for the /task_list topic. Assigns tasks to the robot when available.

        Args:
            msg (TaskList): message containing the list of tasks produced by the task manager
        """
        # Get tasks from data
        tasks = msg.tasks
        if len(tasks) > 0:
            for task in tasks:
                # If a task has this robots id attached and the robot is idle, accept task
                if (task.robot_id == self.robot_id) and (self.state == State.IDLE):
                    self.get_logger().info(f"Task: {task.task_id} accepted")
                    self.current_task = task
                    break
        else:
            self.current_task = None

    def notify_task_complete(self, task_id):
        """
        Calls the TaskComplete service to inform the task manager that the current task is complete.

        Args:
            task_id (str): The id of the task to be removed
        """
        # Create request
        request = TaskComplete.Request()
        request.task_id = task_id
        request.robot_id = self.robot_id

        # Create a future object and spin until the service has been completed
        future = self.task_complete_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f"Successfully marked task {task_id} as complete.")
        else:
            self.get_logger().warn(f"Failed to mark task {task_id} as complete: {future.result().message}")
        return 

    def odom_callback(self, msg):

        self.pose.pose = msg.pose.pose

        # Generate yaw from the orientation
        (roll, pitch, yaw) = euler_from_quaternion([self.pose.pose.orientation.x,
                                                    self.pose.pose.orientation.y,
                                                    self.pose.pose.orientation.z,
                                                    self.pose.pose.orientation.w])
        self.yaw = yaw

    def scan_callback(self, msg):
        front_ranges = msg.ranges[331:359] + msg.ranges[0:30]
        left_ranges  = msg.ranges[31:90]
        back_ranges  = msg.ranges[91:270]
        right_ranges = msg.ranges[271:330]

        self.scan_triggered[SCAN_FRONT] = min(front_ranges) < SCAN_THRESHOLD 
        self.scan_triggered[SCAN_LEFT]  = min(left_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_BACK]  = min(back_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_RIGHT] = min(right_ranges) < SCAN_THRESHOLD

    def robots_callback(self, msg):
        for robot in msg.data:
            # Trial and error 0.4-0.5 is too close, potential for the robots to get stuck
            if robot.size >= 0.45:
                self.robot_in_way = True 
        else:
            self.robot_in_way = False

    def control_loop(self):
        # Publish marker for Rviz and task manager
        marker_input = StringWithPose()
        marker_input.text = str(self.state)
        marker_input.pose = self.pose.pose 
        self.marker_publisher.publish(marker_input)

        match self.state:
            # Base state, has its own navigation so that it is interruptable 
            case State.IDLE:
                # Check for valid task
                if self.current_task != None:
                    # If there is a valid task, add the destination to the goal pose and change to the navigating state
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
                        # Obstacle detection in case of a nav2 malfunction
                        if True in self.scan_triggered:
                            self.state = State.OBSTACLE_AVOIDANCE
                            return
                        feedback = self.navigator.getFeedback()
                        # Allow for interruption if a task is found
                        if self.current_task != None:
                            self.get_logger().info("Task recieved, canceling exploration")
                            self.navigator.cancelTask()
                        # Dont let the robot get stuck navigating if the goal isnt achievable 
                        if Duration.from_msg(feedback.navigation_time) > Duration(seconds = 30):
                            self.get_logger().info("Navigation took too long... cancelling")
                            self.navigator.cancelTask()
                
            # Once the robot has reached its destination, enter this state to pick up item
            case State.PICK_UP_ITEM:

                # Generate item request
                request = ItemRequest.Request()
                request.robot_id = self.robot_id
                try:
                    # Call pick up service and spin untill complete
                    future = self.pick_up_service.call_async(request)
                    self.executor.spin_until_future_complete(future)
                    response = future.result()
                    # If the item is successfully deposited report task as being complete so task manager can assign a new task
                    if response.success:
                        self.get_logger().info('Item picked up.')
                    else:
                        # If the service fails, report task as complete anyway to remove task from task list. This assumes that something is wrong with that task rather than the service
                        self.get_logger().info('Unable to pick up item: ' + response.message)
                    self.state = State.REPORT_TASK_COMPLETE
                except Exception as e:
                    self.get_logger().info('Exception ' + str(e))

            case State.OFFLOAD_ITEM:
                # Generate item request
                request = ItemRequest.Request()
                request.robot_id = self.robot_id
                try:
                    # Call offload service and spin untill its complete
                    future = self.offload_service.call_async(request)
                    self.executor.spin_until_future_complete(future)
                    response = future.result()
                    # If the item is offloaded successfully report task complete so task manager can assign new task
                    if response.success:
                        self.get_logger().info('Item dropped.')
                    else:
                        # If the service fails again still remove task as it may be a faulty message
                        self.get_logger().info('Unable to drop item: ' + response.message)
                    self.state = State.REPORT_TASK_COMPLETE
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
                    # Obstacle avoidance
                    if True in self.scan_triggered:
                        self.state = State.OBSTACLE_AVOIDANCE
                        self.current_task = None
                        return
                    
                    # Give feedback on estimated time of arrival
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
                                self.state = State.PICK_UP_ITEM
                            elif self.current_task.action == DROP_OFF:
                                self.state = State.OFFLOAD_ITEM
                            else:
                                raise ValueError("Invalid task action. 0 for pick up and 1 for offload.")
                        else:
                            self.state = State.IDLE

                    # Report any cancellations, failures or anything else
                    case TaskResult.CANCELED:
                        self.get_logger().info(f"Goal was canceled!")                       
                        self.state = State.IDLE

                    case TaskResult.FAILED:
                        self.get_logger().info(f"Goal failed!")
                        self.state = State.IDLE

                    case _:
                        self.get_logger().info(f"Goal has an invalid return status!")
                        self.state = State.IDLE

            case State.REPORT_TASK_COMPLETE:
                # Create task complete request
                request = TaskComplete.Request()
                request.robot_id = self.robot_id
                request.task_id = self.current_task.task_id
                try:
                    # Call task complete service and wait till complete
                    future = self.task_complete_service.call_async(request)
                    self.executor.spin_until_future_complete(future)
                    response = future.result()
                    if response.success:
                        # If the task is removed successfully get rid of curent task and go idle
                        self.current_task = None
                        self.state = State.IDLE
                    else:
                        # If task cannot be removed from task list, clear it from current_task and move on, but warn that it couldnt be destroyed
                        self.get_logger().warn(f"Failed to mark task {request.task_id} as complete: {future.result().message}")
                        self.current_task = None
                        self.state = State.IDLE
                except Exception as e:
                    self.get_logger().info('Exception ' + str(e))

            case State.OBSTACLE_AVOIDANCE:
                # Cancel current task, as it won't be marked as complete it can be picked up again later
                self.get_logger().warn("Obstacle detected. Current task canceled")
                self.navigator.cancelTask()
                # Reset Navigator
                self.navigator.setInitialPose(self.pose)
                if self.robot_in_way:
                    # Calculate a new goal behind the robot
                    backup_distance = 0.3  # Distance to back up
                    backup_pose = PoseStamped()
                    backup_pose.header.frame_id = 'map'
                    backup_pose.header.stamp = self.get_clock().now().to_msg()
                    backup_pose.pose.position.x = self.pose.pose.position.x - backup_distance * math.cos(self.yaw)
                    backup_pose.pose.position.y = self.pose.pose.position.y - backup_distance * math.sin(self.yaw)
                    backup_pose.pose.orientation = self.pose.pose.orientation

                    # Navigate to the backup position
                    self.get_logger().info("Backing up to avoid robot in the way.")
                    try:
                        self.navigator.goToPose(backup_pose)
                    except Exception as e:
                        self.get_logger().error(f"Failed to navigate to backup position: {e}")
                        self.state = State.IDLE  # Return to idle if navigation fails

                    while not self.navigator.isTaskComplete():
                        feedback = self.navigator.getFeedback()
                        if feedback:
                            self.get_logger().info("Backing up...")

                    # Back out of range of each other and wait a random amount of time for the other robot to move off
                    result = self.navigator.getResult()
                    if result == TaskResult.SUCCEEDED:
                        self.get_logger().info("Successfully backed up.")
                    else:
                        self.get_logger().warn("Failed to back up.")
                    rclpy.sleep(Duration(seconds=random.uniform(1, 5)))
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