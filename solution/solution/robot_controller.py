import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from assessment_interfaces.msg import RobotList, ItemList, ZoneList 

from auro_interfaces.msg import StringWithPose
from auro_interfaces.srv import ItemRequest

import math
import random
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String

import angles
from enum import Enum
from tf_transformations import euler_from_quaternion
from auro_interfaces.msg import StringWithPose


LINEAR_VELOCITY  = 0.3 # Metres per second
ANGULAR_VELOCITY = 0.5 # Radians per second

TURN_LEFT = 1 # Postive angular velocity turns left
TURN_RIGHT = -1 # Negative angular velocity turns right

SCAN_THRESHOLD = 0.5 # Metres per second
 # Array indexes for sensor sectors
SCAN_FRONT = 0
SCAN_LEFT = 1
SCAN_BACK = 2
SCAN_RIGHT = 3

# Finite state machine (FSM) states
class State(Enum):
    FORWARD = 0
    TURNING = 1
    COLLECTING = 2
    DEPOSITING = 3

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.state = State.FORWARD
        self.pose = Pose()

        self.previous_pose = Pose() # Store a snapshot of the pose for comparison against future poses
        self.previous_yaw = 0.0 # Snapshot of the angle for comparison against future angles
        self.turn_angle = 0.0 # Relative angle to turn to in the TURNING state
        self.turn_direction = TURN_LEFT # Direction to turn in the TURNING state
        self.goal_distance = random.uniform(1.0, 2.0) # Goal distance to travel in FORWARD state
        self.scan_triggered = [False] * 4 # Boolean value for each of the 4 LiDAR sensor sectors. True if obstacle detected within SCAN_THRESHOLD
        self.items = ItemList()
        self.holding_Item = False

        self.declare_parameter('robot_id', 'robot1')
        self.robot_id = self.get_parameter('robot_id').value

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Services
        client_callback_group = MutuallyExclusiveCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()

        self.pick_up_service = self.create_client(ItemRequest, '/pick_up_item', callback_group=client_callback_group)
        self.offload_service = self.create_client(ItemRequest, '/offload_item', callback_group=client_callback_group)

        # Publishers
        self.marker_publisher = self.create_publisher(StringWithPose, 'robot_marker', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        self.robot_subscriber = self.create_subscription(
            RobotList,
            'robots',
            self.robot_callback,
            10)
        
        self.item_subscriber = self.create_subscription(
            ItemList, 
            'items', 
            self.item_callback,
            10)
        
        self.zone_list_subscriber = self.create_subscription(
            ZoneList, 
            'zone', 
            self.zone_callback, 
            10)

    def item_callback(self, msg):
        self.items = msg

    def robot_callback(self, msg):
        self.robots = msg

    def zone_callback(self, msg):
        self.zones = msg   

    def odom_callback(self, msg):
        self.pose = msg.pose.pose 

        (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x,
                                                    self.pose.orientation.y,
                                                    self.pose.orientation.z,
                                                    self.pose.orientation.w])
        
        self.yaw = yaw 
    
    def scan_callback(self, msg):
        self.scan = msg
        # Group scan ranges into 4 segments
        # Front, left, and right segments are each 60 degrees
        # Back segment is 180 degrees
        front_ranges = msg.ranges[331:359] + msg.ranges[0:30] # 30 to 331 degrees (30 to -30 degrees)
        left_ranges  = msg.ranges[31:90] # 31 to 90 degrees (31 to 90 degrees)
        back_ranges  = msg.ranges[91:270] # 91 to 270 degrees (91 to -90 degrees)
        right_ranges = msg.ranges[271:330] # 271 to 330 degrees (-30 to -91 degrees)

        # Store True/False values for each sensor segment, based on whether the nearest detected obstacle is closer than SCAN_THRESHOLD
        self.scan_triggered[SCAN_FRONT] = min(front_ranges) < SCAN_THRESHOLD 
        self.scan_triggered[SCAN_LEFT]  = min(left_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_BACK]  = min(back_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_RIGHT] = min(right_ranges) < SCAN_THRESHOLD
        

    def control_loop(self):
        self.get_logger().info(f"Initial pose - x: {self.initial_x}, y: {self.initial_y}, yaw: {self.initial_yaw}")
        # Send message to rviz_text_marker node
        marker_input = StringWithPose()
        marker_input.text = str(self.state)  # Visualise robot state as an RViz marker
        marker_input.pose = self.pose  # Set the pose of the RViz marker to track the robot's pose
        self.marker_publisher.publish(marker_input)

        match self.state:
            case State.FORWARD:
                if self.scan_triggered[SCAN_FRONT]:
                    self.previous_yaw = self.yaw
                    self.state = State.TURNING
                    self.turn_angle = random.uniform(150, 170)
                    self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
                    self.get_logger().info("Detected obstacle in front, turning " + ("left" if self.turn_direction == TURN_LEFT else "right") + f" by {self.turn_angle:.2f} degrees")
                    return

                if self.scan_triggered[SCAN_LEFT] or self.scan_triggered[SCAN_RIGHT]:
                    self.previous_yaw = self.yaw
                    self.state = State.TURNING
                    self.turn_angle = 45

                    if self.scan_triggered[SCAN_LEFT] and self.scan_triggered[SCAN_RIGHT]:
                        self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
                        self.get_logger().info("Detected obstacle to both the left and right, turning " + ("left" if self.turn_direction == TURN_LEFT else "right") + f" by {self.turn_angle:.2f} degrees")
                    elif self.scan_triggered[SCAN_LEFT]:
                        self.turn_direction = TURN_RIGHT
                        self.get_logger().info(f"Detected obstacle to the left, turning right by {self.turn_angle} degrees")
                    else:  # self.scan_triggered[SCAN_RIGHT]
                        self.turn_direction = TURN_LEFT
                        self.get_logger().info(f"Detected obstacle to the right, turning left by {self.turn_angle} degrees")
                    return

                if not self.holding_Item:
                    if len(self.items.data) > 0:
                        self.state = State.COLLECTING
                        return
                else:
                    if len(self.zones.data) > 0:
                        self.state = State.DEPOSITING
                        return

                msg = Twist()
                msg.linear.x = LINEAR_VELOCITY
                self.cmd_vel_publisher.publish(msg)

                difference_x = self.pose.position.x - self.previous_pose.position.x
                difference_y = self.pose.position.y - self.previous_pose.position.y
                distance_travelled = math.sqrt(difference_x ** 2 + difference_y ** 2)

                if distance_travelled >= self.goal_distance:
                    self.previous_yaw = self.yaw
                    self.state = State.TURNING
                    self.turn_angle = random.uniform(30, 150)
                    self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
                    self.get_logger().info("Goal reached, turning " + ("left" if self.turn_direction == TURN_LEFT else "right") + f" by {self.turn_angle:.2f} degrees")

            case State.TURNING:
                self.get_logger().info("Turning state")

                if not self.holding_Item:
                    if len(self.items.data) > 0:
                        self.state = State.COLLECTING
                        return
                else:
                    if len(self.zones.data) > 0:
                        self.state = State.DEPOSITING
                        return

                msg = Twist()
                msg.angular.z = self.turn_direction * ANGULAR_VELOCITY
                self.cmd_vel_publisher.publish(msg)

                yaw_difference = angles.normalize_angle(self.yaw - self.previous_yaw)

                if math.fabs(yaw_difference) >= math.radians(self.turn_angle):
                    self.previous_pose = self.pose
                    self.goal_distance = random.uniform(1.0, 2.0)
                    self.state = State.FORWARD
                    self.get_logger().info(f"Finished turning, driving forward by {self.goal_distance:.2f} metres")

            case State.COLLECTING:
                if len(self.items.data) == 0:
                    self.previous_pose = self.pose
                    self.state = State.FORWARD
                    return

                item = self.items.data[0]
                # closest_item = item
                # for item in self.items.data:
                #     if item.diameter < closest_item.diameter:
                #         closest_item = item

                estimated_distance = 32.4 * float(item.diameter) ** -0.75

                self.get_logger().info(f'Estimated distance {estimated_distance}')

                if estimated_distance <= 0.35:
                    rqt = ItemRequest.Request()
                    rqt.robot_id = self.robot_id
                    try:
                        future = self.pick_up_service.call_async(rqt)
                        self.executor.spin_until_future_complete(future)
                        response = future.result()
                        if response.success:
                            self.get_logger().info('Item picked up.')
                            self.holding_Item = True
                            self.state = State.DEPOSITING
                            self.items.data = []
                        else:
                            self.get_logger().info('Unable to pick up item: ' + response.message)
                    except Exception as e:
                        self.get_logger().info('Exception ' + str(e))

                msg = Twist()
                msg.linear.x = 0.25 * estimated_distance
                msg.angular.z = item.x / 320.0
                self.cmd_vel_publisher.publish(msg)

            case State.DEPOSITING:
                if len(self.zones.data) == 0:
                    self.previous_pose = self.pose
                    self.state = State.TURNING
                    return

                zone = self.items.data[0]

                estimated_distance = 32.4 * float(zone.diameter) ** -0.75

                self.get_logger().info(f'Estimated distance {estimated_distance}')

                if estimated_distance <= 0:
                    rqt = ItemRequest.Request()
                    rqt.robot_id = self.robot_id
                    try:
                        future = self.offload_service.call_async(rqt)
                        self.executor.spin_until_future_complete(future)
                        response = future.result()
                        if response.success:
                            self.get_logger().info('Item dropped in zone.')
                            self.state = State.TURNING
                            self.zones.data = []
                        else:
                            self.get_logger().info('Unable to deposit item: ' + response.message)
                    except Exception as e:
                        self.get_logger().info('Exception ' + str(e))

                msg = Twist()
                msg.linear.x = 0.25 * estimated_distance
                msg.angular.z = item.x / 320.0
                self.cmd_vel_publisher.publish(msg)

            case _:
                pass

    def destroy_node(self):
        super().destroy_node()

def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = RobotController()

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