import sys

# ROS2 Libraries
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

# Messages
from geometry_msgs.msg import Point
from assessment_interfaces.msg import Item, ItemList, ItemHolder, ItemHolders, Zone, ZoneList 
from auro_interfaces.msg import StringWithPose
from solution_interfaces.msg import TaskList, Task
from solution_interfaces.srv import TaskComplete

# Misc
from collections import defaultdict
import yaml


# Task Constants
PICK_UP = 0
DROP_OFF = 1

# Zone Contants
ZONE_1 = Point(x = -3.5, y = 2.5)
ZONE_2 = Point(x = -3.5, y = -2.5)
ZONE_3 = Point(x = 2.5, y = -2.5)
ZONE_4 = Point(x = 2.5, y = 2.5)


# Cluster Constants
RED_CLUSTER = Point(x = -1.0, y = -2.0)
GREEN_CLUSTER = Point(x = 1.0, y = -2.0)
BLUE_CLUSTER = Point(x = 1.0, y = 2.0)

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')

        # Services
        self.task_complete_service = self.create_service(TaskComplete, '/task_complete', self.task_complete_callback)
        
        # Publishers
        self.task_list_publisher = self.create_publisher(TaskList, '/task_list', 10)
        
        # Subscribers
        self.item_holders_subscriber = self.create_subscription(ItemHolders, '/item_holders', self.item_holders_callback, 10)
        self.robot_marker_subscriber = self.create_subscription(StringWithPose, '/robot_marker', self.robot_marker_callback, 10)
        self.items_subscriber_robot1 = self.create_subscription(ItemList, 'robot1/items', self.items_callback, 10)
        self.items_subscriber_robot2 = self.create_subscription(ItemList, 'robot2/items', self.items_callback, 10)
        self.items_subscriber_robot3 = self.create_subscription(ItemList, 'robot3/items', self.items_callback, 10)

        # Data storage
        self.robots_state_dict= {}  # {robot_id: status}
        self.robots_pose_dict = {} # {robot_id: pose}
        self.items = []   # List of items with color and location
        self.zones = {}   # {assigned colour : Point(x,y) representing the zone}
        self.item_holders = [] 
        self.task_list = TaskList()
        self.task_ids = self.task_ids = defaultdict(int)  # Keep track of current task_id robot association, task id = robot_id + incremented value eg 4th task for robot1 = 13
        
        # Timer
        self.timer = self.create_timer(1, self.control_loop)


    # Callback Functions

    def task_complete_callback(self, request, response):
    # Extract task_id from the request
        task_id = request.task_id
        # Find and remove the task from the task list
        for task in self.task_list.tasks:
            if task.task_id == task_id:
                self.task_list.tasks.remove(task)
                self.get_logger().info(f"{request.robot_id} has completed task {task_id}.")
                response.success = True
                response.message = f"Task {task_id} has been marked as complete."
                return response

        # If task not found, return a failure response
        self.get_logger().warn(f"Task {task_id} not found in the list.")
        response.success = False
        response.message = f"Task {task_id} does not exist."
        return response

    def item_holders_callback(self, msg):
        self.item_holders = msg.data

    # Get the state and pose of all robots
    def robot_marker_callback(self, msg):  
        robot_id = msg.header.frame_id
        self.robots_pose_dict[robot_id] = msg.pose
        self.robots_state_dict[robot_id] = msg.text

    def items_callback(self, msg):
        # TODO collate each robots list of items making sure not to collect the same item twice (actually might not matter)
        self.items = msg.data 

    def assign_task(self, robot_id, point, action):
        task = Task()
        task.task_id = str(robot_id).lstrip('robot') + str(self.task_ids[robot_id])
        task.robot_id = robot_id                          
        task.destination = point
        task.action = action
        flag = False
        for t in self.task_list.tasks:
            # Only one task per robot at any time and make sure that 2 robots arent headed to the same item.
            if (task.destination == t.destination and task.action == t.action) or (task.robot_id == t.robot_id):
                flag = True
                break
        if not flag:
            self.task_ids[robot_id] += 1
            self.get_logger().info(f"Task assigned to robot_id: {robot_id}, task id: {task.task_id}")
            self.task_list.tasks.append(task)

    # Control Loop
    def control_loop(self):
        # Publish lits of tasks to be completed
        for task in self.task_list.tasks:
            robot_exists = False
            for robot in self.item_holders:
                if task.robot_id == robot.robot_id:
                    robot_exists = True
            if not robot_exists:
                self.task_list.remove(task)
                self.get_logger(f"{robot.robot_id} cannot be found, removing the task assigned to it")
        self.task_list_publisher.publish(self.task_list)

        # Generate tasks for the robots 
        for robot in self.item_holders:
            # TODO proper zone finding
            if robot.holding_item:
                match str(robot.item_colour):
                    case "RED":
                        self.assign_task(robot.robot_id, ZONE_2, DROP_OFF)
                    case "GREEN":
                        self.assign_task(robot.robot_id, ZONE_3, DROP_OFF)
                    case "BLUE":
                        self.assign_task(robot.robot_id, ZONE_4, DROP_OFF)
            else:
                match str(robot.robot_id):
                    case "robot1":
                        self.assign_task(robot.robot_id, BLUE_CLUSTER, PICK_UP)
                    case "robot2":
                        self.assign_task(robot.robot_id, RED_CLUSTER, PICK_UP)   
                    case "robot3":
                        self.assign_task(robot.robot_id, GREEN_CLUSTER, PICK_UP)
                
    
    def destroy_node(self):
        super().destroy_node()
    

def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()

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
