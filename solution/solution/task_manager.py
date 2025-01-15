import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from assessment_interfaces.msg import Item, ItemList, ItemHolder, ItemHolders, ItemLog, Zone, ZoneList, Task, TaskList 
# from assessment_interfaces.msg import RobotList
from auro_interfaces.msg import StringWithPose

# Task Constants
PICK_UP = 0
DROP_OFF = 1

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        
        # Publishers
        self.task_publisher = self.create_publisher(Task, '/task', 10)
        
        # Subscribers
        self.item_holders_subscriber = self.create_subscription(ItemHolders, '/item_holders', self.item_holders_callback, 10)
        self.robot_marker_subscriber = self.create_subscription(StringWithPose, '/robot_marker', self.robot_marker_callback, 10)
        self.items_subscriber = self.create_subscription(ItemList, '/items', self.items_callback, 10)

        # Data storage
        self.robots_state_dict= {}  # {robot_id: status}
        self.robots_pose_dict = {} # {robot_id: pose}
        self.items = []   # List of items with color and location
        self.zones = {}   # {assigned colour : zone_id}
        self.item_holders = []  

        # Timer
        self.timer = self.create_timer(1, self.control_loop)

    # Callback Functions
    def item_holders_callback(self, msg):
        self.item_holders = msg

    def robot_marker_callback(self, msg):
        # topic_name = msg.text  
        # robot_id = topic_name.split('/')[1]  
        robot_id = msg.header.frame_id
        self.robots_pose_dict[robot_id] = msg.pose
        self.robots_state_dict[robot_id] = msg.text

    def items_callback(self, msg):
        self.items = msg.items 

    def assign_task(self, robot_id):
        task = Task()

    # Control Loop
    def control_loop(self):
        self.get_logger().info("Task time")
        task = Task()
        task.robot_id = 'robot1' #robot.robot_id
        task.destination = Point(x = 2.5, y = 2.5)
        task.action = PICK_UP
        self.task_publisher.publish(task)
        # if task:
        #     self.task_publisher.publish(task)
        #     self.get_logger().info(f"Task assigned to robot {robot.robot_id}: {task}")
        # # Prioritise depositing items in currently being held by robots
        # for robot in self.item_holders:
        #     if robot.holding_item:
        #         if self.robots_state[robot.robot_id] ==  "State.IDLE":
        #             task = Task()
        #             task.robot_id = robot.robot_id
        #             if robot.item_colour not in self.zones:
        #                 # If the item colour doesn't exist, assign it to a zone
        #                 self.zones[robot.item_colour] = robot.item_colour
        #             # Now assign the destination based on the item colour
        #             task.destination = self.zones[robot.item_colour]
        #             task.action = DROP_OFF
        #             self.task_publisher.publish(task)
        #     else:
        #         # If the robot is idle, assign a task
        #         if self.robots_state[robot.robot_id] ==  "State.IDLE":
        #             task = Task()
        #             task.robot_id = 'robot1' #robot.robot_id
        #             task.destination = Point(x = 2.5, y = 2.5)
        #             task.action = PICK_UP
        #             self.task_publisher.publish(task)
        #             if task:
        #                 self.task_publisher.publish(task)
        #                 self.get_logger().info(f"Task assigned to robot {robot.robot_id}: {task}")
            

    
    def destroy_node(self):
        super().destroy_node()
    

def main(args=None):
    rclpy.init(args=args)
    task_manager = TaskManager()
    rclpy.spin(task_manager)
    task_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
