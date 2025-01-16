import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from assessment_interfaces.msg import Item, ItemList, ItemHolder, ItemHolders, ItemLog, Zone, ZoneList 
from auro_interfaces.msg import StringWithPose
from solution_interfaces.msg import TaskList, Task
from solution_interfaces.srv import TaskComplete 
from collections import defaultdict



# Task Constants
PICK_UP = 0
DROP_OFF = 1

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
        self.items_subscriber = self.create_subscription(ItemList, '/items', self.items_callback, 10)

        # Data storage
        self.robots_state_dict= {}  # {robot_id: status}
        self.robots_pose_dict = {} # {robot_id: pose}
        self.items = []   # List of items with color and location
        self.zones = {}   # {assigned colour : zone_id}
        self.item_holders = [] 
        self.task_list = TaskList()
        self.task_ids = self.task_ids = defaultdict(int)  # Keep track of current task_id robot association, task id = robot_id + incremented value eg 4th task for robot1 = 13
        
        # Timer
        self.timer = self.create_timer(1, self.control_loop)
        # self.assign_task("robot1", Point(x = 1.0, y = 2.0), PICK_UP)
        # self.assign_task("robot1", Point(x = 2.5, y = 2.5), DROP_OFF)
        # self.assign_task("robot2", Point(x = -1.0, y = 0.0), PICK_UP)
        # self.assign_task("robot2", Point(x = -3.5, y = -2.5), DROP_OFF)
        # self.assign_task("robot3", Point(x = 1.0, y = 0.0), PICK_UP)
        # self.assign_task("robot3", Point(x = 2.5, y = -2.5), DROP_OFF)

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
        self.item_holders = msg

    def robot_marker_callback(self, msg):
        # topic_name = msg.text  
        # robot_id = topic_name.split('/')[1]  
        robot_id = msg.header.frame_id
        self.robots_pose_dict[robot_id] = msg.pose
        self.robots_state_dict[robot_id] = msg.text

    def items_callback(self, msg):
        self.items = msg.items 

    def assign_task(self, robot_id, point, action):
        task = Task()
        task.task_id = str(robot_id).lstrip('robot') + str(self.task_ids[robot_id])
        task.robot_id = robot_id                          
        task.destination = point
        task.action = action
        flag = False
        for t in self.task_list.tasks:
            if task.destination == t.destination:
                flag = True
                break
        if not flag:
            self.task_ids[robot_id] += 1
            self.get_logger().info(f"Task assigned to robot_id: {robot_id}, task id: {task.task_id}")
            self.task_list.tasks.append(task)

    # Control Loop
    def control_loop(self):
        
        # self.assign_task("robot2", Point(x = 2.5, y = -2.5), PICK_UP)
        # self.assign_task("robot3", Point(x = -3.5, y = -2.5), PICK_UP)
        self.task_list_publisher.publish(self.task_list)
    
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
