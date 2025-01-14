import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from assessment_interfaces.msg import ItemHolder, ItemHolders, ItemLog, Task
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

        # Data storage
        self.robots_state_dict= {}  # {robot_id: status}
        self.robots_pose_dict = {} # {robot_id: pose}
        self.items = []   # List of items with color and location
        self.zones = {}   # {assigned colour : zone_id}
        self.item_holders = []


    # Callback Functions
    def item_holders_callback(self, msg):
        self.item_holders = msg

    def robot_marker_callback(self, msg):
        # topic_name = msg.text  
        # robot_id = topic_name.split('/')[1]  
        robot_id = msg.header.frame_id
        self.robots_pose_dict[robot_id] = msg.pose
        self.robots_state_dict[robot_id] = msg.text

    def assign_task(self, robot_id):
        task = Task()




    # Control Loop
    def control_loop(self):
        # Prioritise depositing items in currently being held by robots
        for robot in self.item_holders:
            if robot.holding_item:
                if self.robots_state[robot.robot_id] ==  "State.IDLE":
                    task = Task()
                    task.robot_id = robot.robot_id
                    if robot.item_colour not in self.zones:
                        # If the item colour doesn't exist, assign it to a zone
                        self.zones[robot.item_colour] = robot.item_colour
                    # Now assign the destination based on the item colour
                    task.destination = self.zones[robot.item_colour]
                    task.action = DROP_OFF
                    self.task_publisher.publish(task)
            else:
                pass
            

        # Give any idle robots, without a
        for robot_id, state in self.robots_state_dict.items():
            if state == "State.IDLE":
                task
        

    def assign_tasks(self):
        """Assign tasks to robots dynamically."""
        for robot_id, status in self.robots.items():
            if status == 'idle':
                # Find a suitable item and zone
                task = self.create_task(robot_id)
                if task:
                    self.task_pub.publish(task)
                    self.get_logger().info(f"Task assigned to robot {robot_id}: {task}")

    def create_task(self, robot_id):
        """Create a task for the robot based on available items and zones."""
        for item in self.items:
            for zone_id, color in self.zones.items():
                if item['color'] == color:
                    # Create and return a task message
                    task = Task()
                    task.robot_id = robot_id
                    task.item_color = item['color']
                    task.item_location = Point(x=item['x'], y=item['y'], z=0.0)
                    task.zone = Point(x=zone_id['x'], y=zone_id['y'], z=0.0)
                    
                    # Remove assigned item
                    self.items.remove(item)
                    return task
        return None
    
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
