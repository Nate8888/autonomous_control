import random
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from std_msgs.msg import Int8, Float64, Bool
from geometry_msgs.msg import Point, Twist, Pose

from action_interfaces.action import Waypoint
from msgsrc_interfaces.srv import GetRoverStatus
from msgsrc_interfaces.msg import Path

class WaypointBridgeActionServer(Node):

    def __init__(self):
        super().__init__('waypoint_bridge')
        self.status_sub = self.create_subscription(Int8, 'ezrassor/mode', self.mode_callback, 10)
        self.mode = 0
        self.rate = self.create_rate(10, self.get_clock())
        self.server_name = "waypoint"
        self.waypoint_server = ActionServer(
            self,
            Waypoint,
            'waypoint',
            self.execute_action
        )
        self.get_logger().info('Waypoint Bridge Action Server Ready!')
        # Create a subscriber to listen to topic ezrassor/mode Int8
       
    
    def mode_callback(self, msg):
        self.mode = msg.data
    
    def execute_action(self, goal_handle):
        """
        Action Server handling:

        Inputs:
        ======
        goal_handle: GoalHandle for the action server
        
            Goal:
            - Path path: Path to follow

            Result:
            - geometry_msgs/Pose pose
            - int8 battery
            - int8 preempted

            Feedback:

            geometry_msgs/Pose pose
            int8 battery

        Outputs:
        =======
        result: Result of the action
        """
        the_path = Path()
        the_path.path = goal_handle.request.path.path
        it = 0
        self.target_pub = self.create_publisher(Path, 'swarm_target', 10)
        self.get_logger().info('Executing goal...')
        self.get_logger().info('Bridge is sending Targets to EZ-RASSOR...')

        self.target_pub.publish(the_path)
        self.get_logger().info('Point Sent...')
        
        # Build Feedback_msg
        feedback_msg = Waypoint.Feedback()
        feedback_msg.pose = Pose()
        feedback_msg.pose.position.x = float(1)
        feedback_msg.pose.position.y = float(1)
        feedback_msg.battery = 100

        for i in range(10):
            # Get a random number from 1 to 10
            random_number = random.randint(1, 10)
            feedback_msg.pose.position.x = float(random_number)
            feedback_msg.pose.position.y = float(random_number)
            
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info('Feedback Sent...')

        # Builds result
        time.sleep(1)

        # Throw a Success message!
        goal_handle.succeed()
        result = Waypoint.Result()
        result.battery = 100
        result.pose = Pose()
        result.pose.position.x = float(1)
        result.pose.position.y = float(0)
        result.pose.position.z = float(0)
        
        self.get_logger().info('Result Returned...')
        return result

def main(args=None):
    rclpy.init(args=args)

    waypoint_action_server = WaypointBridgeActionServer()

    rclpy.spin(waypoint_action_server)


if __name__ == '__main__':
    main()