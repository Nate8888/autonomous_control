# Import necessary dependencies
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Int8
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
import threading
# LinkStates:

# broadcast all link states in world frame
# string[] name                 # link names
# geometry_msgs/Pose[] pose     # desired pose in world frame
# geometry_msgs/Twist[] twist   # desired twist in world frame

from sensor_msgs.msg import JointState, Imu, LaserScan
import sys

# Check if the action library is necessary
from rclpy.action import ActionServer
from action_interfaces.action import Waypoint

from msgsrc_interfaces.srv import GetRoverStatus
from msgsrc_interfaces.msg import Path

from .ai_objects import *
from .auto_functions import *
from .utility_functions import *


class RoverController(Node):
    """RoverController class that inherits from Node class in order to create an autonomous control node"""


    def __init__(
        self,
        target_x,
        target_y,
        start_x,
        start_y,
        movement_topic,
        front_arm_topic,
        back_arm_topic,
        front_drum_topic,
        back_drum_topic,
        obstacle_threshold,
        obstacle_buffer,
        move_increment,
        max_linear_velocity,
        max_angular_velocity,
        real_odometry,
        swarm_control,
        name
    ):
        """
        Initialize the rover controller class.

        Inputs:
        ======
        target_x: The x coordinate of the target location.
        target_y: The y coordinate of the target location.
        start_x: The x coordinate of the start location.
        start_y: The y coordinate of the start location.
        movement_topic: The topic to publish to for movement commands.
        front_arm_topic: The topic to publish to for the front arm.
        back_arm_topic: The topic to publish to for the back arm.
        front_drum_topic: The topic to publish to for the front drum.
        back_drum_topic: The topic to publish to for the back drum.
        obstacle_threshold: The distance threshold for an obstacle.
        obstacle_buffer: The distance buffer for an obstacle.
        move_increment: The distance increment for the rover to move.
        max_linear_velocity: The maximum linear velocity of the rover.
        max_angular_velocity: The maximum angular velocity of the rover.
        real_odometry: Boolean value that determines if the real odometry is used.
        swarm_control: Boolean value that determines if the swarm control is used.
        name: The name of the rover
        """
        super().__init__("autonomous_control")
        self.ROBOT_NAME = name # Default name used

        self.rate = self.create_rate(10, self.get_clock())
        self.namespace = self.get_namespace()
        self.get_logger().info('Created node w/ namespace: {}'.format(self.namespace))
        self.swarm_control = swarm_control

        # Used to trigger timer function execution
        self.front_arm_flag = False
        self.back_arm_flag = False
        self.target_angle = 0

        # Create Utility Objects
        self.world_state = WorldState(self)
        self.ros_util = ROSUtility(
            movement_topic,
            front_arm_topic,
            back_arm_topic,
            front_drum_topic,
            back_drum_topic,
            max_linear_velocity,
            max_angular_velocity,
            obstacle_threshold,
            obstacle_buffer,
            move_increment,
            self
        )


        # Setup Subscriber Callbacks
        if real_odometry:
            # Get initial spawn coords
            self.world_state.initial_spawn(start_x, start_y)
            self.odom = self.create_subscription(Odometry, "odometry/filtered", self.world_state.odometryCallBack, 10)
        else:
            self.gazebolink = self.create_subscription(LinkStates, "gazebo/link_states", self.world_state.simStateCallBack, 10)

        self.change_targets = self.create_subscription(Point, "{}/change_target".format(self.ROBOT_NAME), self.change_target, 10)
        
        
        self.imu = self.create_subscription(Imu, "{}/imu_plugin/out".format(self.ROBOT_NAME), self.world_state.imuCallBack, 10)
        self.joint_state = self.create_subscription(JointState, "{}/joint_states".format(self.ROBOT_NAME), self.world_state.jointCallBack, 10)
        self.autonomous = self.create_subscription(Int8, "{}/autonomous_toggles".format(self.ROBOT_NAME), self.ros_util.autoCommandCallBack, 10)
        
        self.combined = self.create_subscription(LaserScan, f"/{self.ROBOT_NAME}/obstacle_detection/combined", on_scan_update, 1)

        self.front_arm_timer = self.create_timer(0.1, self.front_arm_timer_cb)
        self.back_arm_timer = self.create_timer(0.1, self.back_arm_timer_cb)

        if swarm_control:
            self.swarm_target_sub = self.create_subscription(Point, "swarm_target", self.swarmTargetCallBack, 10)
        
        # if False: # if True to test swarm
        #     self.ros_util.auto_function_command = 16

        #     self.server_name = "waypoint"
        #     self.waypoint_server = ActionServer(
        #         self,
        #         Waypoint,
        #         'waypoint',
        #         self.execute_action
        #     )

        #     #self.waypoint_server.register_preempt_callback(self.preempt_cb)
        #     self.get_logger().info("Rover waypoint server initialized.")

        #     # Register GetRoverStatus, used by the swarm controller to retrieve
        #     # a rover's position and battery
        #     self.status_service = self.create_service(
        #         GetRoverStatus, "rover_status", self.send_status
        #     )
        #     self.get_logger().info("Rover status service initialized.")
        else:
            # Basic autonomous control using the autonomous control loop
            target_location = Point()
            temp = Point()

            target_location.x = float(target_x)
            target_location.y = float(target_y)

            temp.x = float(target_x)
            temp.y = float(target_y)

            self.world_state.target_location = target_location
            self.world_state.dig_site = temp
    
    def change_target(self, msg):
        """ Changes the target location of the rover

        Inputs:
        ======
        msg: The Point message containing the new target location.

        Outputs:
        =======
        -
        """

        target_location = Point()
        temp = Point()

        target_location.x = float(msg.x)
        target_location.y = float(msg.y)

        temp.x = float(msg.x)
        temp.y = float(msg.y)

        self.world_state.target_location = target_location
        self.world_state.dig_site = temp
        self.get_logger().info("Goal Changed to {} {}".format(msg.x, msg.y))

    
    def swarmTargetCallBack(self, msg):
        """ Changes the target location of the rover to the swarm target and triggers the autonomous control loop

        Inputs:
        ======
        msg: The Point message containing the new target location.

        Outputs:
        =======
        -
        """
        target_location = Point()
        temp = Point()

        target_location.x = float(msg.x)
        target_location.y = float(msg.y)

        temp.x = float(msg.x)
        temp.y = float(msg.y)

        self.world_state.target_location = target_location
        self.world_state.dig_site = temp
        self.get_logger().info("Goal Changed to {} {}".format(msg.x, msg.y))
        self.ros_util.auto_function_command = 1



    def front_arm_timer_cb(self):
        """
        Timer function for the front arm. It raises the arm when flag is raised.

        Inputs:
        ======
        -

        Outputs:
        =======
        -

        """
        if self.front_arm_flag:
            if self.target_angle > self.world_state.front_arm_angle:
                self.ros_util.publish_actions("stop", 1, 0, 0, 0)
                self.get_logger().info("Current front arm angle: {}".format(self.world_state.front_arm_angle))
                self.get_logger().info("target angle: {}".format(self.target_angle))
                d = Bool()
                d.data = True
                self.ros_util.arms_up_pub.publish(d)
            else:
                self.front_arm_flag = False
                self.ros_util.publish_actions("stop", -1, 0, 0, 0)
                # self.get_logger().info("Current front arm angle: {}".format(self.world_state.front_arm_angle))
                # self.get_logger().info("target angle: {}".format(self.target_angle))
                d = Bool()
                d.data = False
                self.ros_util.arms_up_pub.publish(d)
                self.ros_util.publish_actions("stop", 0, 0, 0, 0)

    def back_arm_timer_cb(self):
        """
        Timer function for the back arm. It raises back arm when flag is raised.

        Inputs:
        ======
        -

        Outputs:
        =======
        -
        """
        if self.back_arm_flag:
            if self.target_angle > self.world_state.back_arm_angle:
                self.ros_util.publish_actions("stop", 0, 1, 0, 0)
                self.get_logger().info("Current back arm angle: {}".format(self.world_state.back_arm_angle))
                self.get_logger().info("target angle: {}".format(self.target_angle))
            else:
                self.back_arm_flag = False
                self.ros_util.publish_actions("stop", 0, -1, 0, 0)
                self.ros_util.publish_actions("stop", 0, 0, 0, 0)
            
    def send_status(self, request, response):
        """Send the rover's battery and pose to the swarm controller.
        
        Inputs:
        ======
        request: The GetRoverStatus request message.
        response: The GetRoverStatus response message.

        Outputs:
        =======
        response: The populated GetRoverStatus response message.
        """
        self.get_logger().info("SENDING GetRoverStatus() with send_status function")
        response.pose.position.x = float(self.world_state.positionX)
        response.pose.position.y = float(self.world_state.positionY)
        response.pose.position.z = float(self.world_state.positionZ)
        response.battery = max(int(self.world_state.battery), 0)
        return response

    def execute_action(self, goal):
        """Handle a swarm controller action (depracated)."""

        self.get_logger().info("Received action goal.")

        if goal.request.target.x == -999 and goal.request.target.y == -999:
            self.get_logger().info(
                "Waypoint server {} executing charge command".format(
                    self.namespace + self.server_name
                )
            )

            # Set rover to charge
            # 
            charge_battery(self.world_state, self.ros_util)
            result = build_result(self.world_state, preempted=0)

            #goal.publish_feedback(result)

            self.waypoint_server.set_succeeded(result)

        elif goal.request.target.x == -998 and goal.request.target.y == -998:
            self.get_logger().info(
                "Waypoint server {} executing dig command".format(
                    self.namespace + self.server_name
                )
            )
            # Set rover to dig for 1000 seconds
            feedback, preempted = auto_dig( 
                self.world_state, self.ros_util, 1000, self, self.waypoint_server
            ) #auto_dig

            goal.publish_feedback(feedback)

            if (
                feedback is not None
                and not preempted
                and not self.waypoint_server.is_preempt_requested()
            ):
                result = build_result(self.world_state, preempted=0)
                self.waypoint_server.set_succeeded(result)

        else:
            self.world_state.target_location = goal.request.target
            self.get_logger().info(
                "Waypoint server {} moving rover to {}".format(
                    self.namespace + self.server_name,
                    (goal.request.target.x, goal.request.target.y),
                )
            )

            # Set rover to autonomously navigate to target
            set_front_arm_angle(self.world_state, self.ros_util, 1.3)
            set_back_arm_angle(self.world_state, self.ros_util, 1.3)
            
            feedback = Waypoint.Feedback()
            preempted = False
            # feedback, preempted = auto_drive_location(self.world_state, self.ros_util, self, self.waypoint_server)
            
            goal.publish_feedback(feedback)

            # Send resulting state to client and set server to succeeded, as
            # long as request wasn't preempted
            if (
                feedback is not None
                and not preempted
                # and not self.waypoint_server.is_preempt_requested()
            ):
                result = Waypoint.Result(feedback.pose, feedback.battery, 0) #waypointResult(feedback.pose, feedback.battery, 0)
                
                goal.succeed()
                #self.waypoint_server.set_succeeded(result)
                return result

    def preempt_cb(self):
        """Handle preempt request from swarm controller. (depracated)"""
        if self.waypoint_server.is_preempt_requested():
            # Stop the rover
            self.ros_util.publish_actions("stop", 0, 0, 0, 0)

            # Send rover status while preempting the waypoint goal
            result = build_result(self.world_state, preempted=1)

            # ADDED 2.0

            self.waypoint_server.set_preempted(result)

    def full_autonomy(self, world_state, ros_util):
        """ Full Autonomy Loop Function
        
        Inputs:
        ======
        world_state: The current world state.
        ros_util: The ROS utility object.

        Outputs:
        =======
        -
        """

        self.get_logger().info("Full autonomy activated.")

        while ros_util.auto_function_command == 16:
            auto_drive_location(world_state, ros_util, self)
            if ros_util.auto_function_command != 16:
                break
            auto_dig(world_state, ros_util, 7, self)
            if ros_util.auto_function_command != 16:
                break
            auto_dock(world_state, ros_util, self)
            if ros_util.auto_function_command != 16:
                break
            auto_dump(world_state, ros_util, 4, self)
            world_state.target_location.x = world_state.dig_site.x
            world_state.target_location.y = world_state.dig_site.y

        world_state.target_location.x = world_state.dig_site.x
        world_state.target_location.y = world_state.dig_site.y

    def autonomous_control_loop(self, world_state, ros_util):
        """ Control Auto Functions based on auto_function_command input.
        
        Inputs:
        ======
        world_state: The current world state.
        ros_util: The ROS utility object.

        Outputs:
        =======
        -
        """

        while True:
            while (
                ros_util.auto_function_command == 0
                or ros_util.auto_function_command == 32
            ):
                ros_util.publish_actions("stop", 0, 0, 0, 0)
                self.get_logger().info("publishing stop")
                self.rate.sleep()

            d = Bool()
            d.data = True
            ros_util.control_pub.publish(d)

            if ros_util.auto_function_command == 1:
                auto_drive_location(world_state, ros_util, self)
            elif ros_util.auto_function_command == 2:
                auto_dig(world_state, ros_util, 10, self)
            elif ros_util.auto_function_command == 4:
                auto_dump(world_state, ros_util, 4, self)
            elif ros_util.auto_function_command == 8:
                auto_dock(world_state, ros_util, self)
            elif ros_util.auto_function_command == 16:
                self.full_autonomy(world_state, ros_util)
            else:
                self.get_logger().info(
                    "Invalid auto-function request: %s.",
                    str(ros_util.auto_function_command),
                )

            ros_util.auto_function_command = 0
            ros_util.publish_actions("stop", 0, 0, 0, 0)
            d = Bool()
            d.data = False
            ros_util.control_pub.publish(d)


def on_start_up(
    target_x=10,
    target_y=10,
    start_x=0,
    start_y=0,
    movement_topic="/wheel_instructions",
    front_arm_topic="/front_arm_instructions",
    back_arm_topic="/back_arm_instructions",
    front_drum_topic="/front_drum_instructions",
    back_drum_topic="/back_drum_instructions",
    obstacle_threshold=3.0,
    obstacle_buffer=2.0,
    move_increment=3.0,
    max_linear_velocity=1.0,
    max_angular_velocity=10.0,
    real_odometry=False,
    swarm_control=False,
    name="ezrassor",
    args=None
):
    """ Initialization Function
    
    Inputs:
    ======
    target_x: The x coordinate of the target location.
    target_y: The y coordinate of the target location.
    start_x: The x coordinate of the start location.
    start_y: The y coordinate of the start location.
    movement_topic: The topic to publish movement commands to.
    front_arm_topic: The topic to publish front arm commands to.
    back_arm_topic: The topic to publish back arm commands to.
    front_drum_topic: The topic to publish front drum commands to.
    back_drum_topic: The topic to publish back drum commands to.
    obstacle_threshold: The distance threshold for an obstacle to be considered
    obstacle_buffer: The distance buffer for an obstacle to be considered
    move_increment: The distance increment for each move command
    max_linear_velocity: The maximum linear velocity of the rover
    max_angular_velocity: The maximum angular velocity of the rover
    real_odometry: Whether to use real odometry or simulated odometry
    swarm_control: Whether to use swarm control or autonomous control
    name: The name of the rover
    args: The arguments passed to the function

    Outputs:
    =======
    -
    """
    rclpy.init(args=args)

    rover_controller = RoverController(
        target_x,
        target_y,
        start_x,
        start_y,
        name + movement_topic,
        name + front_arm_topic,
        name + back_arm_topic,
        name + front_drum_topic,
        name + back_drum_topic,
        obstacle_threshold,
        obstacle_buffer,
        move_increment,
        max_linear_velocity,
        max_angular_velocity,
        real_odometry,
        swarm_control,
        name
    )

    thread = threading.Thread(target=rclpy.spin, args=(rover_controller, ), daemon=True)
    thread.start()
    rover_controller.rate = rover_controller.create_rate(10)
    rover_controller.autonomous_control_loop(rover_controller.world_state, rover_controller.ros_util)


    rclpy.shutdown()
    thread.join()
