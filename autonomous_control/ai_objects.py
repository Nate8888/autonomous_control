#!/usr/bin/env python

import rclpy
from .nav_functions import *
import math
from random import uniform
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Point, Twist


class WorldState:
    """World State Object Representing
    All Sensor Data
    """

    def __init__(self, node):
        self.positionX = 0
        self.positionY = 0
        self.positionZ = 0
        self.startPositionX = 0
        self.startPositionY = 0
        self.front_arm_angle = 0
        self.back_arm_angle = 0
        self.front_arm_angle = 0
        self.heading = 0
        self.warning_flag = 0
        self.target_location = Point()
        self.on_side = False
        self.on_back = False
        self.front_up = False
        self.back_up = False
        self.battery = 100
        self.hardware_status = True
        self.node = node

    def jointCallBack(self, data):
        """ Finds the angle of the front and back arm.

        Inputs
        ======
        data: The joint state message -> JointState(metaclass=Metaclass_JointState)

        Outputs
        =======
        self.front_arm_angle: The angle of the front arm
        self.back_arm_angle: The angle of the back arm
        """
        
        # Find the index of 'arm_front_joint'
        index_farm = data.name.index("arm_front_joint")
        index_barm = data.name.index("arm_back_joint")
        self.front_arm_angle = data.position[index_farm]
        self.back_arm_angle = data.position[index_barm]

        # self.node.get_logger().info("front arm angle: {}".format(self.front_arm_angle))
        # self.node.get_logger().info("back arm angle: {}".format(self.back_arm_angle))

    def odometryCallBack(self, data):
        """ Finds which way we need to turn to face the target goal

        Inputs
        ======
        data: The odometry message -> Odometry(metaclass=Metaclass_Odometry)
        
        Outputs
        =======
        self.heading: The heading of the robot
        """

        self.positionX = data.pose.pose.position.x + self.startPositionX
        self.positionY = data.pose.pose.position.y + self.startPositionY

        heading = quaternion_to_yaw(data.pose.pose) * 180 / math.pi

        if heading > 0:
            self.heading = heading
        else:
            self.heading = 360 + heading

    def simStateCallBack(self, data):
        """ Callback function that updates the current position of the rover in the simulation.
        This is the function that depends on the gazebo simulation

        Inputs
        ======
        data: The LinkStates message -> LinkStates(metaclass=Metaclass_LinkStates)
        
        Outputs
        =======
        self.positionX: The x position of the rover in the simulation
        self.positionY: The y position of the rover in the simulation
        self.heading: The heading of the robot
        """
        index = 0

        # Get the namespace

        # SO APPARENTLY this method doesn't exist on the ros2 branch
        #namespace = rospy.get_namespace()

        # @TODO: figure out how to get the namespace in ros2
        # apparently we can use the node to get the namespace like
        # node.get_namespace()
        
        namespace = self.node.get_namespace()
        #self.node.get_logger().info('Trying to get the namespace inside simState: {}'.format(namespace))

        namespace = namespace[1:-1] + "::base_link"
        default_ns = "{}::base_link".format(self.node.ROBOT_NAME)
        try:
            index = data.name.index(default_ns)
        except Exception:
            self.node.get_logger().info('Hit exception')
            return

        self.positionX = data.pose[index].position.x
        self.positionY = data.pose[index].position.y

        # Display the positionX and positionY of the robot
        #self.node.get_logger().info("Robot Position Update: ({},{})".format(self.positionX, self.positionY))

        heading = quaternion_to_yaw(data.pose[index]) * 180 / math.pi

        if heading > 0:
            self.heading = heading
        else:
            self.heading = 360 + heading

    def imuCallBack(self, data):
        """ Callback function that receives imu data from the rover and updates rover status.

        Inputs
        ======
        data: The Imu message -> Imu(metaclass=Metaclass_Imu)
        
        Outputs
        =======
        self.on_side: True if the rover is in the side position
        """
        if abs(data.linear_acceleration.y) > 9:
            self.on_side = True
        else:
            self.on_side = False

    def get_arm_force(self):
        """ Get the force of the front and back arm.

        Inputs
        ======
        -
        
        Outputs
        =======
        front_arm_force: The force of the front arm
        back_arm_force: The force of the back arm
        """
        front_arm_force = (
            self.state_flags["front_arm_angle"] + 0.2 + uniform(-0.2, 0.2)
        )
        back_arm_force = (
            self.state_flags["back_arm_angle"] + 0.2 + uniform(-0.2, 0.2)
        )
        return front_arm_force, back_arm_force

    # Use initial spawn coordinates to later offset position
    def initial_spawn(self, start_x, start_y):
        """ Set the initial spawn coordinates of the rover.

        Inputs
        ======
        start_x: The x position of the rover
        start_y: The y position of the rover
        
        Outputs
        =======
        self.startPositionX: The x position of the rover
        self.startPositionY: The y position of the rover
        """
        self.startPositionX = start_x
        self.startPositionY = start_y


class ROSUtility:
    """ROS Utility class that provides publishers,
    subscribers, and convenient ROS utilies.
    """

    def __init__(
        self,
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
        node
    ):
        """ Initialize the ROS utility class.

        Inputs
        ======
        movement_topic: The topic to publish movement commands to
        front_arm_topic: The topic to publish front arm commands to
        back_arm_topic: The topic to publish back arm commands to
        front_drum_topic: The topic to publish front drum commands to
        back_drum_topic: The topic to publish back drum commands to
        max_linear_velocity: The maximum linear velocity of the rover
        max_angular_velocity: The maximum angular velocity of the rover
        obstacle_threshold: The threshold for obstacle detection
        obstacle_buffer: The buffer for obstacle detection
        move_increment: The increment for movement commands
        node: The node to use for ROS
        
        Outputs
        =======
        self.node: The node to use for ROS
        self.movement_pub: The movement publisher
        self.front_arm_pub: The front arm publisher
        self.back_arm_pub: The back arm publisher
        self.front_drum_pub: The front drum publisher
        self.back_drum_pub: The back drum publisher
        self.control_pub: The control publisher
        self.arms_up_pub: The arms up publisher
        self.max_linear_velocity: The maximum linear velocity of the rover
        self.max_angular_velocity: The maximum angular velocity of the rover
        self.obstacle_threshold: The threshold for obstacle detection
        self.obstacle_buffer: The buffer for obstacle detection
        self.move_increment: The increment for movement commands
        """
        self.node = node
        self.movement_pub = self.node.create_publisher(Twist, movement_topic, 10)
        self.front_arm_pub = self.node.create_publisher(Float64, front_arm_topic, 10)
        self.back_arm_pub = self.node.create_publisher(Float64, back_arm_topic, 10)
        self.front_drum_pub = self.node.create_publisher(Float64, front_drum_topic, 10)
        self.back_drum_pub = self.node.create_publisher(Float64, back_drum_topic, 10)
        # @TODO check if it's secondary_override_toggle or /secondary_override_toggle
        self.control_pub = self.node.create_publisher(Bool, "{}/secondary_override_toggle".format(self.node.ROBOT_NAME), 10)
        self.arms_up_pub = self.node.create_publisher(Bool, "{}/arms_up".format(self.node.ROBOT_NAME), 10)
        
        # Setting up a hz rate?
        # https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
        #self.rate = rclpy.Rate(45)  # 10hz

        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity

        self.auto_function_command = 0

        self.threshold = 0.5

        self.obstacle_threshold = obstacle_threshold
        self.obstacle_buffer = obstacle_buffer
        self.move_increment = move_increment

    def publish_actions(
        self, movement, front_arm, back_arm, front_drum, back_drum
    ):
        """ Publishes actions for all joints and motors 
        
        Inputs
        ======
        movement: The movement command to publish
        front_arm: The front arm command to publish
        back_arm: The back arm command to publish
        front_drum: The front drum command to publish
        back_drum: The back drum command to publish

        Outputs
        =======
        -
        
        """

        twist_message = Twist()

        if movement == "forward":
            twist_message.linear.x = self.max_linear_velocity
        elif movement == "reverse":
            twist_message.linear.x = -self.max_linear_velocity
        elif movement == "left":
            twist_message.angular.z = self.max_angular_velocity
        elif movement == "right":
            twist_message.angular.z = -self.max_angular_velocity
        else:
            pass

        front_arm_float = Float64()
        front_arm_float.data = float(front_arm)
        back_arm_float = Float64()
        back_arm_float.data = float(back_arm)
        front_drum_float = Float64()
        front_drum_float.data = float(front_drum)
        back_drum_float = Float64()
        back_drum_float.data = float(back_drum)

        self.movement_pub.publish(twist_message)
        self.front_arm_pub.publish(front_arm_float)
        self.back_arm_pub.publish(back_arm_float)
        self.front_drum_pub.publish(front_drum_float)
        self.back_drum_pub.publish(back_drum_float)

    def autoCommandCallBack(self, data):
        """ Callback function that receives auto function commands.
        
        Inputs
        ======
        data: The Int8 message to process

        Outputs
        =======
        self.auto_function_command: The auto function command
        
        """
        # get_logger message with new data
        self.node.get_logger().info('Received auto command: {}'.format(data.data))
        self.auto_function_command = data.data

        self.node.get_logger().info("Autonomous control initialized.")
        #self.node.autonomous_control_loop(self.node.world_state, self.node.ros_util)
