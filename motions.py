# Imports
import rclpy

from rclpy.node import Node

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile

# TODO Part 3: Import message types needed: 
    # For sending velocity commands to the robot: Twist
    # For the sensors: Imu, LaserScan, and Odometry
# Check the online documentation to fill in the lines below
from geometry_msgs.msg import Twist # https://docs.ros.org/en/humble/p/geometry_msgs/msg/Twist.html
from sensor_msgs.msg import Imu # https://docs.ros.org/en/humble/p/sensor_msgs/msg/Imu.html
from sensor_msgs.msg import LaserScan # https://docs.ros.org/en/humble/p/sensor_msgs/msg/LaserScan.html
from nav_msgs.msg import Odometry # https://docs.ros.org/en/humble/p/nav_msgs/msg/Odometry.html

from rclpy.time import Time

# You may add any other imports you may need/want to use below
import math
import argparse


CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

class motion_executioner(Node):
    """Publishes velocity commands for preset motions and records sensor streams."""

    def __init__(self, motion_type=0):
        """Prepare publishers, subscriptions, loggers, and motion state."""

        super().__init__("motion_types")
        
        self.type=motion_type
        
        self.radius = 0.0 # global radius variable for spiral motion
        
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False
        
        # TODO Part 3: Create a publisher to send velocity commands by setting the proper parameters in (...)
        self.vel_publisher=self.create_publisher(Twist, "/cmd_vel", 10)
                
        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "angle_increment", "stamp"])
        
        # TODO Part 3: Create the QoS profile by setting the proper parameters in (...)
        qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)

        # TODO Part 5: Create below the subscription to the topics corresponding to the respective sensors
        # IMU subscription
        self.create_subscription(Imu, '/imu', self.imu_callback, qos_profile=qos)
        
        # ENCODER subscription
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile=qos)
        
        # LaserScan subscription 
        self.create_subscription(LaserScan, '/scan', self.laser_callback, qos_profile=qos)
        
        self.create_timer(0.1, self.timer_callback)


    # TODO Part 5: Callback functions: complete the callback functions of the three sensors to log the proper data.
    # To also log the time you need to use the rclpy Time class, each ros msg will come with a header, and then
    # inside the header you have a stamp that has the time in seconds and nanoseconds, you should log it in nanoseconds as 
    # such: Time.from_msg(imu_msg.header.stamp).nanoseconds
    # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py

    def imu_callback(self, imu_msg: Imu):
        """
        Extracts linear acceleration (x, y) and angular velocity (z) from the IMU message,
        logs them to the IMU logger. 
        Sets imu_initialized to True to indicate the IMU stream active.
        """
        acc_x = imu_msg.linear_acceleration.x
        acc_y = imu_msg.linear_acceleration.y
        angular_z = imu_msg.angular_velocity.z
        timestamp = Time.from_msg(imu_msg.header.stamp).nanoseconds

        self.imu_logger.log_values([acc_x, acc_y, angular_z, timestamp])

        self.imu_initialized = True
        
    def odom_callback(self, odom_msg: Odometry):
        """
        Extracts the robot's x and y position, computes the yaw (heading) from the orientation quaternion,
        and logs these values to the odometry logger.
        Sets odom_initialized to True to indicate the odometry stream is active.
        """
        odom_x = odom_msg.pose.pose.position.x
        odom_y = odom_msg.pose.pose.position.y

        quat = odom_msg.pose.pose.orientation
        yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        timestamp = Time.from_msg(odom_msg.header.stamp).nanoseconds

        self.odom_logger.log_values([odom_x, odom_y, yaw, timestamp])
        self.odom_initialized = True


    def laser_callback(self, laser_msg: LaserScan):
        """
        Extracts the full range array and angle increment from the LaserScan message,
        logs them to the laser logger.
        Sets laser_initialized to True to indicate the LiDAR stream is active.
        """
        ranges = laser_msg.ranges  # Array of range measurements (len(ranges) == 360)
        angle_increment = laser_msg.angle_increment  # Angle increment per measurement (radians)
        timestamp = Time.from_msg(laser_msg.header.stamp).nanoseconds

        self.laser_logger.log_values([ranges, angle_increment, timestamp])
        self.laser_initialized = True
                
    def timer_callback(self):
        """Publish the active motion once all required sensors report."""
        
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True
            
        if not self.successful_init:
            return
        
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        self.vel_publisher.publish(cmd_vel_msg)
        
    # TODO Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot

    def make_circular_twist(self):
        """Return a Twist that drives a constant-radius circle."""

        msg=Twist()

        radius = 0.4
        omega = 0.5  # angular velocity (rad/s)
        v = omega * radius  # linear speed for the chosen radius

        msg.linear.x = v
        msg.angular.z = omega

        return msg

    def make_spiral_twist(self):
        """Return a Twist with growing radius to trace a planar spiral.
           self.radius is incremented (up to a max value); appropriate linear
           speed is calculated based on self.radius and constant omega.
        """

        msg = Twist()
        
        max_radius = 1.5
        increment = 0.01
        omega = 3.0  # angular velocity (rad/s)

        # grow the spiral radius
        self.radius = min(self.radius + increment, max_radius)

        # compute linear velocity based on new radius
        linear = self.radius * omega

        # assign velocities to the Twist message
        msg.linear.x = linear
        msg.angular.z = omega

        return msg
    
    def make_acc_line_twist(self):
        """Return a Twist that maintains a straight path at constant speed."""
        msg=Twist()
        # fill up the twist msg for line motion
        msg.linear.x = 0.5
        msg.angular.z = 0.0
        return msg


if __name__=="__main__":
    
    argParser=argparse.ArgumentParser(description="input the motion type")
    argParser.add_argument("--motion", type=str, default="circle")

    rclpy.init()

    args = argParser.parse_args()

    ME = None
    if args.motion.lower() == "circle":
        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)

    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)

    else:
        print(f"we don't have {args.motion.lower()} motion type")

    if ME is not None:
        try:
            rclpy.spin(ME)
        except KeyboardInterrupt:
            print("Exiting")