# Imports


import sys

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensor

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController
from utilities import calculate_linear_error

# You may add any other imports you may need/want to use below
# import ...
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from dataclasses import dataclass

# Created dataclass to contain PID params to speed up tuning process
@dataclass
class PID_Params:
    KP: float
    KD: float
    KI: float

class decision_maker(Node):
    
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, goalPoint, rate=10, motion_type=POINT_PLANNER):

        super().__init__("decision_maker")

        # TODO Part 4: Create a publisher for the topic responsible for robot's motion
        # We create the publisher for the robot's velocity commands using create_publisher
        self.publisher=self.create_publisher(publisher_msg, publishing_topic, qos_publisher)

        publishing_period=1/rate
        
        # Instantiate the controller
        # TODO Part 5: Tune your parameters here
        # Define the PID parameters for linear and angular controllers
        self.pid_a = PID_Params(0.4, 0.55, 0.05) #PDI

        # Choose planner and controller type according to the selected motion/planner mode.
        if motion_type == POINT_PLANNER:
            self.controller=controller(klp=self.pid_l.KP, klv=self.pid_l.KD, kli=self.pid_l.KI, kap=self.pid_a.KP, kav=self.pid_a.KD, kai=self.pid_a.KI)
            self.planner=planner(POINT_PLANNER)    
    
        elif motion_type==TRAJECTORY_PLANNER:
            self.controller=trajectoryController(klp=self.pid_l.KP, klv=self.pid_l.KD, kli=self.pid_l.KI, kap=self.pid_a.KP, kav=self.pid_a.KD, kai=self.pid_a.KI)
            self.planner=planner(TRAJECTORY_PLANNER)

        else:
            print("Error! you don't have this planner", file=sys.stderr)


        # Instantiate the localization, use rawSensor for now  
        # This will provide the robot's current pose using raw sensor data
        self.localizer=localization(rawSensor)

        # Instantiate the planner and get the planned path/goal
        # NOTE: goalPoint is used only for the pointPlanner
        self.goal=self.planner.plan(goalPoint)

        # Create timer to periodically call the callback controlling robot motion
        self.create_timer(publishing_period, self.timerCallback)

    def timerCallback(self):
        # TODO Part 3: Run the localization node
        # Call spin_once so the localization (another Node) processes any incoming odometry messages and updates the pose.
        spin_once(self.localizer)
        
        if self.localizer.getPose() is None:
            print("waiting for odom msgs ....")
            return

        vel_msg = Twist()

        # Check if we reached the goal, and choose correct goal for trajectory vs point planner
        if type(self.goal) == list:  # trajectory: list of points
            last_goal = self.goal[-1]
            cur_pose = self.localizer.getPose()
            # change tolerance as needed
            reached_goal = calculate_linear_error(cur_pose, last_goal) < 0.07
        else:  # point planner, goal is [x, y]
            cur_pose = self.localizer.getPose()
            reached_goal = calculate_linear_error(cur_pose, self.goal) < 0.07  # 7cm tolerance

        if reached_goal:
            print("reached goal")
            self.publisher.publish(vel_msg)
            
            # Save the PID controller logs for both angular and linear controllers
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()
            
            # TODO Part 3: exit the spin
            # Exits the main event loop to stop ROS2 spinning when goal is reached (terminates process).
            import sys
            sys.exit(0)
        
        # Ask the controller for velocity commands based on the current pose and goal
        velocity, yaw_rate = self.controller.vel_request(self.localizer.getPose(), self.goal, True)

        # Publish the velocity to move the robot
        vel_msg.linear.x = velocity
        vel_msg.angular.z = yaw_rate
        self.publisher.publish(vel_msg)

import argparse


def main(args=None):
    
    init()

    # TODO Part 3: You might need to change the QoS profile based on whether you're using the real robot or in simulation.
    # This QoS profile is chosen for the decision maker - use best effort for odom
    cmd_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10
    )

    TARGET_POSE = [1, 1]

    # TODO Part 4: instantiate the decision_maker with the proper parameters for moving the robot
    # Create the decision_maker node, specifying the command message type, topic, QoS profile, and target/goal.
    if args.motion.lower() == "point":
        DM=decision_maker(Twist, "/cmd_vel", cmd_qos, goalPoint=TARGET_POSE, motion_type=POINT_PLANNER)
    elif args.motion.lower() == "trajectory":
        print("using trajectory planner")
        DM=decision_maker(Twist, "/cmd_vel", cmd_qos, TARGET_POSE, motion_type=TRAJECTORY_PLANNER)
    else:
        print("invalid motion type", file=sys.stderr)        
    
    # Start spinning (event loop) to allow node to process callbacks for controlling the robot
    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")


if __name__=="__main__":

    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="point")
    args = argParser.parse_args()

    main(args)
