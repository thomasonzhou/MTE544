import numpy as np


from pid import PID_ctrl
from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error

M_PI=3.1415926535

P=0; PD=1; PI=2; PID=3

class controller:
    
    
    # Default gains of the controller for linear and angular motions
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        
        # TODO Part 5 and 6: Modify the below lines to test your PD, PI, and PID controller
        # Instantiate the linear and angular PID controllers with your selected gains and log files
        self.PID_linear=PID_ctrl(PID, klp, klv, kli, filename_="logs/linear.csv")
        self.PID_angular=PID_ctrl(PID, kap, kav, kai, filename_="logs/angular.csv")

    
    def vel_request(self, pose, goal, status):
        
        e_lin=calculate_linear_error(pose, goal) # Calculate the linear error between current and goal pose
        e_ang=calculate_angular_error(pose, goal) # Calculate the angular error between current and goal pose

        # Call the PID controllers to compute the velocity commands using the current errors and pose timestamp
        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status)
        
        # TODO Part 4: Add saturation limits for the robot linear and angular velocity (hint: you can use np.clip function)

        # Limit the velocities to hardware constraints using np.clip
        # Constraints are found from online documentation

        # TurtleBot3 Burger
        # linear_vel = np.clip(linear_vel, -0.22, 0.22)
        # angular_vel = np.clip(angular_vel, -2.84, 2.84)
        
        # TurtleBot4 
        linear_vel = np.clip(linear_vel, -0.31, 0.31)
        angular_vel = np.clip(angular_vel, -1.90, 1.90)
        
        return linear_vel, angular_vel
    

class trajectoryController(controller):

    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        
        super().__init__(klp, klv, kli, kap, kav, kai)
    
    def vel_request(self, pose, listGoals, status):
        
        goal=self.lookFarFor(pose, listGoals) # Look-ahead method for smoother path following
        
        finalGoal=listGoals[-1]
        
        e_lin = calculate_linear_error(pose, finalGoal) # Compute error to final goal for linear velocity
        e_ang = calculate_angular_error(pose, goal)     # Compute error to intermediate goal for angular velocity

        # Call the PID controllers to compute the velocity commands using the current errors and pose timestamp
        linear_vel = self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel = self.PID_angular.update([e_ang, pose[3]], status) 

        # TODO Part 5: Add saturation limits for the robot linear and angular velocity (hint: you can use np.clip function)

        # Limit the velocities to hardware constraints for TurtleBot4 using np.clip
        # TurtleBot3 Burger
        # linear_vel = np.clip(linear_vel, -0.22, 0.22)
        # angular_vel = np.clip(angular_vel, -2.84, 2.84)
        
        # TurtleBot4
        linear_vel = np.clip(linear_vel, -0.31, 0.31)
        angular_vel = np.clip(angular_vel, -1.90, 1.90)

        
        return linear_vel, angular_vel

    def lookFarFor(self, pose, listGoals):
        # Returns a lookahead target point further along the list of goals for smoother trajectory following
        poseArray=np.array([pose[0], pose[1]]) 
        listGoalsArray=np.array(listGoals)

        distanceSquared=np.sum((listGoalsArray-poseArray)**2,
                               axis=1)
        closestIndex=np.argmin(distanceSquared)

        return listGoals[ min(closestIndex + 3, len(listGoals) - 1) ]
