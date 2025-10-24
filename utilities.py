from math import atan2, asin, sqrt
import os

M_PI=3.1415926535

class Logger:
    
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        
        self.filename = filename

        # Ensure the parent directory exists if provided
        parent_dir=os.path.dirname(self.filename)
        if parent_dir != "":
            os.makedirs(parent_dir, exist_ok=True)

        with open(self.filename, 'w') as file:
            
            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "
            
            header_str+="\n"
            
            file.write(header_str)


    def log_values(self, values_list):

        with open(self.filename, 'a') as file:
            
            vals_str=""
            
            for value in values_list:
                vals_str+=f"{value}, "
            
            vals_str+="\n"
            
            file.write(vals_str)
            

    def save_log(self):
        # Ensure file exists and is flushed; this method can be expanded if needed
        try:
            with open(self.filename, 'a') as file:
                file.flush()
        except Exception:
            # Intentionally ignore logging persistence errors to not crash the node
            return


class FileReader:
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table
    
    

# TODO Part 3: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    x, y, z, w = quat

    # Yaw (rotation about Z axis)
    # Formula: atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return yaw # for our purposes, we only care about yaw


#TODO Part 4: Implement the calculation of the linear error
def calculate_linear_error(current_pose, goal_pose):
        
    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y]
    # Remember to use the Euclidean distance to calculate the error.
    dx = goal_pose[0] - current_pose[0]
    dy = goal_pose[1] - current_pose[1]
    error_linear = sqrt(dx ** 2 + dy ** 2)

    return error_linear

#TODO Part 4: Implement the calculation of the angular error
def calculate_angular_error(current_pose, goal_pose):

    dx = goal_pose[0] - current_pose[0]
    dy = goal_pose[1] - current_pose[1]
    desired_theta = atan2(dy, dx)
    error_angular = desired_theta - current_pose[2]
    error_angular = (error_angular + M_PI) % (2 * M_PI) - M_PI
    return error_angular
