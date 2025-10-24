# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1

import numpy as np

class planner:
    def __init__(self, type_):

        self.type=type_

    def plan(self, goalPoint=[-1.0, -1.0], trajectoryType="sigmoid"):
        """
        Returns a single goal point (for POINT_PLANNER) or a list of trajectory points (for TRAJECTORY_PLANNER).
        For trajectory, trajectoryType: "parabola" or "sigmoid"
        """
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner(trajectoryType=trajectoryType)

    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self, trajectoryType="sigmoid"):
        """
        Generate a trajectory as a list of [x, y] points.
        Supported: 'parabola', 'sigmoid'
        """
        traj = []

        if trajectoryType.lower() == "parabola":
            # Parabola: y = x^2, x in [0.0, 1.5]
            x_vals = np.linspace(0.0, 1.5, 75)  # 75 points, ~0.02 spacing
            for x in x_vals:
                y = x**2
                traj.append([x, y])
            return traj

        elif trajectoryType.lower() == "sigmoid":
            # Sigmoid: σ(x) = 2/(1+e^{-2x})-1, x in [0.0, 2.5]
            x_vals = np.linspace(0.0, 2.5, 100)  # 100 points, finer steps
            for x in x_vals:
                y = 2.0 / (1.0 + np.exp(-2*x)) - 1.0
                traj.append([x, y])
            return traj

        else:
            # Default: straight line to origin, just as a fallback
            return [[0.0, 0.0], [1.0, 0.0]]

        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
