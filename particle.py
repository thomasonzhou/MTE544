
from mapUtilities import *
from utilities import *
from numpy import cos, sin
import numpy as np


class particle:

    def __init__(self, pose, weight):
        self.pose = pose
        self.weight = weight

    def motion_model(self, v, w, dt):
        #TODO: Implement the motion model for the particle
        """
        v: linear velocity
        w: angular velocity
        dt: time step
        """
        # this is a linearized motion model that assumes the timestep is small and rotational displacement is minimal
        theta = self.pose[2]
        self.pose[0] += v*np.cos(theta)*dt
        self.pose[1] += v*np.sin(theta)*dt
        self.pose[2] += w*dt

    # TODO: You need to explain the following function to TA
    def calculateParticleWeight(self, scanOutput: LaserScan, mapManipulatorInstance: mapManipulator, laser_to_ego_transformation: np.array):

        # transform to align point clouds with the pose of the robot
        T = np.matmul(self.__poseToTranslationMatrix(), laser_to_ego_transformation) # laser frame to ego frame to map frame

        # convert polar coordinates (360 degrees with range) to x, y, 1
        _, scanCartesianHomo = convertScanToCartesian(scanOutput) # (360, 3)
        # transform cartesian points to be centered and rotated in map frame
        scanInMap = np.dot(T, scanCartesianHomo.T).T # (360, 3)

        # get a map with Gaussian likelihoods of obstacles
        # it doesn't sum to 1 but we only care about relative weights of particles
        # (we will ensure total probability of selection is 1 using normalization)
        likelihoodField = mapManipulatorInstance.getLikelihoodField()

        # get cell of each laser point in the map
        cellPositions = mapManipulatorInstance.position_2_cell(
            scanInMap[:, 0:2])

        lm_x, lm_y = likelihoodField.shape

        # remove all laser scans outside the map boundaries
        cellPositions = cellPositions[np.logical_and.reduce(
                (cellPositions[:, 0] > 0, -cellPositions[:, 1] > 0, cellPositions[:, 0] < lm_y,  -cellPositions[:, 1] < lm_x))]

        # take the log probability to allow for summation rather than multiplication
        log_weights = np.log(
            likelihoodField[-cellPositions[:, 1], cellPositions[:, 0]])
        # the sum of the log probablities defines how closely the walls detected by the particle match the map
        log_weight = np.sum(log_weights)
        weight = np.exp(log_weight) # take exponential to get back to the total probability (unscaled)
        weight += 1e-10 # add small value for numerical stability (we don't want to divide by zero)

        self.setWeight(weight)

    def setWeight(self, weight):
        self.weight = weight

    def getWeight(self):
        return self.weight

    def setPose(self, pose):
        self.pose = pose

    def getPose(self):
        return self.pose[0], self.pose[1], self.pose[2]

    def __poseToTranslationMatrix(self):
        x, y, th = self.getPose()

        translation = np.array([[cos(th), -sin(th), x],
                                [sin(th), cos(th), y],
                                [0, 0, 1]])

        return translation
