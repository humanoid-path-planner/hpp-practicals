from motion_planner import MotionPlanner

import numpy as np
from pinocchio import SE3
from pyhpp.pinocchio import Device, urdf
from pyhpp.core import Problem, Roadmap, WeighedDistance
from pyhpp.gepetto.viewer import Viewer
# Robot configuration
urdfFilename = "package://example-robot-data/robots/ur_description/urdf/ur5_joint_limited_robot.urdf"
srdfFilename = "package://example-robot-data/robots/ur_description/srdf/ur5_joint_limited_robot.srdf"

# Initialize robot and viewer
robot = Device("ur5")

# Add robot and obstacles to scene
urdf.loadModel(robot, 0, "r0", "anchor", urdfFilename, srdfFilename, SE3.Identity())


urdf.loadModel(robot, 0, "table", "anchor", "package://hpp_environments/urdf/ur_benchmark/table.urdf", "", SE3.Identity())
urdf.loadModel(robot, 0, "wall", "anchor", "package://hpp_environments/urdf/ur_benchmark/wall.urdf", "", SE3.Identity())
urdf.loadModel(robot, 0, "obstacles", "anchor", "package://hpp_environments/urdf/ur_benchmark/obstacles.urdf", "", SE3.Identity())


# Define initial and goal configurations
q1 = np.array([0.2, -1.57, -1.8, 0, 0.8, 0])
q2 = np.array([1.57, -1.57, -1.8, 0, 0.8, 0])

# Setup problem and RRT components
problem = Problem(robot)
configurationShooter = problem.configurationShooter()
steer = problem.steeringMethod()
weighedDistance = WeighedDistance(robot)

# Initialize roadmap
roadmap = Roadmap(weighedDistance, robot)
roadmap.initNode(q1)
roadmap.addGoalNode(q2)

m = MotionPlanner(robot, problem, roadmap)
path = m.solveBiRRT(maxIter=1000)

# v = Viewer(robot)
# v(q1)
# v(q2)
# v.playPath(path)
