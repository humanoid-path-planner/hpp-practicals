import numpy as np
from pinocchio import SE3, Quaternion
from pyhpp.core import ConfigurationShooter, Dichotomy  # noqa: F401
from pyhpp.manipulation import (
    Device,
    Graph,
    ManipulationPlanner,
    Problem,
    ProgressiveProjector,
    urdf,
)

urdf_ur5 = "package://example-robot-data/robots/ur_description/urdf/ur5_gripper.urdf"
srdf_ur5 = "package://example-robot-data/robots/ur_description/srdf/ur5_gripper.srdf"

urdf_ball = "package://hpp_practicals/urdf/ur_benchmark/pokeball.urdf"
srdf_ball = "package://hpp_practicals/srdf/ur_benchmark/pokeball.srdf"

urdf_ground = "package://hpp_practicals/urdf/ur_benchmark/ground.urdf"
srdf_ground = "package://hpp_practicals/srdf/ur_benchmark/ground.srdf"

robot = Device("bot")

urdf.loadModel(robot, 0, "ur5", "anchor", urdf_ur5, srdf_ur5, SE3.Identity())
urdf.loadModel(robot, 0, "pokeball", "freeflyer", urdf_ball, srdf_ball, SE3.Identity())

urdf.loadModel(robot, 0, "ground", "anchor", urdf_ground, srdf_ground, SE3.Identity())


ballName = "pokeball/root_joint"
robot.setJointBounds(
    ballName,
    [
        -0.4,
        0.4,
        -0.4,
        0.4,
        -0.1,
        1.0,
        -1.0001,
        1.0001,
        -1.0001,
        1.0001,
        -1.0001,
        1.0001,
        -1.0001,
        1.0001,
    ],
)

urdfFilenameBox = "package://hpp_practicals/urdf/ur_benchmark/box.urdf"
srdfFilenameBox = "package://hpp_practicals/srdf/ur_benchmark/box.srdf"


q = Quaternion(0, 0, 0, 1)
box_pos = SE3(q, np.array([0.3, 0, 0.04]))
urdf.loadModel(robot, 0, "box", "anchor", urdfFilenameBox, srdfFilenameBox, box_pos)

model = robot.model()
data = robot.data()

problem = Problem(robot)

q1 = [0, -1.57, 1.57, 0, 0, 0, 0.3, 0, 0.025, 0, 0, 0, 1]

# Create graph
graph = Graph("graph", robot, problem)
state_placement = graph.createState("placement", False, 0)

problem.pathValidation = Dichotomy(robot, 0)
problem.pathProjector = ProgressiveProjector(
    problem.distance(), problem.steeringMethod(), 0.01
)
graph.initialize()
q1 = np.array(q1)
robot.currentConfiguration(q1)

# Project initial configuration on state 'placement'
res, q_init, error = graph.applyStateConstraints(state_placement, q1)
q2 = q1[::].copy()
q2[7] = 0.2

res, q_goal, error = graph.applyStateConstraints(state_placement, q2)

# Define manipulation planning problem
problem.initConfig(q_init)
problem.addGoalConfig(q_goal)
problem.constraintGraph(graph)

manipulationPlanner = ManipulationPlanner(problem)
manipulationPlanner.maxIterations(5000)
# manipulationPlanner.solve()
# v = Viewer (robot)
# v.playPath (v)
