from pyhpp.manipulation import Device, urdf, Graph, Problem, ProgressiveProjector, ManipulationPlanner
from pyhpp.core import ConfigurationShooter, Dichotomy  # noqa: F401
import numpy as np
from pinocchio import SE3, StdVec_Bool as Mask, Quaternion

from pyhpp.gepetto.viewer import Viewer


urdf_ur5 = (
    "package://example-robot-data/robots/ur_description/urdf/ur5_gripper.urdf"
)
srdf_ur5 = (
    "package://example-robot-data/robots/ur_description/srdf/ur5_gripper.srdf"
)

urdf_ball = "package://hpp_environments/urdf/ur_benchmark/pokeball.urdf"
srdf_ball = "package://hpp_environments/srdf/ur_benchmark/pokeball.srdf"

urdf_ground = "package://hpp_practicals/urdf/ur_benchmark/ground.urdf"
srdf_ground = "package://hpp_practicals/srdf/ur_benchmark/ground.srdf"

robot = Device("bot")

urdf.loadModel(robot, 0, "ur5", "anchor", urdf_ur5, srdf_ur5, SE3.Identity())
urdf.loadModel(
    robot, 0, "pokeball", "freeflyer", urdf_ball, srdf_ball, SE3.Identity()
)

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

urdfFilenameBox = "package://hpp_environments/urdf/ur_benchmark/box.urdf"
srdfFilenameBox = "package://hpp_environments/srdf/ur_benchmark/box.srdf"

urdf.loadModel(
    robot, 0, "box", "anchor", urdfFilenameBox, srdfFilenameBox, SE3.Identity()
)

model = robot.asPinDevice().model()
data = robot.asPinDevice().data()

obj = ["box/base_link_0", "box/base_link_1", "box/base_link_2", "box/base_link_3"]
positions = [
    [0.3 + 0.04, 0, 0.04],
    [0.3 - 0.04, 0, 0.04],
    [0.3, 0.04, 0.04],
    [0.3, -0.04, 0.04]
]
for collision in robot.asPinDevice().geomModel().geometryObjects:
    if collision.name in obj:
        collision.placement = SE3(np.eye(3), np.array(positions[obj.index(collision.name)]))
for visual in robot.asPinDevice().visualModel().geometryObjects:
    if visual.name in obj:
        visual.placement = SE3(np.eye(3), np.array(positions[obj.index(visual.name)]))

problem = Problem(robot)

q1 = [0, -1.57, 1.57, 0, 0, 0, 0.3, 0, 0.025, 0, 0, 0, 1]

# Create graph
graph = Graph("graph", robot, problem)
state_placement = graph.createState("placement", False, 0)

problem.pathValidation = Dichotomy(robot.asPinDevice(), 0)
problem.pathProjector = ProgressiveProjector(
    problem.distance(), problem.steeringMethod(), 0.01
)
graph.initialize()
q1 = np.array(q1)
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
#v = Viewer (robot)
# v.playPath (v)
