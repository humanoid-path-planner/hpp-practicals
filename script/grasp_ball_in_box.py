import numpy as np
from pinocchio import SE3, Quaternion
from pyhpp.constraints import (
    ComparisonType,  # noqa
    ComparisonTypes,  # noqa
    Implicit,  # noqa
    RelativeTransformation,  # noqa
    Transformation,  # noqa
)
from pyhpp.core import ConfigurationShooter, Discretized  # noqa: F401
from pyhpp.viser import Viewer  # noqa
from pyhpp.manipulation import (
    Device,
    Graph,
    ManipulationPlanner,
    Problem,
    ProgressiveProjector,
    urdf,
)

state_placement = None  # please linters

urdf_ur5 = "package://example-robot-data/robots/ur_description/urdf/ur5_gripper.urdf"
srdf_ur5 = "package://example-robot-data/robots/ur_description/srdf/ur5_gripper.srdf"

urdf_ball = "package://hpp_practicals/urdf/ur_benchmark/pokeball.urdf"
srdf_ball = "package://hpp_practicals/srdf/ur_benchmark/pokeball.srdf"

urdf_ground = "package://hpp_practicals/urdf/ur_benchmark/ground.urdf"
srdf_ground = "package://hpp_practicals/srdf/ur_benchmark/ground.srdf"

urdf_box = "package://hpp_practicals/urdf/ur_benchmark/box.urdf"
srdf_box = "package://hpp_practicals/srdf/ur_benchmark/box.srdf"

robot = Device("bot")

urdf.loadModel(robot, 0, "ur5", "anchor", urdf_ur5, srdf_ur5, SE3.Identity())
urdf.loadModel(robot, 0, "pokeball", "freeflyer", urdf_ball, srdf_ball, SE3.Identity())
urdf.loadModel(robot, 0, "ground", "anchor", urdf_ground, srdf_ground, SE3.Identity())

box_pose = SE3(Quaternion(0, 0, 0, 1), np.array([0.3, 0, 0.04]))
urdf.loadModel(robot, 0, "box", "anchor", urdf_box, srdf_box, box_pose)

robot.setJointBounds(
    "pokeball/root_joint",
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

problem = Problem(robot)

graph = Graph("graph", robot, problem)
graph.errorThreshold(1e-4)
graph.maxIterations(40)

pokeball = robot.model().getJointId("pokeball/root_joint")
gripper = robot.model().getJointId("ur5/wrist_3_joint")
I_SE3 = SE3.Identity()

# Create nodes and edges
#  Warning the order of the nodes is important. When checking in which node
#  a configuration lies, node constraints will be checked in the order of node
#  creation.


# Create constraints


# Set constraints of nodes and edges


problem.pathValidation(Discretized(robot, 0.01))
problem.pathProjector(
    ProgressiveProjector(problem.distance(), problem.steeringMethod(), 0.1)
)
graph.initialize()

q1 = np.array([0, -1.57, 1.57, 0, 0, 0, 0.3, 0, 0.025, 0, 0, 0, 1])
robot.currentConfiguration(q1)

# Project initial configuration on state 'placement'
res, q_init, error = graph.applyStateConstraints(state_placement, q1)
q2 = q1.copy()
q2[7] = 0.2

# Project goal configuration on state 'placement'
res, q_goal, error = graph.applyStateConstraints(state_placement, q2)

problem.initConfig(q_init)
problem.addGoalConfig(q_goal)
problem.constraintGraph(graph)

planner = ManipulationPlanner(problem)
# v = Viewer (robot)
# v.initViewer(open=True, loadModel=True)
