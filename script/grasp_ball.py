import numpy as np
from pinocchio import SE3, Quaternion
from pyhpp.constraints import (
    ComparisonType,
    ComparisonTypes,
    Implicit,
    RelativeTransformation,
    Transformation,
)
from pyhpp.core import ConfigurationShooter, Discretized  # noqa: F401
from pyhpp.manipulation import (
    Device,
    Graph,
    ManipulationPlanner,
    Problem,
    ProgressiveProjector,
    urdf,
)
from pyhpp.viser import Viewer  # noqa

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
state_grasp = graph.createState("grasp", False, 0)
state_placement = graph.createState("placement", False, 0)

transition_transit = graph.createTransition(
    state_placement, state_placement, "transit", 1, state_placement
)
transition_transfer = graph.createTransition(
    state_grasp, state_grasp, "transfer", 1, state_grasp
)
transition_grasp_ball = graph.createTransition(
    state_placement, state_grasp, "grasp-ball", 1, state_placement
)
transition_release_ball = graph.createTransition(
    state_grasp, state_placement, "release-ball", 1, state_grasp
)

# Create constraints
# Placement
mask = [
    False,
    False,
    True,
    True,
    True,
    False,
]
ballOnGround = SE3(Quaternion(0, 0, 0, 1), np.array([0, 0, 0.025]))
function = Transformation(
    "placement_constraint", robot, pokeball, I_SE3, ballOnGround, mask
)
cts = ComparisonTypes()
cts[:] = 3 * (ComparisonType.EqualToZero,)
placement_constraint = Implicit(function, cts, [True, True, True])

# Placement complement
mask = [
    True,
    True,
    False,
    False,
    False,
    True,
]

function = Transformation(
    "placement_complement_constraint", robot, pokeball, I_SE3, ballOnGround, mask
)
cts[:] = 3 * (ComparisonType.Equality,)
placement_complement_constraint = Implicit(function, cts, [True, True, True])

# Grasp
ballInGripper = SE3(Quaternion(0.5, 0.5, -0.5, 0.5), np.array([0, 0.137, 0]))
mask = 6 * [
    True,
]
function = RelativeTransformation(
    "grasp", robot, gripper, pokeball, ballInGripper, I_SE3, mask
)
cts = ComparisonTypes()
cts[:] = 6 * (ComparisonType.EqualToZero,)
grasp_constraint = Implicit(function, cts, mask)

# Set constraints of nodes and edges

graph.addNumericalConstraintsToState(state_placement, [placement_constraint])
graph.addNumericalConstraintsToState(state_grasp, [grasp_constraint])
graph.addNumericalConstraintsToTransition(
    transition_transit, [placement_complement_constraint]
)
graph.addNumericalConstraintsToTransition(
    transition_grasp_ball, [placement_complement_constraint]
)

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
# v.loadPath(path)
