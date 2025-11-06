from pyhpp.manipulation import Device, urdf, Graph, Problem, ProgressiveProjector, ManipulationPlanner
from pyhpp.core import ConfigurationShooter, Dichotomy  # noqa: F401
import numpy as np
from pinocchio import SE3, StdVec_Bool as Mask, Quaternion

from pyhpp.gepetto.viewer import Viewer
from hpp import Transform

from pyhpp.constraints import (
    RelativeTransformation,
    Transformation,
    ComparisonTypes,
    ComparisonType,
    Implicit,
)

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

problem = Problem(robot)

q1 = [0, -1.57, 1.57, 0, 0, 0, 0.3, 0, 0.025, 0, 0, 0, 1]

graph = Graph("graph", robot, problem)

# Create nodes and edges
#  Warning the order of the nodes is important. When checking in which node
#  a configuration lies, node constraints will be checked in the order of node
#  creation.
state_placement = graph.createState("placement", False, 0)
state_grasp = graph.createState("grasp", False, 0)

transition_transit = graph.createTransition(state_placement, state_placement, "transit", 1, state_placement)
transition_transfer = graph.createTransition(state_grasp, state_grasp, "transfer", 1, state_grasp)
transition_grasp_ball = graph.createTransition(state_placement, state_grasp, "grasp-ball", 1, state_placement)
transition_release_ball = graph.createTransition(state_grasp, state_placement, "release-ball", 1, state_grasp)


joint2 = robot.model().getJointId("pokeball/root_joint")
joint1 = robot.model().getJointId("ur5/wrist_3_joint")
Id = SE3.Identity()

m = [
    False,
    False,
    True,
    True,
    True,
    False,
]
q = Quaternion(0, 0, 0, 1)
ballGround = SE3(q, np.array([0, 0, 0.025]))
pc = Transformation(
    "placement_constraint", robot.asPinDevice(), joint2, Id, ballGround, m
)
cts = ComparisonTypes()
cts[:] = (
    ComparisonType.EqualToZero,
    ComparisonType.EqualToZero,
    ComparisonType.EqualToZero,
)
implicit_mask = [True, True, True]
placement_constraint = Implicit(pc, cts, implicit_mask)


m = [
    True,
    False,
    False,
    False,
    False,
    True,
]

pc = Transformation(
    "placement__complement_constraint", robot.asPinDevice(), joint2, Id, ballGround, m
)
cts = ComparisonTypes()
cts[:] = (
    ComparisonType.Equality,
    ComparisonType.Equality,
    ComparisonType.Equality,
)
implicit_mask = [True, True, True]
placement_complement_constraint = Implicit(pc, cts, implicit_mask)

# Create constraint of relative position of the ball in the gripper when ball
# is grasped
q = Quaternion(0.5, 0.5, -0.5, 0.5)
ballInGripper = SE3(q, np.array([0, 0.137, 0]))
m = Mask()
m[:] = (True,) * 6
pc = RelativeTransformation(
    "grasp", robot.asPinDevice(), joint1, joint2, ballInGripper, Id, m
)
cts = ComparisonTypes()
cts[:] = (
    ComparisonType.EqualToZero,
    ComparisonType.EqualToZero,
    ComparisonType.EqualToZero,
    ComparisonType.EqualToZero,
    ComparisonType.EqualToZero,
    ComparisonType.EqualToZero,
)
grasp_constraint = Implicit(pc, cts, m)

problem.setConstantRightHandSide(placement_constraint, True)
problem.setConstantRightHandSide(placement_complement_constraint, False)

# Set constraints of nodes and edges
graph.addNumericalConstraintsToState(state_placement, [placement_constraint])
graph.addNumericalConstraintsToState(state_grasp, [grasp_constraint])

graph.addNumericalConstraintsToTransition(transition_transit, [placement_complement_constraint])
graph.addNumericalConstraintsToTransition(transition_grasp_ball, [placement_complement_constraint])

# # These edges are in node 'grasp'
# graph.addConstraints(edge="transfer", constraints=Constraints())
# graph.addConstraints(edge="release-ball", constraints=Constraints())

problem.pathValidation = Dichotomy(robot.asPinDevice(), 0)
problem.pathProjector = ProgressiveProjector(
    problem.distance(), problem.steeringMethod(), 0.01
)
graph.initialize()

q1 = np.array(q1)
# Project initial configuration on state 'placement'
res, q_init, error = graph.applyStateConstraints(state_placement, q1)
q2 = q1[::]
q2[7] = 0.2

# Project goal configuration on state 'placement'
res, q_goal, error = graph.applyStateConstraints(state_placement, q2)

# Define manipulation planning problem
problem.initConfig(q_init)
problem.addGoalConfig(q_goal)
problem.constraintGraph(graph)

#v = Viewer (robot)
# v.playPath (v)


# Build relative position of the ball with respect to the gripper
for i in range(100):
    q = problem.configurationShooter().shoot()
    res, q3, err = graph.generateTargetConfig(transition_grasp_ball, q_init, q)
    configValid, report = problem.isConfigValid(q3)
    if res and configValid:
        break

if res:
    robot.currentConfiguration(q3)
    gripperPose = Transform(robot.asPinDevice().getJointPosition('ur5/wrist_3_joint'))
    ballPose = Transform(robot.asPinDevice().getJointPosition(ballName))
    gripperGraspsBall = gripperPose.inverse() * ballPose
    gripperAboveBall = Transform(gripperGraspsBall)
    gripperAboveBall.translation[2] += 0.1
