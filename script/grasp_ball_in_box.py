from hpp.corbaserver import Client  # noqa: F401
from hpp.corbaserver.manipulation import ConstraintGraph
from manipulation import (
    Box,
    Ground,
    PathPlayer,  # noqa: F401
    Pokeball,
    ps,
    robot,
    vf,
)

vf.loadEnvironmentModel(Ground, "ground")
vf.loadEnvironmentModel(Box, "box")
vf.moveObstacle("box/base_link_0", [0.3 + 0.04, 0, 0.04, 0, 0, 0, 1])
vf.moveObstacle("box/base_link_1", [0.3 - 0.04, 0, 0.04, 0, 0, 0, 1])
vf.moveObstacle("box/base_link_2", [0.3, 0.04, 0.04, 0, 0, 0, 1])
vf.moveObstacle("box/base_link_3", [0.3, -0.04, 0.04, 0, 0, 0, 1])

vf.loadObjectModel(Pokeball, "pokeball")
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

q1 = [0, -1.57, 1.57, 0, 0, 0, 0.3, 0, 0.025, 0, 0, 0, 1]

# Create graph
graph = ConstraintGraph(robot, "graph")


ps.selectPathValidation("Discretized", 0.01)
ps.selectPathProjector("Progressive", 0.1)
graph.initialize()

res, q_init, error = graph.applyNodeConstraints("placement", q1)
q2 = q1[::]
q2[7] = 0.2

res, q_goal, error = graph.applyNodeConstraints("placement", q2)

ps.setInitialConfig(q_init)
ps.addGoalConfig(q_goal)

# v = vf.createViewer ()
# pp = PathPlayer (v)
# v (q1)
