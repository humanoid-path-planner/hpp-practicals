from hpp.corbaserver import loadServerPlugin
from hpp.corbaserver.manipulation import Client, ProblemSolver
from hpp.corbaserver.practicals.manipulation.ur5 import Robot
from hpp.gepetto import PathPlayer  # noqa: F401
from hpp.gepetto.manipulation import ViewerFactory

loadServerPlugin("corbaserver", "manipulation-corba.so")
Client().problem.resetProblem()

Robot.urdfName = "ur5_gripper"
Robot.urdfSuffix = ""
Robot.srdfSuffix = ""


class Pokeball:
    rootJointType = "freeflyer"
    packageName = "hpp_practicals"
    meshPackageName = "hpp_practicals"
    urdfName = "ur_benchmark/pokeball"
    urdfSuffix = ""
    srdfSuffix = ""


class Ground:
    rootJointType = "anchor"
    packageName = "hpp_practicals"
    urdfName = "ur_benchmark/ground"
    meshPackageName = "hpp_practicals"
    urdfSuffix = ""
    srdfSuffix = ""


class Box:
    rootJointType = "anchor"
    packageName = "hpp_practicals"
    urdfName = "ur_benchmark/box"
    meshPackageName = "hpp_practicals"
    urdfSuffix = ""
    srdfSuffix = ""


robot = Robot("ur5-pokeball", "ur5")
ps = ProblemSolver(robot)
ps.setErrorThreshold(1e-4)
ps.setMaxIterProjection(40)

vf = ViewerFactory(ps)
gripperName = "ur5/wrist_3_joint"
ballName = "pokeball/root_joint"
