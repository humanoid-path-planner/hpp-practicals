class MotionPlanner:
    def __init__(self, robot, problem, roadmap):
        self.robot = robot
        self.problem = problem
        self.roadmap = roadmap

    def solveBiRRT(self, maxIter=float("inf")):
        print("Method solveBiRRT is not implemented yet")
        finished = False

        # In the framework of the course,
        # we restrict ourselves to 2 connected components.
        nbCC = self.roadmap.numberConnectedComponents()
        if nbCC != 2:
            raise Exception("There should be 2 connected components.")

        iter = 0
        while True:
            # RRT begin
            # write your algorithm here
            # RRT end
            # Check if the problem is solved.
            # Check if problem is solved
            nbCC = self.roadmap.numberConnectedComponents()
            if nbCC == 1:
                print("Problem solved!")
                finished = True
                break
            iter = iter + 1
            if iter > maxIter:
                break
        if finished:
            path = self.problem.target().computePath(self.roadmap)
            return path
