class MotionPlanner:
    def __init__(self, robot, problem, roadmap):
        self.robot = robot
        self.problem = problem
        self.roadmap = roadmap

    def solveBiRRT(self, maxIter=float("inf")):
        finished = False
        iter = 0
        maxIter = 1000

        # Main RRT loop
        print("Method solveBiRRT is not implemented yet.")
        return None
        while not finished and iter < maxIter:
            iter += 1
            #### RRT begin
            #### RRT end
            # Check if problem is solved
            nbCC = self.roadmap.numberConnectedComponents()
            if nbCC == 1:
                print("Problem solved!")
                finished = True

        # Compute and display final path
        if finished:
            path = self.problem.target().computePath(self.roadmap)
            return path
        else:
            print(f"Maximum iterations ({maxIter}) reached without finding solution")


