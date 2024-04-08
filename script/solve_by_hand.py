from grasp_ball import graph, ps, q_goal, q_init, robot, vf  # noqa: F401

# Warning, this script is provided only as an example. The loop below never
# ends since all direct paths between q_init and q1 are in collision.
success = False
trial = 0
while not success:
    paths = list()
    print(f"trial {trial}")
    trial += 1
    q = robot.shootRandomConfig()
    res, q1, err = graph.generateTargetConfig("grasp-ball", q_init, q)
    if not res:
        continue
    res, msg = robot.isConfigValid(q1)
    if not res:
        continue
    res, pid, msg = ps.directPath(q_init, q1, True)
    paths.append(pid)
    if not res:
        continue
    success = True
