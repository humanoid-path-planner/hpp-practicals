from grasp_ball import (  # noqa: F401
    graph,
    problem,
    q_goal,
    q_init,
    robot,
    transition_grasp_ball,
    v,
)

# Warning, this script is provided only as an example. The loop below never
# ends since all direct paths between q_init and q1 are in collision.
success = False
trial = 0
while not success:
    paths = list()
    print(f"trial {trial}")
    trial += 1
    q = problem.configurationShooter().shoot()
    res, q1, err = graph.generateTargetConfig(transition_grasp_ball, q_init, q)
    if not res:
        continue
    res, msg = problem.isConfigValid(q1)
    if not res:
        continue
    res, path, msg = problem.directPath(q_init, q1, True)
    paths.append(path)
    if not res:
        continue
    success = True
