""" Python module for path planning algorithms on a 2D grid world. """

import tools
import time
import math
import heapq


def planner_random(world_state, robot_pose, goal_pose, max_step_number=10000):
    """ Random walk path planner

    :param world_state: 2D matrix representing the world
    :param robot_pose: Tuple of robot pose
    :param goal_pose: Tuple of goal pose
    :param max_step_number: Max number of iterations before terminating
    :return: Returns path, a list of tuples of (x, y) coordinates - Returns None if there is an error

    """

    if not tools.validate_input(world_state, robot_pose, goal_pose):
        return None

    success = False
    iters = 0
    visited = []
    path = [robot_pose]

    # While goal not reached
    while iters < max_step_number:
        if robot_pose == goal_pose:
            success = True
            break
        iters += 1
        # Store current pose
        visited.append(robot_pose)
        # Keep length of visited cells constant
        if len(visited) > math.sqrt(max_step_number):
            visited.pop(0)
        robot_pose = tools.random_walk_memory(world_state, robot_pose, visited)
        path.append(robot_pose)

    if not success:
        print "ERROR: Failed to find a feasible path in %d steps" % max_step_number
        return None

    return path


def planner_optimal(world_state, robot_pose, goal_pose):
    """ Optimal A* path planner with manhattan distance heuristic. Assumes admissible heuristic.

    :param world_state: 2D matrix representing the world
    :param robot_pose: Tuple of robot pose
    :param goal_pose: Tuple of goal pose
    :return: Returns path, a list of tuples of (x, y) coordinates - Returns None if there is an error

    """

    def update_matrices():
        """ Updates costs and parents of the expanded node """
        f_cost_matrix[nbor[0]][nbor[1]] = f_cost
        g_cost_matrix[nbor[0]][nbor[1]] = g_cost
        parent_matrix[nbor[0]][nbor[1]] = current

    if not tools.validate_input(world_state, robot_pose, goal_pose):
        return None

    success = False
    # Initialize open (frontier) and closed lists
    frontier = []
    closed = set()
    # Initialize cost matrices
    # g_cost of a node is the cost of traveling from start to that node
    # f_cost = heuristic cost to goal + g_cost
    g_cost_matrix = [[float('inf') for i in range(len(world_state[0]))] for j in range(len(world_state))]
    f_cost_matrix = [[float('inf') for i in range(len(world_state[0]))] for j in range(len(world_state))]
    parent_matrix = [[(-1, -1) for i in range(len(world_state[0]))] for j in range(len(world_state))]
    g_cost_matrix[robot_pose[0]][robot_pose[1]] = 0
    f_cost_matrix[robot_pose[0]][robot_pose[1]] = tools.manhattan_dist(robot_pose, goal_pose)

    heapq.heappush(frontier, (0, robot_pose))

    # While frontier is not empty
    while frontier:
        # Expand most promising node in frontier
        current = heapq.heappop(frontier)[1]
        # Terminate if the goal is reached
        if current == goal_pose:
            success = True
            break
        # Add expanded node to closed list
        closed.add(current)
        # For each neighbor of node being expanded
        for nbor in tools.get_orthogonal_nbors(world_state, current):
            # Heuristic is manhattan distance, path_cost can be replaced by 1 for increased efficiency
            g_cost = g_cost_matrix[current[0]][current[1]] + tools.path_cost(current, nbor)
            f_cost = g_cost + tools.manhattan_dist(nbor, goal_pose)
            # If a node has already been expanded, skip it
            if nbor in closed:
                continue
            # If node does not exist in frontier, push it to frontier and update costs and parent
            if nbor not in [n[1] for n in frontier]:
                heapq.heappush(frontier, (f_cost, nbor))
                update_matrices()
            # If node exists in frontier, update costs and parents if it is more promising
            elif g_cost < g_cost_matrix[nbor[0]][nbor[1]]:
                del frontier[[n[1] for n in frontier].index(nbor)]
                heapq.heappush(frontier, (f_cost, nbor))
                update_matrices()

    if not success:
        print "ERROR: Failed to find a feasible path"
        return None

    path = [goal_pose]
    # Construct final path from parent_matrix
    while goal_pose != robot_pose:
        goal_pose = parent_matrix[goal_pose[0]][goal_pose[1]]
        path.append(goal_pose)
    path.reverse()

    return path


if __name__ == '__main__':
    # Simple example for planner usage
    # User can adjust parameters world, start_pose, and goal_pose to get results on planners
    world = [[1, 1, 1, 0, 0, 0, 0],
             [1, 1, 1, 0, 0, 1, 0],
             [1, 0, 0, 0, 0, 1, 1],
             [1, 0, 0, 1, 0, 0, 0],
             [1, 1, 1, 0, 0, 0, 0],
             [1, 0, 1, 1, 1, 1, 0],
             [1, 0, 1, 0, 0, 1, 0],
             [1, 0, 0, 1, 0, 0, 0],
             [1, 0, 0, 0, 0, 0, 0],
             [1, 1, 1, 1, 0, 0, 0]]

    start_pose = (3, 1)
    end_pose = (5, 1)
    t = time.time()
    optimal_path = planner_optimal(world, start_pose, end_pose)
    print "Time elapsed: ", tools.sec_to_ms(time.time() - t), "ms"
    print "Optimal path: ", optimal_path
    print "Length: ", tools.path_length(optimal_path)

    t = time.time()
    random_path = planner_random(world, start_pose, end_pose)
    print "Time elapsed: ", tools.sec_to_ms(time.time() - t), "ms"
    print "Random path: ", random_path
    print "Length: ", tools.path_length(random_path)
