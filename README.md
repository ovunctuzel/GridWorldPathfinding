# GridWorldPathfinding
Code sample for pathfinding in a grid world

Full documentation at: https://ovunctuzel.github.io/GridWorldPathfinding/

This package contains two planners, a random walk algorithm and an optimal a* planner.

Both planners require three inputs: A world state, starting coordinates, and goal coordinates. 
The random planner can also be supplied with an additional argument `max_step_size` for termintating the algorithm after a given number of steps.


Planners are implemented as functions in the algorihms.py module, and they can be called in the following fashion:

```
planner_random(world_state, robot_pose, goal_pose, max_step_number=10000)

planner_optimal(world_state, robot_pose, goal_pose)
```

`world_state` is a 2D list of 1s and 0s, where 1s indicate occupied cells, and 0s indicate free cells.
`robot_pose` is a tuple of x,y coordinates.
`world_state` is a tuple of x,y coordinates.

Example
------------------

Simple example of using the planners:

```
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

    optimal_path = planner_optimal(world, start_pose, end_pose)
    print "Optimal path: ", optimal_path

    random_path = planner_random(world, start_pose, end_pose)
    print "Random path: ", random_path

```
Above code will print:

```
Optimal path:  [(3, 1), (2, 1), (2, 2), (2, 3), (2, 4), (3, 4), (4, 4), (4, 5), (4, 6), (5, 6), (6, 6), (7, 6), (7, 5), (7, 4), (8, 4), (8, 3), (8, 2), (7, 2), (7, 1), (6, 1), (5, 1)]
Random path:  [(3, 1), (3, 2), (2, 2), (2, 1), (2, 2), (2, 3), (1, 3), (1, 4), (2, 4), (3, 4), (4, 4), (4, 3), (4, 4), (4, 5), (3, 5), (3, 6), (4, 6), (5, 6), (6, 6), (7, 6), (7, 5), (8, 5), (8, 6), (9, 6), (9, 5), (9, 4), (8, 4), (8, 3), (8, 2), (7, 2), (7, 1), (6, 1), (5, 1)]
```
