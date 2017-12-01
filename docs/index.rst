.. Pathfinding documentation master file, created by
   sphinx-quickstart on Thu Nov 30 11:38:30 2017.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Grid World Pathfinding Documentation
=======================================
`GitHub Repository <https://github.com/ovunctuzel/GridWorldPathfinding>`_

Requirements:
	- `numpy <http://www.numpy.org/>`_ (Only for benchmarks)


Planners
=======================================

.. automodule:: algorithms
	:members:

Planners in this module compute a path from start_pose to goal_pose given a world represented by a 2D list, with 1's denoting obstacles and 0's denoting free space. 

The random planner is implemented as a simple algorithm where random steps are taken from the start_pose until goal_pose is reached within ``max_step_number`` steps. The last ``sqrt(max_step_number)`` steps are stored in the list ``visited``, and if possible, the random walk refrains from visiting cells in this list. The planner returns a path, a list of coordinates to reach the ``goal_pose`` from the ``start_pose``, if successful. If a feasible path is not calculated in ``max_step_number`` steps, or the start or goal states are occupied, the planner will return ``None``.

The optimal planner is implemented as a A* Search algorithm. The planner returns a path if successful, and returns ``None`` if it fails. The heuristic is selected as the manhattan distance, which is proven to be admissible in orthogonally connected grid worlds. 

`More on A* Pathfinding Algorithm <https://en.wikipedia.org/wiki/A*_search_algorithm>`_


Simple Example
=======================================
::

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

The above code will print:

.. code-block:: none

	Time elapsed:  0.618934631348 ms
	Optimal path:  [(3, 1), (2, 1), (2, 2), (2, 3), (2, 4), (3, 4), (4, 4), (4, 5), (4, 6), (5, 6), (6, 6), (7, 6), (7, 5), (7, 4), (8, 4), (8, 3), (8, 2), (7, 2), (7, 1), (6, 1), (5, 1)]
	Length:  20.0
	Time elapsed:  0.396013259888 ms
	Random path:  [(3, 1), (3, 2), (2, 2), (2, 1), (2, 2), (2, 3), (1, 3), (1, 4), (2, 4), (3, 4), (4, 4), (4, 3), (4, 4), (4, 5), (3, 5), (3, 6), (4, 6), (5, 6), (6, 6), (7, 6), (7, 5), (8, 5), (8, 6), (9, 6), (9, 5), (9, 4), (8, 4), (8, 3), (8, 2), (7, 2), (7, 1), (6, 1), (5, 1)]
	Length:  32.0


Helper Tools
=======================================

.. automodule:: tools
	:members:


Benchmarks
=======================================

Benchmark Results
---------------------------------------

Benchmark results are obtained by selecting random problems from three hand picked problem sets from the Moving AI lab website. Problem sets and world files can be found in the benchmarks folder. Additional benchmarks can be downloaded at:

`MovingAI LAB <http://www.movingai.com/benchmarks/>`_

The problem sets are:

* `tiny.map.scen <http://www.movingai.com/benchmarks/dao/lak110d.jpg>`_ 21x30 map with relatively few open spaces. Similar to a generic indoor environment.
* `dragonage.map.scen <http://www.movingai.com/benchmarks/dao/lak103d.jpg>`_ 49x49 map ripped from the video game Dragon Age: Origins. Similar to a generic outdoor environment.
* `maze.map.scen <http://www.movingai.com/benchmarks/mazes1/maze512-1-0.jpg>`_ 512x512 map with very complex corridors of width 1. Artificial environment for testing.

Each problem set contains hundreds of problems with valid start and goal positions, and a world map. The world map representation is slightly different, with ``.`` denoting empty states. The world maps are converted to 1 and 0 representation with the :func:`tools.read_world_file()` function.

100 problems are picked randomly from each problem set, and path lengths and computation times are calculated. Means and standard deviations are reported for each problem set. 

.. image:: /img/table1.png

The A* algorithm can find the optimal path reasonably fast in all three problems. Even the complicated maze problem can be solved in about a second on average. 

.. image:: /img/table2.png

The random planner is only viable in tiny maps with narrow corridors. Compared to the A* algorithm, the random planner performed an order of magnitude worse. Even in the smallest map in the problem set, the mean path length is 357.88 compared to A*'s 14.92, and the computation speed is 7 times slower. The difference is even larger with the intermediate sized map, and the random planner completely fails to find a path in 100000 steps, thus, no results are provided on the random planner with the maze map.   

Code Correctness
---------------------------------------

Optimality of the A* Algorithm has been verified by selecting many problems from the problem sets and comparing the path lengths computed with the planner with the values reported in the sets. All tests were passed. Since the problem sets are originally 8-connected, octile neighborhoods were allowed when testing this way. 

Remarks on Complexity
---------------------------------------
The A* algorithm has a time and space complexity of O(b^d), where b is the branching factor, and d is the depth of the optimal path. The branching factor is 4 when the grid world is orthogonally connected. For configuration spaces with relatively few number of nodes, and static obstacles, A* is one of the most reliable algorithms when an admissible heuristic is used. 

The random walk algorithm is stochastic by its nature. There is no guarantee that the goal will be found in a certain number of steps. However, the algorithm is probabilistically complete. As the number of steps taken approaches infinity, the probability of finding a solution approaches 1. The worst case scenario of this algorithm is unbounded, however an expected time for covering all nodes in the graph can be stated. 

Let n be the number of nodes in the graph, and m be the number of connections. In an undirected graph, we would have a maximum of n(n-1) connections. In order to have an expected value of covering all the connections, we would have to visit each node n-1 times. Thus, the expected cover time would be n(n-1)(n-1), which can be stated as O(n^3). However, in an orthogonally connected grid world, the number of connections would be 4n, and the complexity can be stated as O(n^2).

Benchmarking Code
---------------------------------------
.. automodule:: benchmarks
	:members:

.. toctree::
