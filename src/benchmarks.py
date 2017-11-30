import tools
import random
import time
import algorithms
import numpy as np


def benchmark(problem_file, test_set_file):
    """ Evaluates planners with a random problem from a given problem set and world map

        :param problem_file: A string of map file with .map extension
        :param test_set_file: A string of problem set file with .scen extension
        :return: Returns a tuple of (results_optimal, results_random) where each element is a custom data structure
                 carrying calculated path, path length and time elapsed to calculate path.

    """
    class Results(object):
        def __init__(self, path, path_length, time_elapsed):
            self.path = path
            self.path_length = path_length
            self.time_elapsed = time_elapsed

    world = tools.read_world_file(problem_file)

    f = open(test_set_file, 'r')
    problems = f.readlines()
    # Pick random problem
    problem_str = problems[random.randint(1, len(problems) - 1)].split()
    # Parse problem string
    start_pose = int(problem_str[5]), int(problem_str[4])
    goal_pose = int(problem_str[7]), int(problem_str[6])

    # Evaluate optimal planner
    t = time.time()
    path = algorithms.planner_optimal(world, start_pose, goal_pose)
    time_ms = tools.sec_to_ms((time.time() - t))
    results_optimal = Results(path, tools.path_length(path), time_ms)

    # Evaluate random planner
    t = time.time()
    path = algorithms.planner_random(world, start_pose, goal_pose, max_step_number=100000)
    time_ms = tools.sec_to_ms((time.time() - t))
    results_random = Results(path, tools.path_length(path), time_ms)

    return results_optimal, results_random


def statistics(benchmarks):
    """ Prints statistical results for a given set of benchmarks.
        Reports mean and standard deviations on total path length and standard deviations """
    optimal_lengths = [bench[0].path_length for bench in benchmarks]
    optimal_time = [bench[0].time_elapsed for bench in benchmarks]
    random_lengths = [bench[1].path_length for bench in benchmarks]
    random_time = [bench[1].time_elapsed for bench in benchmarks]
    print "Random Planner Mean Path Length and Standard Deviation: ", np.mean(random_lengths), np.std(random_lengths)
    print "Random Planner Mean Elapsed Time and Standard Deviation: ", np.mean(random_time), np.std(random_time)
    print "Optimal Planner Mean Path Length and Standard Deviation: ", np.mean(optimal_lengths), np.std(optimal_lengths)
    print "Optimal Planner Mean Elapsed Time and Standard Deviation: ", np.mean(optimal_time), np.std(optimal_time)


if __name__ == '__main__':
    benchmarks = []
    for i in range(100):
        benchmarks.append(benchmark('benchmarks/dragonage.map', 'benchmarks/dragonage.map.scen'))
    statistics(benchmarks)
    benchmarks = []
