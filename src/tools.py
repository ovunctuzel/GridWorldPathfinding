""" Module for helper functions for the planners. """

import math
import random


def sec_to_ms(secs):
    return secs * 1000.0


def within_bounds(world_state, pose):
    """ Returns true if pose is inside bounds of world_state """
    if pose[0] >= len(world_state) or pose[1] >= len(world_state[0]) or pose[0] < 0 or pose[1] < 0:
        return False
    else:
        return True


def get_orthogonal_nbors(world_state, pose):
    """ Returns a list of unoccupied orthogonal neighbors of a cell 'pose' """
    nbors = []
    # For each orthogonal direction
    for i, j in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            # Append cell to neighbor list if it is free
            if within_bounds(world_state, (pose[0]+i, pose[1]+j)) and world_state[pose[0]+i][pose[1]+j] == 0:
                nbors.append((pose[0]+i, pose[1]+j))
    return nbors


def get_octile_nbors(world_state, pose):
    """ Returns a list of unoccupied octile neighbors of a cell 'pose' """
    nbors = []
    # For each orthogonal direction
    for i, j in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]:
            # Append cell to neighbor list if it is free
            if within_bounds(world_state, (pose[0]+i, pose[1]+j)) and world_state[pose[0]+i][pose[1]+j] == 0:
                nbors.append((pose[0]+i, pose[1]+j))
    return nbors


def random_walk(world_state, pose):
    """ Returns tuple of a random valid neighboring cell to cell 'pose' """
    nbors = get_orthogonal_nbors(world_state, pose)
    return random.choice(nbors)


def random_walk_memory(world_state, pose, visited):
    """ Returns a random valid neighboring cell that is not recently visited.
        Can return visited cells if there is no other option """
    nbors = get_orthogonal_nbors(world_state, pose)
    # Get neighbors that aren't recently visited
    new_nbors = list(set(nbors) - set(visited))
    if new_nbors:
        return random.choice(new_nbors)
    else:
        # If all neighbors are recently visited, return a random neighbor
        return random.choice(nbors)


def manhattan_dist(pose1, pose2):
    """ Returns manhattan distance between two poses """
    return abs(pose1[0] - pose2[0]) + abs(pose1[1] - pose2[1])


def path_cost(pose1, pose2):
    return math.sqrt((pose1[0] - pose2[0]) ** 2 + (pose1[1] - pose2[1]) ** 2)


def path_length(path):
    """ Returns path length. Useful if neighborhoods other than orthogonal is allowed. """
    sum = 0
    for i in range(len(path)-1):
        sum += path_cost(path[i], path[i+1])
    return sum


def read_world_file(filename):
    """ Loads world from file, returns world_state as 2D list """
    f = open(filename, 'r')
    graph = f.readline()
    height = f.readline()[7:]
    width = f.readline()[6:]
    type = f.readline()
    world = []
    for i in range(int(height)):
        row = f.readline()
        world.append([0 if cell == '.' else 1 for cell in row])
    return world


def validate_input(world_state, start_pose, goal_pose):
    """ Returns false and prints error message if inputs to the planner are not valid - Returns true otherwise """
    if not within_bounds(world_state, start_pose):
        print "ERROR: Start pose is not within world bounds!"
        return False
    elif not within_bounds(world_state, goal_pose):
        print "ERROR: Goal pose is not within world bounds!"
        return False
    elif world_state[start_pose[0]][start_pose[1]] != 0:
        print "ERROR: Start pose is not free!"
        return False
    elif world_state[goal_pose[0]][goal_pose[1]] != 0:
        print "ERROR: Goal pose is not free!"
        return False
    else:
        return True
