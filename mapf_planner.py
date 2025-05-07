#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from visualize import Animation
from single_agent_planner import get_sum_of_cost
from itertools import permutations
import multiprocessing
import numpy as np
import matplotlib.pyplot as plt
import os

SOLVER = "CBS"

def compute_goal_matrix_list(goals, lamda):
    goal_perm_list = []
    for i in range(len(lamda)):
        idx_sort = np.argsort(np.abs(lamda[i]))
        permuted = [goals[idx] for idx in idx_sort]
        goal_perm_list.append(permuted)
    return goal_perm_list

def mapf_planner(lamda, my_map, starts, goals, disjoint=False):
    goal_perm_list = compute_goal_matrix_list(goals, lamda)
    costs = []
    for perm in goal_perm_list:
        cbs = CBSSolver(my_map, starts, perm)
        paths = cbs.find_solution(disjoint)
        cost = get_sum_of_cost(paths)
        costs.append(cost)
    return costs

def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    line = f.readline()
    num_agents = int(line)
    starts, goals = [], []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None)
    parser.add_argument('--batch', action='store_true', default=False)
    parser.add_argument('--disjoint', action='store_true', default=False)
    parser.add_argument('--solver', type=str, default=SOLVER)
    args = parser.parse_args()

    result_file = open("results_5.csv", "w", buffering=1)

    for file in sorted(glob.glob(args.instance)):
        print("***Import an instance***")
        my_map, starts, goals_permutation = import_mapf_instance(file)
        if args.solver == "CBS":
            print("***Run CBS with CEM Optimization***")

            # Example usage with mapf_planner inside CEM, retained for reference
            # best_perm, best_cost = cem_plan(my_map, starts, goals_permutation, args.disjoint)

        else:
            raise RuntimeError("Unknown solver!")

        if not args.batch:
            print("***Simulating Best Plan Found by CEM***")
            cbs = CBSSolver(my_map, starts, best_perm)
            paths = cbs.find_solution(args.disjoint)

            animation = Animation(my_map, starts, best_perm, paths)
            filename_only = os.path.basename(file)
            gif_filename = os.path.join("videos", filename_only.replace(".txt", "_animation.gif"))

            os.makedirs("videos", exist_ok=True)
            animation.save(gif_filename, time_step=1.0, writer='pillow')

    result_file.close()
