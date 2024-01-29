# Multi-Agent Path Finding in Python

## Introduction

This repository consists of the implementation of some multi-agent path-planning algorithms in Python. The following algorithms are currently implemented:

- [Introduction](#introduction)
- [Dependencies](#dependencies)
- [A\*](#a-*)
- [Prioritized Planning](#prioritized-planning)
- [Conflict Based Search](#conflict-based-search)
- [Conflict Based Search with Disjoint Splitting](#conflict-based-search-with-disjoint-splitting)

## Dependencies

Install the necessary dependencies by running.

```shell
python3 -m pip install -r requirements.txt
```

## A\*

The Independent MAPF solver finds paths for each agents at the same time, however collide can occur.

#### Execution

```
python run_experiments.py --instance instances/test_50.txt --solver Independent
```

#### Results sample

## Prioritized Planning

The prioritized MAPF solver finds paths for all agents, one after the other, that do not collide with the environment or the already planned paths of the other agents. However, it might not complete and optimal.

#### Execution

```
python run_experiments.py --instance instances/test_50.txt --solver Prioritized
```

#### Results sample

## Conflict Based Search

Conflict-Based Search (CBS) is slower than prioritized planning but complete and optimal.

#### Execution

```
python run_experiments.py --instance "instances/test_*" --solver CBS --batch
```

The batch command generates a results file named results.csv, to compare it with the one available in instances/min-sum-of-cost.csv.

#### Result sample

## Conflict Based Search with Disjoint Splitting

The main difference between Conflict-Based Search (CBS) and Conflict Based Search with Disjoint Splitting (CBS-DS) lies in the high-level search strategy. CBS addresses conflicts for all agents collectively, while CBS-DS introduces disjoint splitting to address conflicts independently in disjoint groups. The choice between CBS and CBS-DS may depend on the specific characteristics of the MAPF instance and the desired trade-offs between solution quality and computational efficiency. CBS-DS is designed to exploit structure in the MAPF problem to improve scalability and reduce computational complexity.

#### Execution

```
python run_experiments.py --instance instances/exp4.txt --solver CBS
```

#### Result sample

### Note

To find other scene situation, look into `instances/` folder and replace `[scene_to_test]` with the desired scene.

```
python run_experiments.py --instance instances/[scene_to_test].txt --solver CBS
```

### References

[J. Li, D. Harabor, P. Stuckey, A. Felner, H. Ma, and S. Koenig. Disjoint splitting for multi- agent path finding with conflict-based search. In Proceedings of the International Conference on Automated Planning and Scheduling, pages 279-283, 2019.](https://ojs.aaai.org/index.php/ICAPS/article/view/3487)
