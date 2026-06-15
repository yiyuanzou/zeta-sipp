Zeta* and Zeta*-SIPP
======================================================================

## Description

Zeta* is a fast and optimal algorithm for any-angle path planning on grids with static obstacles. It is slightly slower than Anya but relatively simpler to implement.

Zeta*-SIPP extends Zeta* to handle dynamic obstacles, enabling it to compute time-optimal, conflict-free paths in dynamic environments. It is much slower than the suboptimal planners SIPP and AA-SIPP, but about 20× faster than the optimal planner TO-AA-SIPP. For real-time any-angle path planning, AA-SIPP is generally the better choice, whereas Zeta*-SIPP is recommended when optimal solutions are desired.

Zeta* and Zeta*-SIPP together provide a unified and scalable solution for optimal any-angle path planning on grids.

## Code Structure
The repository contains two folders: `static` and `dynamic`, which include benchmarking setups and algorithm implementations for static and dynamic environments, respectively. Each folder contains:

| Name      | Description                                                                                          |
| --------- | ---------------------------------------------------------------------------------------------------- |
| algorithm | Pathfinding algorithms and relevant functions.                                                       |
| dataset   | Example benchmark datasets from the [Moving AI Lab](https://www.movingai.com/benchmarks/index.html). |
| scenario  | Scenario generation for pathfinding experiments.                                                     |
| benchmark | Entry point for running benchmark tests.                                                             |
| output    | Generated automatically when executing `benchmark.js`.                                               |

Since the code is written in JavaScript, you need to install [Node.js](https://nodejs.org/en) first. To run the benchmark tests, execute:

    node benchmark.js

Two versions of Zeta* and Zeta*-SIPP are provided, distinguished by the suffixes `-i` and `-f`. The `-i` variant uses inverted scanning, while the `-f` variant adopts forward scanning. The `-f` variants (`Zeta*-f` and `Zeta*-SIPP-f`) are recommended, as they are generally faster and more memory-efficient.

Additional algorithms are also implemented in JavaScript for comparison, including A*, A* with 2<sup>k</sup> neighborhoods, Theta*, Anya, SIPP, SIPP with 2<sup>k</sup> neighborhoods, AA-SIPP, TO-AA-SIPP and TO-AA-FoV-SIPP.

Note that in the `static` folder, search nodes are located at grid corners, whereas in the `dynamic` folder, they are placed at grid centers.

## Publications
Further details are available in:

[1] Zou, Yiyuan, and Clark Borst. "Optimal Any-Angle Path Planning in Static and Dynamic Environments." (Under Review)

[2] Zou, Yiyuan, and Clark Borst. "[Zeta*-SIPP: Improved Time-Optimal Any-Angle Safe-Interval Path Planning](https://www.ijcai.org/proceedings/2024/754)." 33rd International Joint Conference on Artificial Intelligence, pp. 6823-6830. 2024.

## Demo
A demo is available at http://dronectr.tudelft.nl/, using the ID `zeta-sipp`.

Zeta*-SIPP has been applied in our research on Uncrewed Air Traffic Management (UTM). A demo is available at the same link using the ID `transparency`.