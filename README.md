Zeta*-SIPP
======================================================================

## Description

Zeta*-SIPP is a time-optimal any-angle path-planning algorithm designed to identify conflict-free paths on 2D grids with dynamic obstacles. It improves Time-Optimal Any-Angle Safe Interval Path Planning (TO-AA-SIPP) by means of 1) reducing useless search nodes that have no contribution to finding optimal solutions, and 2) introducing Field of View (FoV) instead of Line of Sight (LoS) to speed up visibility checks with static obstacles. 

Further details are available in the IJCAI-24 paper titled "Zeta*-SIPP: Improved Time-Optimal Any-Angle Safe-Interval Path Planning".

The Zeta*-SIPP demo is available at http://dronectr.tudelft.nl/, using the ID: Zeta*. This project is a side product of our research on Unmanned Air Traffic Management (UTM). If you are interested, feel free to visit our UTM simulator, DroneCTR, with the ID: demo.

## Code Structure

| Name      | Description                                  |
| --------- | -------------------------------------------- |
| algorithm | Pathfinding and relevant functions.          |
| dataset   | Pathfinding benchmarks.                      |
| scenario  | Dynamic scenario generation for pathfinding. |
| benchmark | The main file for benchmark testing.         |
| output    | Generated only when executing "benchmark.js" |

For benchmark testing, you can directly run the code of "benchmark.js". For example, 

    node benchmark.js

Since the code is written in JavaScript, you have to install Node.js at first.