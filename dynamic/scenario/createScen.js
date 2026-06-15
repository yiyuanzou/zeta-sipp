const { SIPP } = require('../algorithm/pathfinding/sipp');
const { SIPP_k } = require('../algorithm/pathfinding/sipp-k');
const { AASIPP } = require('../algorithm/pathfinding/aa-sipp');
const { TOAASIPP } = require('../algorithm/pathfinding/to-aa-sipp');
const { TOAAFoVSIPP } = require('../algorithm/pathfinding/to-aa-fov-sipp');
const { ZetastarSIPP_i } = require('../algorithm/pathfinding/zetastar-sipp_i');
const { ZetastarSIPP_f } = require('../algorithm/pathfinding/zetastar-sipp_f');

const { lineSegOnGrid } = require('../algorithm/line-of-sight');
const { getDist, unionAll } = require("../algorithm/function");
const fs = require('fs');

// The first N (N = 32, 64, 96, 128) pairs of starts and ends in each scenfile
// are dynamic obstacles, which builds a dynamic environment for benchmarking.
// The remaining pairs are the scenarios to be tested. We select the last 200 pairs in each scenfile.
function createScen(dynamicScen, scenarios, graph, dynamicObs, speed) {

    if (fs.existsSync(dynamicScen)) {
        const dynamicEnv = require(dynamicScen);
        for (let i = 0; i < dynamicObs; i++)
            updateSafeIntervals(graph, dynamicEnv[i].path, dynamicEnv[i].speed);
    }
    else {
        const dynamicEnv = [];
        for (let i = 0; i < dynamicObs; i++) {
            const scen = scenarios[i];
            graph.reset();
            const planner = initPlanner(graph, speed, "ZetastarSIPP_f");
            const start = [scen.start[0] + 0.5, scen.start[1] + 0.5];
            const end = [scen.end[0] + 0.5, scen.end[1] + 0.5];

            const path = planner.search(start, end);
            dynamicEnv.push({ id: i, speed: planner.speed, path: path.map(node => [node.x, node.y, node.waitTime]) });
            updateSafeIntervals(graph, dynamicEnv[i].path, dynamicEnv[i].speed);
        }
        fs.writeFileSync(dynamicScen, JSON.stringify(dynamicEnv));
    }
    return scenarios.slice(-200);
}

// Risk intervals are expanded with a buffer 
// to compensate for the omission of dynamic obstacle radii.
const timebuffer = 10;  // grid size / speed

function updateSafeIntervals(graph, path, speed) {
    let startTime = 0;
    for (let i = 1; i < path.length; i++) {
        const [p1, p2] = [path[i - 1], path[i]];

        // The start point of the path segment
        if (p2[2] > 0) { // waiting time > 0 
            const start = graph.cells[Math.floor(p1[0])][Math.floor(p1[1])];
            start.riskInterval = unionAll(start.riskInterval, [startTime, startTime + p2[2]]);
            startTime += p2[2];
        }

        // The grid cells traversed by the path segment
        const pathTime = getDist(p1.slice(0, -1), p2.slice(0, -1)) / speed;
        const traceGrids = lineSegOnGrid(p1[0], p1[1], p2[0], p2[1], pathTime);
        const intervals = traceGrids.map(grid => [grid.time[0] + startTime, grid.time[1] + startTime]);
        for (let j = 0; j < traceGrids.length; j++) {
            const cell = graph.cells[traceGrids[j].x][traceGrids[j].y];
            const riskIntervalNew = [Math.max(0, intervals[j][0] - timebuffer), intervals[j][1] + timebuffer];
            cell.riskInterval = unionAll(cell.riskInterval, riskIntervalNew);
        }
        startTime = intervals[intervals.length - 1][1];
    }
}

// Initialize the pathfinding planner
function initPlanner(graph, speed, algorithm) {
    switch (algorithm) {
        case "SIPP": return new SIPP(graph, speed);
        case "SIPP_16": return new SIPP_k(graph, speed, 4);
        case "SIPP_32": return new SIPP_k(graph, speed, 5);
        case "AASIPP": return new AASIPP(graph, speed);
        case "TOAASIPP": return new TOAASIPP(graph, speed);
        case "TOAAFoVSIPP": return new TOAAFoVSIPP(graph, speed);
        case "ZetastarSIPP_i": return new ZetastarSIPP_i(graph, speed);
        case "ZetastarSIPP_f": return new ZetastarSIPP_f(graph, speed);
    }
}

module.exports = { createScen };