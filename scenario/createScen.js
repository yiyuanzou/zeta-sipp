const { TOAASIPP } = require('../algorithm/pathfinding/to-aa-sipp');
const { TOAAFoVSIPP } = require('../algorithm/pathfinding/to-aa-fov-sipp');
const { ZetaSIPP } = require('../algorithm/pathfinding/zeta-sipp');
const { ZetastarSIPP } = require('../algorithm/pathfinding/zetastar-sipp');
const { lineSegOnGrid, getDist, unionAll } = require("../algorithm/function/function");
const fs = require('fs');

// The first N (N = 32, 64, 96, 128) pairs of starts and ends in each scenfile
// are dynamic obstacles, which builds a dynamic environment for benchmarking.
// The remaining pairs are the scenarios to be tested. We select the last 20 pairs in each scenfile.
function createScen(dynamicScen, scenarios, mesh, dynamicObs) {

    if (fs.existsSync(dynamicScen)) {
        const dynamicEnv = require(dynamicScen);
        for (let i = 0; i < dynamicObs; i++)
            updateSafeIntervals(mesh, dynamicEnv[i].path, dynamicEnv[i].speed);
    }
    else {
        const dynamicEnv = [];
        for (let i = 0; i < dynamicObs; i++) {
            const scen = scenarios[i];
            const planner = initPlanner(mesh, "ZetastarSIPP");
            const start = [scen.start[0] + 0.5, scen.start[1] + 0.5];
            const end = [scen.end[0] + 0.5, scen.end[1] + 0.5];

            const path = planner.search(start, end);
            dynamicEnv.push({ id: i, speed: planner.speed, path: path.map(node => [node.x, node.y, node.waitTime]) });
            updateSafeIntervals(mesh, dynamicEnv[i].path, dynamicEnv[i].speed);
        }
        fs.writeFileSync(dynamicScen, JSON.stringify(dynamicEnv));
    }
    return scenarios.slice(-20);
}

function updateSafeIntervals(mesh, path, speed) {
    let startTime = 0;
    for (let i = 1; i < path.length; i++) {
        const [p1, p2] = [path[i - 1], path[i]];

        // Start grid
        if (p2[2] > 0) {
            const start = mesh[Math.floor(p1[0])][Math.floor(p1[1])];
            start.riskInterval = unionAll(start.riskInterval, [startTime, startTime + p2[2]]);
            startTime += p2[2];
        }

        // Trace grids
        const distance = getDist(p1.slice(0, -1), p2.slice(0, -1));
        const traceGrids = lineSegOnGrid(p1[0], p1[1], p2[0], p2[1], distance);
        const intervals = traceGrids.map(grid => [grid.dist[0] / speed + startTime, grid.dist[1] / speed + startTime]);
        for (let j = 0; j < traceGrids.length; j++) {
            const grid = mesh[traceGrids[j].x][traceGrids[j].y];
            grid.riskInterval = unionAll(grid.riskInterval, intervals[j]);
        }
        startTime = intervals[intervals.length - 1][1];
    }
}

// Initialize the pathfinding planner
function initPlanner(mesh, algorithm) {
    const speed = 0.1;
    switch (algorithm) {
        case "TOAASIPP": return new TOAASIPP(mesh, speed, true);
        case "TOAAFoVSIPP": return new TOAAFoVSIPP(mesh, speed, true);
        case "ZetaSIPP": return new ZetaSIPP(mesh, speed, true);
        case "ZetastarSIPP": return new ZetastarSIPP(mesh, speed, true);
    }
}

module.exports = { createScen };