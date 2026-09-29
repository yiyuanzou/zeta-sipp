// Shared any-angle transition timing for AA-SIPP and TO-AA-SIPP families.
// Planner supplies graph, timebuffer, sippGrids, and updateWaitTime.

// Endpoint constraints shared by optimistic bounds and exact transitions.
// A zero buffer gives a lower bound; exact transitions use planner.timebuffer.
function calEndpointWaitTime(node1, node2, pathTime, timeInFirstGrid, timeInFinalGrid, timebuffer) {
    const minTimeIn = node1.g + pathTime - timeInFinalGrid;
    const maxWaitTime1 = node1.safeInterval[1] - node1.g - timeInFirstGrid;
    const maxWaitTime2 = node2.safeInterval[1] - node1.g - pathTime;
    const maxWaitTime = Math.min(maxWaitTime1, maxWaitTime2);
    let waitTime = 0;

    if (minTimeIn <= node2.safeInterval[0])
        waitTime = node2.safeInterval[0] - minTimeIn + timebuffer;
    if (maxWaitTime <= 0 || waitTime >= maxWaitTime) waitTime = Infinity;

    return { waitTime, maxWaitTime };
}

// Optimistic arrival time, before tracing the segment or checking middle cells.
function calArrivalTimeLowerBound(planner, node1, node2) {
    const dx = node2.x - node1.x, dy = node2.y - node1.y;
    const maxGridDistance = Math.max(Math.abs(dx), Math.abs(dy));
    if (!maxGridDistance) return Infinity;

    const pathTime = Math.sqrt(dx * dx + dy * dy) / planner.speed;
    // Cell-center endpoints have equal residence times, computed without a trace.
    const timeInFirstGrid = pathTime / (2 * maxGridDistance);
    const { waitTime } = calEndpointWaitTime(node1, node2, pathTime, timeInFirstGrid, timeInFirstGrid, 0);
    if (waitTime === Infinity) return Infinity;

    const minTimeArr = node1.g + waitTime + pathTime;
    // node2.g is a verified feasible arrival, not another candidate lower bound.
    if (minTimeArr > node2.g + 1e-9) return Infinity;

    // Relax rounding only in the optimistic bound, not in collision checks.
    return Math.max(node1.g + pathTime, node2.safeInterval[0], minTimeArr - 1e-10);
}

// Calculate the waiting time required for a collision-free transition from node1 to node2.
function calWaitTime(planner, node1, node2, pathTime, traceGrids) {
    let timeInFirstGrid, timeInFinalGrid;
    if (traceGrids.length) {
        timeInFirstGrid = traceGrids[0].time[1];
        timeInFinalGrid = traceGrids[traceGrids.length - 1].time[1] - traceGrids[traceGrids.length - 1].time[0];
    }
    else {
        const pathTimeHalf = pathTime / 2;
        timeInFirstGrid = pathTimeHalf;
        timeInFinalGrid = pathTimeHalf;
    }
    let { waitTime, maxWaitTime } = calEndpointWaitTime(node1, node2, pathTime,
        timeInFirstGrid, timeInFinalGrid, planner.timebuffer);
    if (waitTime === Infinity) return Infinity;

    // No need to check the safe intervals of node1 and node2 again.
    const middleCount = Math.max(0, traceGrids.length - 2);
    planner.sippGrids += middleCount + 2;

    // No grid cell between node1 and node2.
    if (!middleCount) return waitTime;

    // Keep trace times relative: waiting changes only the scalar offset.
    const departure = node1.g;

    // Safe interval-based conflict detection and resolution.
    for (let i = 1; i <= middleCount; i++) {
        waitTime = planner.updateWaitTime(traceGrids, departure, i, waitTime, maxWaitTime);
        if (waitTime === Infinity) return Infinity;
    }
    return waitTime;
}

// Compute the waiting time for conflict resolution.
// Entry/exit times stay relative to the source's arrival time.
function updateWaitTime(planner, traceGrids, departure, index, waitTime, maxWaitTime) {
    const waitTimeOld = waitTime;
    const grid = traceGrids[index];
    const cell = planner.graph.cells[grid.x][grid.y];

    // Risk intervals should be ordered.
    const risks = cell.riskInterval;
    for (let k = 0; k < risks.length; k += 2) {

        // Check if there is any conflict.
        const timeIn = grid.time[0] + departure + waitTime;
        const timeOut = grid.time[1] + departure + waitTime;
        if (risks[k] > timeOut || timeIn > risks[k + 1]) continue;

        // Compute the delta of waiting time.
        const delta = risks[k + 1] - timeIn + planner.timebuffer;
        waitTime += delta;
        if (waitTime >= maxWaitTime) return Infinity;
    }

    if (waitTime > waitTimeOld) {
        // Update the waiting time recursively.
        for (let i = 1; i <= index; i++) {
            waitTime = planner.updateWaitTime(traceGrids, departure, i, waitTime, maxWaitTime);
            if (waitTime === Infinity) return Infinity;
        }
    }
    return waitTime;
}

module.exports = { calArrivalTimeLowerBound, calWaitTime, updateWaitTime };
