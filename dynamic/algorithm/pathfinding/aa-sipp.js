const { intersectSets } = require("../function");
const { lineOfSightSegOnGrid } = require("../line-of-sight");
const { SIPP } = require("./sipp");

// Any-Angle Safe-Interval Path Planning
class AASIPP extends SIPP {
    constructor(graph, speed) {
        super(graph, speed);
    }

    heuristic(node1, node2) {
        return this.euclidean(node1, node2) / this.speed;
    }

    // Grid-by-grid forward expansion.
    forwardExpansion(currentNode, currentCell, endCell) {

        // Find all neighbor cells for the current cell.
        const neighborCells = this.findNeighborCells(currentCell);
        const previousCell = currentNode.parent ? this.graph.getCellFromNode(currentNode.parent) : null;

        for (const cell of neighborCells) {
            if (!cell.visited) this.visitCell(cell, endCell);
            if (cell.nodes.every(node => node.closed)) continue;

            // Current cell
            const potentialParents = [{
                cell: currentCell, node: currentNode, traceGrids: [],
                pathTime: this.euclidean(currentCell, cell) / this.speed
            }];

            // Line-of-sight checks.
            if (previousCell) {

                // Cache the path time.
                const pathTime = this.euclidean(previousCell, cell) / this.speed;

                // Find which grid cells the line segment goes through, and compute the distances/times from the start to the intersections.
                const [isUnblocked, traceGrids] = lineOfSightSegOnGrid(previousCell.x, previousCell.y, cell.x, cell.y, pathTime, this.graph.cells);
                this.scannedNodes += traceGrids.length;

                // The parent of the current node is also a potential parent for cell.nodes.
                if (isUnblocked) potentialParents.push({ cell: previousCell, node: currentNode.parent, traceGrids: traceGrids, pathTime: pathTime });
            }

            // Update the neighbor's parent.
            for (const parent of potentialParents) {
                for (const node of cell.nodes) {
                    if (node.closed) continue;
                    this.updateParent(parent.node, node, parent.pathTime, parent.traceGrids);
                }
            }
        }
    }

    // Calculate the waiting time required for a collision-free transition from node1 to node2.
    calWaitTime(node1, node2, pathTime, traceGrids) {
        let timeInFirstGrid, timeInFinalGird;
        if (traceGrids.length) {
            timeInFirstGrid = traceGrids[0].time[1];
            timeInFinalGird = traceGrids[traceGrids.length - 1].time[1] - traceGrids[traceGrids.length - 1].time[0];
        }
        else {
            const pathTimeHalf = pathTime / 2;
            timeInFirstGrid = pathTimeHalf;
            timeInFinalGird = pathTimeHalf;
        }
        const minTimeIn = node1.g + pathTime - timeInFinalGird;

        const maxWaitTime1 = node1.safeInterval[1] - node1.g - timeInFirstGrid;
        const maxWaitTime2 = node2.safeInterval[1] - node1.g - pathTime;
        const maxWaitTime = Math.min(maxWaitTime1, maxWaitTime2);
        let waitTime = 0;

        // Check if node2 is reachable.
        if (maxWaitTime <= 0) return Infinity;

        if (minTimeIn <= node2.safeInterval[0])
            waitTime = node2.safeInterval[0] - minTimeIn + this.timebuffer;
        if (waitTime >= maxWaitTime) return Infinity;

        // No need to check the safe intervals of node1 and node2 again.
        const middleGrids = traceGrids.slice(1, -1);
        this.sippGrids += middleGrids.length + 2;

        // No grid cell between node1 and node2.
        if (!middleGrids.length) return waitTime;

        // Compute the intervals occupied by this line segment.
        const intervals = middleGrids.map(grid => [grid.time[0] + node1.g + waitTime, grid.time[1] + node1.g + waitTime]);

        // Safe interval-based conflict detection and resolution.
        for (let i = 0; i < middleGrids.length; i++) {
            waitTime = this.updateWaitTime(middleGrids, intervals, i, waitTime, maxWaitTime);
            if (waitTime === Infinity) return Infinity;
        }
        return waitTime;
    }

    // Compute the waiting time for conflict resolution.
    // Also, update the intervals occupied by the line segment.
    updateWaitTime(traceGrids, intervals, index, waitTime, maxWaitTime) {
        const waitTimeOld = waitTime;
        const grid = traceGrids[index];
        const cell = this.graph.cells[grid.x][grid.y];

        // Risk intervals should be ordered.
        for (let k = 0; k < cell.riskInterval.length; k += 2) {

            // Check if there is any conflict.
            const cellRiskInterval = [cell.riskInterval[k], cell.riskInterval[k + 1]];
            const conflictInterval = intersectSets(cellRiskInterval, intervals[index]);
            if (!conflictInterval.length) continue;

            // Compute the delta of waiting time.
            const delta = cellRiskInterval[1] - intervals[index][0] + this.timebuffer;
            waitTime += delta;
            if (waitTime >= maxWaitTime) return Infinity;
            intervals.forEach(intv => { intv[0] += delta; intv[1] += delta; });
        }

        if (waitTime > waitTimeOld) {
            // Update the waiting time recursively.
            for (let i = 0; i < index; i++) {
                waitTime = this.updateWaitTime(traceGrids, intervals, i, waitTime, maxWaitTime);
                if (waitTime === Infinity) return Infinity;
            }
        }
        return waitTime;
    }
}

module.exports = { AASIPP };