const { calWaitTime, updateWaitTime } = require("../waiting-time");
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

    // Share transition timing without coupling the two search algorithms.
    calWaitTime(node1, node2, pathTime, traceGrids) {
        return calWaitTime(this, node1, node2, pathTime, traceGrids);
    }

    updateWaitTime(traceGrids, departure, index, waitTime, maxWaitTime) {
        return updateWaitTime(this, traceGrids, departure, index, waitTime, maxWaitTime);
    }
}

module.exports = { AASIPP };
