const { AASIPP } = require("./aa-sipp");
const { lineOfSightSegOnGrid } = require("../line-of-sight");

// Safe-Interval Path Planning with 2^k-neighborhoods
class SIPP_k extends AASIPP {
    constructor(graph, speed, k) {
        super(graph, speed);
        this.k = k - 2;
    }

    // 2^k-neighborhoods forward expansion.
    forwardExpansion(currentNode, currentCell, endCell) {

        // Find all neighbor cells for the current cell.
        const neighborCells = this.findNeighborCells(Math.floor(currentCell.x), Math.floor(currentCell.y), this.k);

        for (const cell of neighborCells) {
            if (!cell.visited) this.visitCell(cell, endCell);
            if (cell.nodes.every(node => node.closed)) continue;

            // Cache the path time.
            const pathTime = this.euclidean(currentCell, cell) / this.speed;

            // Find which grid cells the line segment goes through, and compute the distances/times from the start to the intersections.
            const [isUnblocked, traceGrids] = lineOfSightSegOnGrid(currentCell.x, currentCell.y, cell.x, cell.y, pathTime, this.graph.cells);
            this.scannedNodes += traceGrids.length;
            if (!isUnblocked) continue;

            // Update the neighbor's parent.
            for (const node of cell.nodes) {
                if (node.closed) continue;
                this.updateParent(currentNode, node, pathTime, traceGrids);
            }
        }
    }

    // 2^k-neighborhoods
    findNeighborCells(cx, cy, k) {
        const width = this.graph.cells.length;
        const height = this.graph.cells[0].length;

        const x1 = Math.max(0, cx - k);
        const x2 = Math.min(width - 1, cx + k);
        const y1 = Math.max(0, cy - k);
        const y2 = Math.min(height - 1, cy + k);

        const neighborCells = [];
        for (let x = x1; x <= x2; x++) {
            for (let y = y1; y <= y2; y++) {
                if (x === cx && y === cy) continue;
                if (!this.graph.cells[x][y].weight) continue;
                neighborCells.push(this.graph.cells[x][y]);
            }
        }
        return neighborCells;
    }
}

module.exports = { SIPP_k };