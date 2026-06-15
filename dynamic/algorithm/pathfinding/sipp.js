const { BinaryHeap } = require("../binaryHeap");

// Safe-Interval Path Planning
class SIPP {
    constructor(graph, speed) {
        this.graph = graph;                                         // 0: obstacle, 1: normal
        this.speed = speed;                                         // Time-optimal
        this.openHeap = new BinaryHeap(node => node.f);             // Open heap instead of open list.
        this.timebuffer = 1e-5;                                     // Add a small buffer for the waiting time

        this.treeNodes = 0;                                         // Number of tree nodes
        this.searchSteps = 0;                                       // Number of search steps
        this.scannedNodes = 0;                                      // Number of scanned nodes
        this.sippGrids = 0;                                         // Number of grids whose safe intervals are checked
        this.sortedElements = 0;                                    // Number of sorted elements (for Zeta*)
    }

    // Initialization
    init(start, end) {
        const startCell = this.graph.cells[Math.floor(start[0])][Math.floor(start[1])];
        const endCell = this.graph.cells[Math.floor(end[0])][Math.floor(end[1])];

        this.visitCell(startCell, endCell);
        const startNode = startCell.nodes[0];
        startNode.g = 0;
        startNode.f = startNode.h;
        this.openHeap.push(startNode);

        return endCell;
    }

    // Search the path
    search(start, end) {

        // Initialization
        const endCell = this.init(start, end);

        // Main loop
        while (this.openHeap.size() > 0) {

            // Get the node with the lowest f-value for next step. The heap keeps it sorted.
            const currentNode = this.openHeap.shift();
            this.searchSteps++;

            // Move currentNode from open to closed.
            currentNode.closed = true;
            const currentCell = this.graph.getCellFromNode(currentNode);

            // The target has been found.
            if (currentCell === endCell)
                return this.pathTo(currentNode);

            // Grid-by-grid forward expansion.
            this.forwardExpansion(currentNode, currentCell, endCell);
        }

        // Fail to find a path.
        return [];
    }

    // Grid-by-grid forward expansion.
    forwardExpansion(currentNode, currentCell, endCell) {

        // Find all neighbor cells for the current cell.
        const neighborCells = this.findNeighborCells(currentCell);
        this.scannedNodes += neighborCells.length;

        for (const cell of neighborCells) {
            if (!cell.visited) this.visitCell(cell, endCell);
            if (cell.nodes.every(node => node.closed)) continue;

            // Cache the path time.
            const pathTime = this.euclidean(currentCell, cell) / this.speed;

            // Update the neighbor's parent.
            for (const node of cell.nodes) {
                if (node.closed) continue;
                this.updateParent(currentNode, node, pathTime, []);
            }
        }
    }

    // Visit a cell
    visitCell(cell, endCell) {
        cell.visited = true;
        cell.h = this.heuristic(cell, endCell);
        cell.nodes.forEach(node => node.h = cell.h);
    }

    // 8-connected neighbors
    findNeighborCells(cell) {
        const neighbors = [];
        const x = Math.floor(cell.x);
        const y = Math.floor(cell.y);
        const cells = this.graph.cells;

        // Eight neighbors.
        if (cells[x - 1]) neighbors.push(cells[x - 1][y]);
        if (cells[x + 1]) neighbors.push(cells[x + 1][y]);
        if (cells[x]) neighbors.push(cells[x][y - 1]);
        if (cells[x]) neighbors.push(cells[x][y + 1]);
        if (cells[x - 1]) neighbors.push(cells[x - 1][y - 1]);
        if (cells[x + 1]) neighbors.push(cells[x + 1][y - 1]);
        if (cells[x - 1]) neighbors.push(cells[x - 1][y + 1]);
        if (cells[x + 1]) neighbors.push(cells[x + 1][y + 1]);

        return neighbors.filter(neighbor => neighbor && neighbor.weight && !this.isStumble(cell, neighbor));
    }

    // Prevent movement across the corner between two obstacles
    isStumble(cell, neighbor) {
        const x1 = Math.floor(cell.x);
        const y1 = Math.floor(cell.y);
        const x2 = Math.floor(neighbor.x);
        const y2 = Math.floor(neighbor.y);
        if (x1 === x2 || y1 === y2) return false;
        return !this.graph.cells[x1][y2].weight && !this.graph.cells[x2][y1].weight;
    }

    updateParent(parent, node, pathTime, traceGrids) {
        const waitTime = this.calWaitTime(parent, node, pathTime, traceGrids);
        const gScore = parent.g + waitTime + pathTime;
        const beenVisited = node.visited;

        if (gScore < node.g) {
            node.visited = true;
            node.parent = parent;
            node.g = gScore;
            node.f = node.g + node.h;
            node.waitTime = waitTime;  // node.waitTime is the wait time at node.parent before reaching the node.

            if (!beenVisited) {
                this.openHeap.push(node);
                this.treeNodes++;
            }
            else this.openHeap.sortElement(node);
        }
    }

    // Calculate the waiting time required for a collision-free transition from node1 to node2.
    calWaitTime(node1, node2, pathTime, _) {
        let waitTime = 0;
        const minTimeIn = pathTime / 2 + node1.g;
        if (minTimeIn <= node2.safeInterval[0])
            waitTime = node2.safeInterval[0] - minTimeIn + this.timebuffer;
        const startTime = waitTime + node1.g;
        const minTimeArr = pathTime + startTime;
        if (minTimeArr >= node2.safeInterval[1]) return Infinity;
        const maxTimeDep = pathTime / 2 + startTime;
        if (maxTimeDep >= node1.safeInterval[1]) return Infinity;
        this.sippGrids += 2;
        return waitTime;
    }

    // Traced path
    pathTo(node) {
        let curr = node;
        const path = [];
        while (curr) {
            path.unshift(curr);
            curr = curr.parent;
        }
        return path;
    }

    heuristic(node1, node2) {
        const d1 = Math.abs(node1.x - node2.x);
        const d2 = Math.abs(node1.y - node2.y);
        const distance = (d1 + d2) - 0.585787 * Math.min(d1, d2);
        return distance / this.speed;
    }

    euclidean(node1, node2) {
        const d1 = node2.x - node1.x;
        const d2 = node2.y - node1.y;
        return Math.sqrt(d1 * d1 + d2 * d2);
    }
}

class Cell {
    constructor(point, weight) {
        this.x = point[0];
        this.y = point[1];
        this.weight = weight;
        this.riskInterval = [];
        this.reset();
    }

    reset() {
        this.h = 0;
        this.fh = Infinity;
        this.visited = false;     // Indicate whether the heuristic has been computed.
        this.partClosed = false;  // Indicate whether the cell contains a closed node.
        this.closed = false;      // Indicate whether all nodes are closed.
        this.nodes = [];          // Store nodes on the grid cell.
        this.visibleCells = [];   // Record the visible cells from this cell.
    }
}

class Node {
    constructor(point) {
        this.x = point[0];
        this.y = point[1];
        this.f = Infinity;
        this.g = Infinity;
        this.gLow = Infinity;
        this.h = 0;
        this.visited = false;     // Indicate whether the node has been inserted into the open list.
        this.parent = null;
        this.closed = false;
        this.bestPotentialParent = null;
        this.potentialParents = [];
        this.gLowArray = [];

        // Note that the waitTime here indicates the time to wait at its bestPotentialParent.
        // Since a node has only one parent but maybe a lot of children.
        this.waitTime = 0;
        this.safeInterval = [];
    }
}

class Graph {
    constructor(grid) {
        this.grid = grid;                   // Storing grid
        this.cells = [];                    // 2D array for storing cells
        this.initCells();
    }

    // Initialize cells
    initCells() {
        for (let x = 0; x < this.grid.length; x++) {
            this.cells[x] = [];
            for (let y = 0; y < this.grid[x].length; y++) {
                const cell = new Cell([x + 0.5, y + 0.5], this.grid[x][y]);
                this.cells[x][y] = cell;
            }
        }
    }

    initNodes(cell) {
        const safeInterval = [0, ...cell.riskInterval, Infinity];
        for (let k = 0; k < safeInterval.length; k += 2) {
            const node = new Node([cell.x, cell.y]);
            node.safeInterval = [safeInterval[k], safeInterval[k + 1]];
            cell.nodes.push(node);
        }
    }

    // Reset the cells and nodes, excluding the riskInterval.
    reset() {
        for (let x = 0; x < this.cells.length; x++)
            for (let y = 0; y < this.cells[0].length; y++) {
                this.cells[x][y].reset();
                this.initNodes(this.cells[x][y]);
            }
    }

    resetIntervals() {
        for (let x = 0; x < this.cells.length; x++)
            for (let y = 0; y < this.cells[0].length; y++)
                this.cells[x][y].riskInterval = [];
    }

    getCellFromNode(node) {
        return this.cells[Math.floor(node.x)][Math.floor(node.y)];
    }
}

module.exports = { SIPP, Graph };