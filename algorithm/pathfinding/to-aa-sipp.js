const { BinaryHeap } = require("../function/binaryHeap");
const { lineOfSightGrid, lineSegOnGrid, intersectSets } = require("../function/function");

// Time-Optimal Any-Angle Safe-Interval Path Planning (TO-AA-SIPP)
class TOAASIPP {
    constructor(mesh, speed, isSIPP) {
        this.mesh = mesh;                                           // 0: obstacle, 1: normal
        this.speed = speed;                                         // Time-optimal

        this.canWait = isSIPP && true;                              // Specifies whether the wait action is allowed or not.
        this.graph = new Graph(mesh, this.canWait);                 // Graph-related operations
        this.openHeap = new BinaryHeap(node => node.f);             // Open heap instead of open list.
        this.boundHeap = new BinaryHeap(cell => cell.fh);           // Bound heap for Zeta/Zeta*-SIPP
        this.openCells = [];                                        // List of open cells (2D grids) to store the results of visibility checks
        this.isSIPP = isSIPP;                                       // Specifies whether safe intervals are considered

        // Add a small buffer for the waiting time (personal preference):
        // Avoid crossing the diagonal corner between two obstacle grids
        this.timebuffer = 10e-3;

        this.steps = 0;                                             // Search steps
        this.treeCells = [];                                        // List of visited cells
    }

    // TO-AA-SIPP Initialization
    init(start, end) {

        // Initialize the start and end.
        const startCell = this.graph.grids[Math.floor(start[0])][Math.floor(start[1])];
        const endCell = this.graph.grids[Math.floor(end[0])][Math.floor(end[1])];

        // Initialize the start cell.
        this.initStart(startCell, endCell);

        // Initialize the other cells.
        this.graph.cells.forEach(cell => {
            if (!cell.weight || cell === startCell) return;
            this.visitCell(cell, endCell);
        });

        return [startCell, endCell];
    }

    // Initialize the start node.
    initStart(startCell, endCell) {
        this.visitCell(startCell, endCell);
        const startNode = startCell.nodes[0];
        startNode.g = 0;
        startNode.f = startNode.h;
        this.openHeap.sortElement(startNode);
    }

    // Visit a cell
    visitCell(cell, endCell) {
        if (cell.h === 0) // Avoid repeated computation
            cell.h = this.heuristic(cell, endCell);
        cell.visited = true;
        this.treeCells.push(cell);
        this.openCells.push(cell);
        cell.nodes.forEach(node => {
            node.h = cell.h;
            this.openHeap.push(node);
        });
    }

    // Search the path
    search(start, end) {

        // TO-AA-SIPP Initialization
        const [startCell, endCell] = this.init(start, end);

        // Main loop
        while (this.openHeap.size() > 0 && this.openHeap.content[0].f < Infinity) {
            this.steps++;
            const currentNode = this.findNextClosedNode();
            if (currentNode) {
                const currentCell = this.graph.getCellFromNode(currentNode);
                if (currentCell === endCell)
                    return this.pathTo(currentNode);
                this.invertedExpansion(currentNode, currentCell, endCell);
            }
            // Add more nodes into the open heap (for Zeta/Zeta*-SIPP)
            this.forwardExpansion(startCell, endCell);
        }
        // Fail to find a path.
        return [];
    }

    // Find the next closed node
    findNextClosedNode() {
        // Get the node with the lowest f-value for next step. The heap keeps it sorted.
        const currentNode = this.openHeap.shift();

        // Make sure the best potential parent is not in the potential parents
        const index = currentNode.potentialParents.indexOf(currentNode.bestPotentialParent);
        if (index > -1) {
            currentNode.potentialParents.splice(index, 1);
            currentNode.gLowArray.splice(index, 1);
        }

        // Calculate the real cost of the collision-free transition.
        const gNew = !currentNode.bestPotentialParent ? 0 :
            this.transition(currentNode.bestPotentialParent, currentNode) + currentNode.bestPotentialParent.g;
        if (gNew < currentNode.g) {
            currentNode.g = gNew;
            currentNode.parent = currentNode.bestPotentialParent;
            currentNode.waitTime = gNew - currentNode.gLow;
        }

        // Best potential parent := True parent
        currentNode.gLow = currentNode.g;
        currentNode.bestPotentialParent = currentNode.parent;
        currentNode.f = currentNode.gLow + currentNode.h;

        // Make sure the current parent is the best of the "current" potential parents.
        if (this.newBestPotentialParentExists(currentNode)) {
            this.openHeap.push(currentNode);
            return;
        }

        // Perhaps there is another node in OPEN that could be the potential parent of Node n. 
        // The inequality makes sure the optimality.
        if ((this.openHeap.size() === 0 || currentNode.g + currentNode.h <= this.openHeap.content[0].f) &&
            (this.boundHeap.size() === 0 || currentNode.g + currentNode.h <= this.boundHeap.content[0].fh)) {

            // When this node is closed, the optimal path from the start node to this node is known.
            currentNode.closed = true;

            // Update the cell
            const currentCell = this.graph.getCellFromNode(currentNode);
            this.updateCloseCell(currentCell);

            return currentNode;
        }
        else {
            this.openHeap.push(currentNode);
            return;
        }
    }

    updateCloseCell() {
        return;
    }

    // A cell is considered non-open if all its nodes are closed.
    // This operation avoids building visibility connections between the current cell and closed cells.
    updateOpenCell(currentCell) {
        if (currentCell.nodes.every(node => node.closed)) {
            const index = this.openCells.indexOf(currentCell);
            if (index > -1) this.openCells.splice(index, 1);
        }
    }

    invertedExpansion(currentNode, currentCell) {
        this.updateOpenCell(currentCell);

        // Build visibility connections between the current cell and other open cells.
        if (!currentCell.visibleCells.length) {
            this.openCells.forEach(cell => {
                if (cell === currentCell) return;
                if (this.graph.lineOfSightGrid(currentCell, cell))
                    currentCell.visibleCells.push(cell);
            });
        }
        // Inverted expansion: add potential parents
        currentCell.visibleCells.forEach(cell =>
            cell.nodes.forEach(node => {
                if (node.closed) return;
                this.addPotentialParent(currentNode, node);
            }));
    }

    forwardExpansion() {
        return;
    }

    // Add a potential parent and update the best potential parent.
    addPotentialParent(currentNode, node) {
        const gLowNew = currentNode.g + this.heuristic(currentNode, node);
        if (gLowNew < node.gLow) {
            node.gLow = gLowNew;
            node.bestPotentialParent = currentNode;
            node.f = node.gLow + node.h;
            this.openHeap.sortElement(node);
        }
        node.potentialParents.push(currentNode);
        node.gLowArray.push(gLowNew); // Avoid repeated computation
    }

    // Check if new best potential parent exists
    newBestPotentialParentExists(node) {
        let exist = false;
        node.potentialParents.forEach((parent, i) => {
            const gLowNew = node.gLowArray[i];
            if (gLowNew < node.gLow) {
                node.gLow = gLowNew;
                node.bestPotentialParent = parent;
                node.f = node.gLow + node.h;
                exist = true;
            }
        });
        return exist;
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
        if (node1 === node2) return 0;
        const distance = this.euclidean(node1, node2);
        if (!this.speed) return distance;
        return distance / this.speed;
    }

    euclidean(node1, node2) {
        const d1 = Math.abs(node2.x - node1.x);
        const d2 = Math.abs(node2.y - node1.y);
        return Math.sqrt(d1 * d1 + d2 * d2);
    }

    // Calculate the real cost of the collision-free transition from node1 to node2
    transition(node1, node2) {
        if (node1 === node2) return 0;
        if (!this.isSIPP) return this.heuristic(node1, node2);
        return this.getCostSIPP(node1, node2);
    }

    getCostSIPP(node1, node2) {
        // Cannot compute safe intervals without speed.
        if (this.speed === undefined) return this.heuristic(node1, node2);
        const distance = this.euclidean(node1, node2);

        // Check if node2 is reachable.
        let waitTime = 0;
        if (this.canWait) {
            const minTime = distance / this.speed + node1.g;
            if (minTime <= node2.safeInterval[0])
                waitTime = node2.safeInterval[0] - minTime + this.timebuffer; // Add a small buffer
            if (waitTime + node1.g >= node1.safeInterval[1]) return Infinity;
        }

        // Find which grids this line segment goes through, and
        // Compute the distances from the start to the intersections.
        const traceGrids = lineSegOnGrid(node1.x, node1.y, node2.x, node2.y, distance);

        // Compute the intervals occupied by this line segment without waiting.
        const intervals = traceGrids.map(grid =>
            [grid.dist[0] / this.speed + node1.g, grid.dist[1] / this.speed + node1.g]);
        intervals.forEach(intv => { intv[0] += waitTime; intv[1] += waitTime; });

        // SIPP-based conflict detection and resolution
        for (let i = 0; i < traceGrids.length; i++) {
            if (!this.canWait) {
                if (this.conflictDetection(traceGrids, intervals, i)) return Infinity;
            }
            else {
                waitTime = this.updateWaitTime(traceGrids, intervals, i, waitTime);
                if (waitTime + node1.g >= node1.safeInterval[1]) return Infinity;
            }
        }
        return waitTime + this.heuristic(node1, node2);
    }

    // Conflict detection based on safe intervals.
    conflictDetection(traceGrids, intervals, index) {
        const grid = traceGrids[index];
        // The mesh should exist since line-of-sight checks have already done.
        const mesh = this.mesh[grid.x][grid.y];
        for (let k = 0; k < mesh.riskInterval.length / 2; k++) {
            const meshRiskInterval = [mesh.riskInterval[2 * k], mesh.riskInterval[2 * k + 1]];
            const conflictInterval = intersectSets(meshRiskInterval, intervals[index]);
            if (conflictInterval.length) return true;
        }
        return false;
    }

    // Compute the waiting time for conflict resolution.
    // Also, update the intervals occupied by the line segment.
    updateWaitTime(traceGrids, intervals, index, waitTime) {
        const waitTimeOld = waitTime;
        const grid = traceGrids[index];
        const mesh = this.mesh[grid.x][grid.y];

        // Risk intervals should be ordered.
        for (let k = 0; k < mesh.riskInterval.length / 2; k++) {

            // Check if there is any conflict.
            const meshRiskInterval = [mesh.riskInterval[2 * k], mesh.riskInterval[2 * k + 1]];
            const conflictInterval = intersectSets(meshRiskInterval, intervals[index]);
            if (!conflictInterval.length) continue;

            // Compute the delta of waiting time.
            const delta = meshRiskInterval[1] - intervals[index][0] + this.timebuffer;
            waitTime += delta;
            intervals.forEach(intv => { intv[0] += delta; intv[1] += delta; });
        }

        if (waitTime > waitTimeOld) {
            // Update the waiting time recursively.
            for (let i = 0; i < index; i++)
                waitTime = this.updateWaitTime(traceGrids, intervals, i, waitTime);
        }
        return waitTime;
    }
}

class Cell {
    constructor(point, weight) {
        this.x = point[0];
        this.y = point[1];
        this.weight = weight;
        this.h = 0;
        this.fh = Infinity;
        this.visited = false;
        this.nodes = [];
        this.visibleCells = [];      // Record the visible cells from this node.
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
    constructor(mesh, canWait) {
        this.mesh = mesh;                   // Storing mesh
        this.canWait = canWait;             // Specifies whether the wait action is allowed or not.
        this.grids = [];                    // 2D array for storing cells
        this.cells = [];                    // 1D array for storing cells
        this.checks = 0;                    // The number of line-of-sight checks
        this.scannedGrids = 0;              // The number of scanned grids
        this.init();
    }

    // Initialize cells and nodes
    init() {
        for (let x = 0; x < this.mesh.length; x++) {
            this.grids[x] = [];
            for (let y = 0; y < this.mesh[x].length; y++) {
                const cell = new Cell([x + 0.5, y + 0.5], this.mesh[x][y].weight);
                if (this.canWait) {
                    const safeInterval = [0, ...this.mesh[x][y].riskInterval, Infinity];
                    for (let k = 0; k < safeInterval.length / 2; k++) {
                        const node = new Node([x + 0.5, y + 0.5]);
                        node.safeInterval = [safeInterval[2 * k], safeInterval[2 * k + 1]];
                        cell.nodes.push(node);
                    }
                }
                else cell.nodes = [new Node([x + 0.5, y + 0.5])];
                this.grids[x][y] = cell;
                this.cells.push(cell);
            }
        }
    }

    getCellFromNode(node) {
        return this.grids[Math.floor(node.x)][Math.floor(node.y)];
    }

    // Check if the line segment connecting node1 and node2 intersects any obstacle / boundary in the environment
    lineOfSightGrid(node1, node2) {
        if (!node1 || !node2) return false;
        this.checks++;
        const [isBlocked, gridNumber] = lineOfSightGrid(node1.x, node1.y, node2.x, node2.y, this.mesh);
        this.scannedGrids += gridNumber;
        return isBlocked;
    }
}

module.exports = { TOAASIPP };