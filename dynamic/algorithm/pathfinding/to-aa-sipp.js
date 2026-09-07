const { BinaryHeap } = require("../binaryHeap");
const { intersectSets } = require("../function");
const { lineOfSightGrid, lineSegOnGrid } = require("../line-of-sight");

// Time-Optimal Any-Angle Safe-Interval Path Planning
class TOAASIPP {
    constructor(graph, speed) {
        this.graph = graph;                                         // 0: obstacle, 1: normal
        this.speed = speed;                                         // Time-optimal
        this.openHeap = new BinaryHeap(node => node.f);             // Open heap instead of open list.
        this.timebuffer = 10e-6;                                    // Add a small buffer for the waiting time
        this.openCells = [];                                        // Record cells containing non-closed nodes

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
        const startNode = startCell.nodes[0];
        startNode.g = 0;

        for (let x = 0; x < this.graph.cells.length; x++) {
            for (let y = 0; y < this.graph.cells[0].length; y++) {
                const cell = this.graph.cells[x][y];
                if (!cell.weight) continue;
                this.visitCell(cell, endCell);

                if (cell === startCell) {
                    if (cell.nodes.length !== 1)
                        this.openCells.push(cell);
                    continue;
                }
                this.openCells.push(cell);

                if (!this.lineOfSight(startCell, cell)) continue;
                cell.nodes.forEach(node => this.initNodes(node, startNode));
            }
        }
        return [startCell, endCell];
    }

    // TO-AA-SIPP inserts all nodes into the open list during initialization.  
    // Unlike the version described in the TO-AA-SIPP paper, we only insert nodes that have a finite f-value.
    initNodes(node, startNode) {
        node.visited = true;
        node.gLow = Math.max(node.safeInterval[0], this.heuristic(startNode, node));
        node.bestPotentialParent = startNode;

        node.f = node.gLow + node.h;
        this.openHeap.push(node);
        this.treeNodes++;

        node.potentialParents.push(startNode);
        node.gLowArray.push(node.gLow);  // Avoid repeated computation
    }

    // Visit a cell
    visitCell(cell, endCell) {
        cell.visited = true;
        cell.h = this.heuristic(cell, endCell);
        cell.nodes.forEach(node => node.h = cell.h);
    }

    // Search the path
    search(start, end) {

        // Initialization
        const [startCell, endCell] = this.init(start, end);

        // Main loop
        while (this.openHeap.size() > 0 && this.openHeap.content[0].f < Infinity) {

            // Find the next closed node.
            const currentNode = this.findNextClosedNode();
            this.searchSteps++;

            // The target has been found.
            if (currentNode) {
                const currentCell = this.graph.getCellFromNode(currentNode);
                if (currentCell === endCell)
                    return this.pathTo(currentNode);
            }
            // Node expansion.
            this.nodeExpansion(currentNode, startCell, endCell);
        }
        // Fail to find a path.
        return [];
    }

    // Find the next closed node.
    findNextClosedNode() {
        const currentNode = this.openHeap.shift();

        // Ensure that the best potential parent is excluded from the list of potential parents.
        this.removeBestPotentialParent(currentNode);

        // Calculate the real cost of the collision-free transition.
        const gNew = this.transition(currentNode.bestPotentialParent, currentNode) + currentNode.bestPotentialParent.g;
        if (gNew < currentNode.g) {
            currentNode.g = gNew;
            currentNode.parent = currentNode.bestPotentialParent;
        }

        // Best potential parent := True parent
        currentNode.gLow = currentNode.g;
        currentNode.bestPotentialParent = currentNode.parent;
        currentNode.f = currentNode.gLow + currentNode.h;

        // Ensure that the current parent is the best of the "current" potential parents.
        if (this.newBestPotentialParentExists(currentNode) || currentNode.g === Infinity) {
            this.openHeap.push(currentNode); return;
        }

        // Perhaps there is another node in OPEN that could be the potential parent of the current node.
        const realCost = currentNode.g + currentNode.h;
        if ((this.openHeap.size() === 0 || realCost <= this.openHeap.content[0].f) && this.isCostBounded(realCost)) {

            // When this node is closed, the optimal path from the start node to this node is known.
            currentNode.closed = true;
            currentNode.waitTime = currentNode.g - currentNode.parent.g -
                this.euclidean(currentNode.parent, currentNode) / this.speed;

            currentNode.potentialParents.length = 0;
            currentNode.gLowArray.length = 0;
            return currentNode;
        }
        else { this.openHeap.push(currentNode); return; }
    }

    removeBestPotentialParent(node) {
        const last = node.potentialParents.length - 1;
        if (last < 0) return;

        if (last > 0) {
            node.potentialParents[0] = node.potentialParents[last];
            node.gLowArray[0] = node.gLowArray[last];
        }

        node.potentialParents.pop();
        node.gLowArray.pop();
    }

    // For Zeta*-SIPP
    isCostBounded() {
        return true;
    }

    // A cell is considered non-open if all its nodes are closed.
    updateOpenCell(currentCell) {
        if (currentCell.nodes.every(node => node.closed)) {
            const index = this.openCells.indexOf(currentCell);
            if (index > -1) this.openCells.splice(index, 1);
        }
    }

    // Inverted expansion based on the current node.
    nodeExpansion(currentNode) {
        if (!currentNode) return;
        const currentCell = this.graph.getCellFromNode(currentNode);
        this.updateOpenCell(currentCell);

        for (const cell of this.openCells) {
            if (cell === currentCell || !this.lineOfSight(currentCell, cell)) continue;
            cell.nodes.forEach(node => {
                if (node.closed) return;
                this.addPotentialParent(currentNode, node);
            });
        }
    }

    // Add a potential parent and update the best potential parent.
    addPotentialParent(parent, child) {
        let gLowNew = parent.g + this.heuristic(parent, child);
        if (gLowNew >= child.safeInterval[1]) return;
        gLowNew = Math.max(child.safeInterval[0], gLowNew);

        // Append first
        child.potentialParents.push(parent);
        child.gLowArray.push(gLowNew);
        const last = child.potentialParents.length - 1;

        const beenVisited = child.visited;
        if (gLowNew < child.gLow) {
            child.visited = true;
            child.gLow = gLowNew;
            child.bestPotentialParent = parent;
            child.f = child.gLow + child.h;
            if (!beenVisited) {
                this.openHeap.push(child);
                this.treeNodes++;
            }
            else this.openHeap.sortElement(child);

            if (last > 0) {
                [child.potentialParents[0], child.potentialParents[last]] = [child.potentialParents[last], child.potentialParents[0]];
                [child.gLowArray[0], child.gLowArray[last]] = [child.gLowArray[last], child.gLowArray[0]];
            }
        }
    }

    // Check if new best potential parent exists
    newBestPotentialParentExists(node) {
        let exist = false;
        let bestIndex = 0;

        for (let i = 0; i < node.potentialParents.length; i++)
            if (node.gLowArray[i] < node.gLow) {
                node.gLow = node.gLowArray[i];
                node.bestPotentialParent = node.potentialParents[i];
                bestIndex = i;
                exist = true;
            }

        if (bestIndex !== 0) {
            [node.potentialParents[0], node.potentialParents[bestIndex]] = [node.potentialParents[bestIndex], node.potentialParents[0]];
            [node.gLowArray[0], node.gLowArray[bestIndex]] = [node.gLowArray[bestIndex], node.gLowArray[0]];
        }

        node.f = node.gLow + node.h;
        return exist;
    }

    // Calculate the real cost of the collision-free transition from node1 to node2
    transition(node1, node2) {
        const pathTime = this.euclidean(node1, node2) / this.speed;
        const traceGrids = lineSegOnGrid(node1.x, node1.y, node2.x, node2.y, pathTime, this.graph.cells);
        const waitTime = this.calWaitTime(node1, node2, pathTime, traceGrids);
        return waitTime + pathTime;
    }

    lineOfSight(cell1, cell2) {
        const [isUnblocked, number] = lineOfSightGrid(cell1.x, cell1.y, cell2.x, cell2.y, this.graph.cells);
        this.scannedNodes += number;
        return isUnblocked;
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
        return this.euclidean(node1, node2) / this.speed;
    }

    euclidean(node1, node2) {
        const d1 = node2.x - node1.x;
        const d2 = node2.y - node1.y;
        return Math.sqrt(d1 * d1 + d2 * d2);
    }
}

module.exports = { TOAASIPP };