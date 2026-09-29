const { calArrivalTimeLowerBound, calWaitTime, updateWaitTime } = require("../waiting-time");
const { OpenHeap } = require("../binaryHeap");
const { trivialPath } = require("../function");
const { lineOfSightGrid, lineSegOnGrid } = require("../line-of-sight");

// Time-Optimal Any-Angle Safe-Interval Path Planning
class TOAASIPP {
    constructor(graph, speed) {
        this.graph = graph;                                         // 0: obstacle, 1: normal
        this.speed = speed;                                         // Time-optimal
        this.openHeap = new OpenHeap();                            // Indexed search heap.
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
        startNode.closed = true;

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

                for (const node of cell.nodes) this.initNodes(node, startNode);
            }
        }
        return [startCell, endCell];
    }

    // Initial candidates obey the same endpoint constraints as later ones.
    initNodes(node, startNode) {
        this.addPotentialParent(startNode, node);
    }

    // Visit a cell
    visitCell(cell, endCell) {
        cell.visited = true;
        cell.h = this.heuristic(cell, endCell);
        cell.nodes.forEach(node => node.h = cell.h);
    }

    // Search the path
    search(start, end) {

        const terminalPath = trivialPath(this.graph, start, end);
        if (terminalPath !== null) return terminalPath;

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

            for (const node of cell.nodes) {
                if (node.closed) continue;
                this.addPotentialParent(currentNode, node);
            }
        }
    }

    // Add a potential parent and update the best potential parent.
    addPotentialParent(parent, child) {
        const gLowNew = this.potentialParentLowerBound(parent, child);
        if (gLowNew >= child.safeInterval[1]) return;

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

    // Endpoint occupancy constrains departure before checking intermediate cells.
    potentialParentLowerBound(parent, child) {
        return calArrivalTimeLowerBound(this, parent, child);
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

    // Share transition timing without coupling the two search algorithms.
    calWaitTime(node1, node2, pathTime, traceGrids) {
        return calWaitTime(this, node1, node2, pathTime, traceGrids);
    }

    updateWaitTime(traceGrids, departure, index, waitTime, maxWaitTime) {
        return updateWaitTime(this, traceGrids, departure, index, waitTime, maxWaitTime);
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
