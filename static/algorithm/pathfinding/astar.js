const { BinaryHeap } = require("../binaryHeap");

class Astar {
    constructor(graph) {
        this.graph = graph;                                         // Graph class
        this.openHeap = new BinaryHeap(node => node.f);             // Open heap instead of open list.

        this.treeNodes = 0;                                         // Number of tree nodes
        this.searchSteps = 0;                                       // Number of search steps
        this.scannedNodes = 0;                                      // Number of scanned nodes
        this.sortedElements = 0;                                    // Number of sorted elements (for Zeta*)
    }

    // Initialization
    init(start, end) {

        const startNode = this.graph.vertices[start[0]][start[1]];
        const endNode = this.graph.vertices[end[0]][end[1]];

        startNode.g = 0;
        startNode.h = this.heuristic(startNode, endNode);
        startNode.f = startNode.h;

        this.openHeap.push(startNode);
        this.treeNodes++;

        // For Zeta*
        this.initBoundary(startNode, endNode);
        return [startNode, endNode];
    }

    initBoundary() {
        return;
    }

    // Search the path
    search(start, end) {

        // Initialization
        const [startNode, endNode] = this.init(start, end);

        // Main loop
        while (this.openHeap.size() > 0) {

            // Get the node with the lowest f-value for next step. The heap keeps it sorted.
            const currentNode = this.openHeap.shift();
            this.searchSteps++;

            // Move currentNode from open to closed.
            currentNode.closed = true;

            // The target has been found.
            if (currentNode === endNode)
                return this.pathTo(endNode);

            // Grid-by-grid forward expansion.
            this.forwardExpansion(currentNode, startNode, endNode);
        }

        // Fail to find a path.
        return [];
    }

    // Grid-by-grid forward expansion.
    forwardExpansion(currentNode, _, endNode) {

        // Find all neighbors for the current node.
        const neighbors = this.findNeighbors(currentNode);

        for (const neighbor of neighbors) {

            // Skip if already closed.
            if (neighbor.closed) continue;

            // Find the neighbor's parent.
            const parent = this.findParent(currentNode, neighbor);

            // Update the neighbor's (true) parent.
            this.updateParent(parent, neighbor, endNode);
        }
    }

    // 8-connected neighbors
    findNeighbors(node) {
        let left, leftdown, leftup, right, rightdown, rightup, down, up;
        const [x, y] = [node.x, node.y];
        const vertices = this.graph.vertices;

        // Left
        if (vertices[x - 1]) {
            if (vertices[x - 1][y])
                left = vertices[x - 1][y];
            if (vertices[x - 1][y - 1])
                leftdown = vertices[x - 1][y - 1];
            if (vertices[x - 1][y + 1])
                leftup = vertices[x - 1][y + 1];
        }

        // Right
        if (vertices[x + 1]) {
            if (vertices[x + 1][y])
                right = vertices[x + 1][y];
            if (vertices[x + 1][y - 1])
                rightdown = vertices[x + 1][y - 1];
            if (vertices[x + 1][y + 1])
                rightup = vertices[x + 1][y + 1];
        }

        // Down
        if (vertices[x][y - 1])
            down = vertices[x][y - 1];

        // Up
        if (vertices[x][y + 1])
            up = vertices[x][y + 1];

        // Unreachable neighbors or dead ends
        // The following code ensures that only four grids are scanned.
        let neighbors = [];
        if (this.isLeftDownBlocked(node.x, node.y)) {
            if (this.isLeftUpBlocked(node.x, node.y)) {
                if (this.isRightDownBlocked(node.x, node.y)) {
                    if (!node.parent)
                        neighbors = [right, up, rightup];
                }
                else {
                    if (this.isRightUpBlocked(node.x, node.y)) {
                        if (!node.parent)
                            neighbors = [right, down, rightdown];
                    }
                    else neighbors = [right, up, down, rightup, rightdown];
                }
            }
            else {
                if (this.isRightDownBlocked(node.x, node.y)) {
                    if (this.isRightUpBlocked(node.x, node.y)) {
                        if (!node.parent)
                            neighbors = [left, up, leftup];
                    }
                    else neighbors = [left, right, up, leftup, rightup];
                }
                else {
                    if (this.isRightUpBlocked(node.x, node.y)) {
                        if (!node.parent)
                            neighbors = [left, right, up, down, leftup, rightdown];
                    }
                    else neighbors = [left, right, up, down, leftup, rightup, rightdown];
                }
            }
        }
        else {
            if (this.isLeftUpBlocked(node.x, node.y)) {
                if (this.isRightDownBlocked(node.x, node.y)) {
                    if (!node.parent) {
                        if (this.isRightUpBlocked(node.x, node.y))
                            neighbors = [left, down, leftdown];
                        else neighbors = [left, right, up, down, leftdown, rightup];
                    }
                }
                else {
                    if (this.isRightUpBlocked(node.x, node.y))
                        neighbors = [left, right, down, leftdown, rightdown];
                    else neighbors = [left, right, up, down, leftdown, rightup, rightdown];
                }
            }
            else {
                if (this.isRightDownBlocked(node.x, node.y)) {
                    if (this.isRightUpBlocked(node.x, node.y))
                        neighbors = [left, up, down, leftup, leftdown];
                    else neighbors = [left, right, up, down, leftup, leftdown, rightup];
                }
                else {
                    if (this.isRightUpBlocked(node.x, node.y))
                        neighbors = [left, right, up, down, leftup, leftdown, rightdown];
                    else neighbors = [left, right, up, down, leftup, leftdown, rightup, rightdown];
                }
            }
        }

        neighbors = neighbors.filter(n => n);
        this.scannedNodes += neighbors.length;
        return neighbors;
    }

    findParent(currentNode) {
        return currentNode;
    }

    updateParent(parent, neighbor, endNode) {
        const gScore = parent.g + this.getCost(parent, neighbor);
        const beenVisited = neighbor.visited;

        if (gScore < neighbor.g) {

            // The parent is better than the neighbor.parent
            // If neighbor.parent == null, neighbor.g == Infinity
            neighbor.visited = true;
            neighbor.parent = parent;
            if (neighbor.h === 0)
                neighbor.h = this.heuristic(neighbor, endNode);
            neighbor.g = gScore;
            neighbor.f = neighbor.g + neighbor.h;

            // Update the neighbor in open.
            if (!beenVisited) {
                this.openHeap.push(neighbor);
                this.treeNodes++;
            }
            else this.openHeap.sortElement(neighbor);
        }
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

    getCost(node1, node2) {
        return this.euclidean(node1, node2);
    }

    heuristic(node1, node2) {
        if (node1 === node2) return 0;
        const d1 = Math.abs(node1.x - node2.x);
        const d2 = Math.abs(node1.y - node2.y);
        // -0.585787 ~= sqrt(2) - 2
        // A litte trick to make the grid path more straight
        const distance = (d1 + d2) - 0.585787 * Math.min(d1, d2);
        return distance;
    }

    euclidean(node1, node2) {
        const d1 = node2.x - node1.x;
        const d2 = node2.y - node1.y;
        return Math.sqrt(d1 * d1 + d2 * d2);
    }

    isLeftDownBlocked(x, y) {
        return this.bit(this.graph.leftdown[x], y);
    }

    isLeftUpBlocked(x, y) {
        return this.bit(this.graph.leftup[x], y);
    }

    isRightDownBlocked(x, y) {
        return this.bit(this.graph.rightdown[x], y);
    }

    isRightUpBlocked(x, y) {
        return this.bit(this.graph.rightup[x], y);
    }

    isCornerPoint(x, y) {
        return this.bit(this.graph.isCorner[x], y);
    }

    // Get the bit at position y.
    bit(row, y) {
        return (row[y >> 5] >>> (y & 31)) & 1;
    }
}

class Node {
    constructor(point) {
        this.x = point[0];
        this.y = point[1];
        this.reset();
    }

    reset() {
        this.f = Infinity;
        this.g = Infinity;
        this.h = 0;
        this.fh = Infinity;
        this.visited = false;
        this.closed = false;
        this.parent = null;
        this.children = [];
    }
}

class Graph {
    constructor(grid) {
        this.grid = grid;                   // Storing grid
        this.width = this.grid.length;
        this.length = this.grid[0].length;
        this.vertices = [];                 // 2D array for storing vertices.
        this.init();
        this.packGrid();
    }

    // Initialize nodes
    init() {
        this.vertices = [];
        for (let x = 0; x <= this.grid.length; x++) {
            this.vertices[x] = [];
            for (let y = 0; y <= this.grid[0].length; y++)
                this.vertices[x][y] = new Node([x, y]);
        }
    }

    packGrid() {
        const wordsPerRow = Math.ceil((this.length + 1) / 32);
        this.leftdown = new Array(this.width + 1);
        this.leftup = new Array(this.width + 1);
        this.rightup = new Array(this.width + 1);
        this.rightdown = new Array(this.width + 1);
        this.isCorner = new Array(this.width + 1);
        this.diagBlocked = new Array(this.width + 1);

        for (let x = 0; x <= this.width; x++) {
            const ld = new Uint32Array(wordsPerRow);
            const lu = new Uint32Array(wordsPerRow);
            const ru = new Uint32Array(wordsPerRow);
            const rd = new Uint32Array(wordsPerRow);
            const ic = new Uint32Array(wordsPerRow);
            const db = new Uint32Array(wordsPerRow);

            for (let y = 0; y <= this.length; y++) {
                const word = y >> 5;
                const bit = y & 31;
                const mask = 1 << bit;

                const ldBlocked = this.isCellBlocked(x - 1, y - 1);
                const luBlocked = this.isCellBlocked(x - 1, y);
                const ruBlocked = this.isCellBlocked(x, y);
                const rdBlocked = this.isCellBlocked(x, y - 1);

                if (ldBlocked) ld[word] |= mask;
                if (luBlocked) lu[word] |= mask;
                if (ruBlocked) ru[word] |= mask;
                if (rdBlocked) rd[word] |= mask;

                const count = ldBlocked + luBlocked + ruBlocked + rdBlocked;
                if (count === 1) ic[word] |= mask;

                const diagBlocked = (ldBlocked && ruBlocked) || (luBlocked && rdBlocked);
                if (diagBlocked) db[word] |= mask;
            }

            this.leftdown[x] = ld;
            this.leftup[x] = lu;
            this.rightup[x] = ru;
            this.rightdown[x] = rd;
            this.isCorner[x] = ic;
            this.diagBlocked[x] = db;
        }
    }

    // Check if a grid cell is blocked
    isCellBlocked(x, y) {
        return !this.grid[x] || !this.grid[x][y];
    }

    // Reset nodes
    reset() {
        for (let x = 0; x < this.vertices.length; x++)
            for (let y = 0; y < this.vertices[0].length; y++)
                this.vertices[x][y].reset();
    }
}

module.exports = { Astar, Graph };