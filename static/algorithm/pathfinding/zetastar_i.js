const { Astar } = require("./astar");
const { Shadowcast } = require("../shadowcasting");
const { BinaryHeap } = require("../binaryHeap");

class Zetastar_i extends Astar {
    constructor(graph) {
        super(graph);                                            // Class inherits from A*.
        this.boundHeap = new BinaryHeap(node => node.fh);        // Bound heap for bound nodes.
        this.shadowcast = new Shadowcast(graph);                 // Recursive shadowcasting.
        this.shadowcast.inverted = true;                         // Inverted scanning, considering only corner nodes and taut paths.
        this.scanBuffer = 2.828428;                              // Add buffer due to grid aliasing.
        this.costBuffer = 10;                                    // Add buffer for expanding the bounding box.
        this.costBound;                                          // Avoid expanding the bounding box frequently.
        this.boundingBox;                                        // Bounding box object.
    }

    heuristic(node1, node2) {
        return this.euclidean(node1, node2);
    }

    // Bounding box
    initBoundary(startNode, endNode) {
        const xMin = Math.min(startNode.x, endNode.x);
        const xMax = Math.max(startNode.x, endNode.x);
        const yMin = Math.min(startNode.y, endNode.y);
        const yMax = Math.max(startNode.y, endNode.y);

        const cx = (startNode.x + endNode.x) / 2;
        const cy = (startNode.y + endNode.y) / 2;

        const ellipseParams = this.cacheEllipseParam(startNode, endNode);

        this.boundingBox = {
            center: [cx, cy],
            xRange: [xMin, xMax],
            yRange: [yMin, yMax],
            ...ellipseParams
        };

        startNode.fh = startNode.h;
        this.shadowcast.startNode = startNode;
        this.shadowcast.endNode = endNode;
        this.costBound = startNode.h;
        this.initBoundList(xMin, xMax, yMin, yMax, startNode, endNode);
    }

    // Initialize the bound list
    initBoundList(xMin, xMax, yMin, yMax, startNode, endNode) {
        for (let x = xMin; x <= xMax; x++)
            for (let y = yMin; y <= yMax; y++)
                this.initBoundNode(x, y, startNode, endNode);
    }

    // Initialize the bound node
    initBoundNode(x, y, startNode, endNode) {
        const node = this.graph.vertices[x][y];
        if (node === startNode) return;
        if (!this.isCornerPoint(node.x, node.y) && node !== endNode) return;
        if (node.fh === Infinity) {
            node.h = this.euclidean(node, endNode);
            node.fh = this.euclidean(startNode, node) + node.h;
        }
        this.boundHeap.push(node);
        this.sortedElements++;
    }

    // Check if the bounding box reaches the map boundary.
    isBoundingBoxAtBoundary() {
        const { xRange, yRange } = this.boundingBox;
        return xRange[0] === 0 && xRange[1] === this.graph.vertices.length - 1 &&
            yRange[0] === 0 && yRange[1] === this.graph.vertices[0].length - 1;
    }

    // Extend the bounding box one step further.
    extendBoundingbox(step, startNode, endNode) {
        const { xRange, yRange } = this.boundingBox;
        const maxX = this.graph.vertices.length - 1;
        const maxY = this.graph.vertices[0].length - 1;

        this.boundingBox.xRange = [
            Math.max(0, xRange[0] - step),
            Math.min(maxX, xRange[1] + step)
        ];

        this.boundingBox.yRange = [
            Math.max(0, yRange[0] - step),
            Math.min(maxY, yRange[1] + step)
        ];

        this.updateBoundList(xRange, yRange, this.boundingBox.xRange,
            this.boundingBox.yRange, startNode, endNode);
    }

    // Update the bounding box according to the ellipse.
    extendBoundingBoxEllipse(cost, startNode, endNode) {
        const a2 = cost * cost / 4;
        const [cx, cy] = this.boundingBox.center;
        const halfWidth = Math.sqrt(a2 - this.boundingBox.c2sin2);
        const halfHeight = Math.sqrt(a2 - this.boundingBox.c2cos2);
        const { xRange, yRange } = this.boundingBox;
        const maxX = this.graph.vertices.length - 1;
        const maxY = this.graph.vertices[0].length - 1;

        this.boundingBox.xRange = [
            Math.max(0, Math.min(xRange[0], Math.floor(cx - halfWidth))),
            Math.min(maxX, Math.max(xRange[1], Math.ceil(cx + halfWidth)))
        ];

        this.boundingBox.yRange = [
            Math.max(0, Math.min(yRange[0], Math.floor(cy - halfHeight))),
            Math.min(maxY, Math.max(yRange[1], Math.ceil(cy + halfHeight)))
        ];

        this.updateBoundList(xRange, yRange, this.boundingBox.xRange,
            this.boundingBox.yRange, startNode, endNode);
    }

    // Parameters of the ellipse's bounding box (c^2 * sin^2)
    cacheEllipseParam(startNode, endNode) {
        const dx = endNode.x - startNode.x;
        const dy = endNode.y - startNode.y;
        return {
            c2sin2: (dy * dy) / 4,
            c2cos2: (dx * dx) / 4
        };
    }

    // Add new nodes to the bound list.
    updateBoundList(xRangeOld, yRangeOld, xRangeNew, yRangeNew, startNode, endNode) {

        const [xMinOld, xMaxOld] = xRangeOld;
        const [yMinOld, yMaxOld] = yRangeOld;
        const [xMinNew, xMaxNew] = xRangeNew;
        const [yMinNew, yMaxNew] = yRangeNew;

        for (let x = xMinNew; x <= xMaxNew; x++)
            for (let y = yMinNew; y <= yMaxNew; y++)
                if (x < xMinOld || x > xMaxOld || y < yMinOld || y > yMaxOld)
                    this.initBoundNode(x, y, startNode, endNode);
    }

    // Elliptical forward expansion.
    forwardExpansion(currentNode, startNode, endNode) {
        for (const child of currentNode.children) {
            if (child.closed) continue;
            if (this.isTautPath(currentNode, child, endNode))
                this.updateParent(currentNode, child, endNode);
        }
        currentNode.children = [];

        while (this.openHeap.size() === 0) {
            this.updateOpenList(startNode, endNode);
            if (this.boundHeap.size() === 0) {
                if (this.isBoundingBoxAtBoundary()) break;
                this.extendBoundingbox(this.costBuffer, startNode, endNode);
            }
        }

        if (this.openHeap.size() > 0) {
            if (this.openHeap.content[0].f > this.costBound) {
                this.costBound = this.openHeap.content[0].f + this.costBuffer;
                this.extendBoundingBoxEllipse(this.costBound, startNode, endNode);
            }
            this.updateOpenList(startNode, endNode);
        }
    }

    updateOpenList(startNode, endNode) {
        while (this.boundHeap.size() > 0 && (this.openHeap.size() === 0 ||
            this.boundHeap.content[0].fh <= this.openHeap.content[0].f)) {
            const newNode = this.boundHeap.shift();
            this.invertedScan(newNode, startNode, endNode);
        }
    }

    // Inverted scanning from an "open" node (not open yet).
    // This node will be added to the open heap if there exists a closed node as its parent.
    invertedScan(node, startNode, endNode) {
        this.shadowcast.maxCost = node.fh + this.scanBuffer;
        const visibleNodes = this.shadowcast.scan(node);
        this.scannedNodes += this.shadowcast.scannedNodes;

        for (const n of visibleNodes) {

            if (n.fh === Infinity) {
                n.h = this.euclidean(n, endNode);
                n.fh = this.euclidean(startNode, n) + n.h;
            }

            if (n.fh > node.fh + 1e-10) continue;
            if (n.closed) {
                if (this.isTautPath(n, node, endNode))
                    this.updateParent(n, node, endNode);
            }
            else {
                n.children.push(node);
                node.children.push(n);
            }
        }
    }

    // Check whether the path from node1 to node2 is taut.
    isTautPath(node1, node2, endNode) {
        if (node2 === endNode) return true;

        // From left to right
        if (node1.x < node2.x) {
            if (node1.y < node2.y)
                return !this.isRightUpBlocked(node2.x, node2.y);
            if (node1.y === node2.y)
                return !this.isRightUpBlocked(node2.x, node2.y) && !this.isRightDownBlocked(node2.x, node2.y);
            return !this.isRightDownBlocked(node2.x, node2.y);
        }

        // From right to left
        if (node1.x > node2.x) {
            if (node1.y < node2.y)
                return !this.isLeftUpBlocked(node2.x, node2.y);
            if (node1.y === node2.y)
                return !this.isLeftUpBlocked(node2.x, node2.y) && !this.isLeftDownBlocked(node2.x, node2.y);
            return !this.isLeftDownBlocked(node2.x, node2.y);
        }

        // Up and down
        if (node1.x === node2.x) {
            if (node1.y < node2.y)
                return !this.isLeftUpBlocked(node2.x, node2.y) && !this.isRightUpBlocked(node2.x, node2.y);
            return !this.isLeftDownBlocked(node2.x, node2.y) && !this.isRightDownBlocked(node2.x, node2.y);
        }

        return false;
    }
}

module.exports = { Zetastar_i };