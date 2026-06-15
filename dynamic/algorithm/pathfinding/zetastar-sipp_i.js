const { TOAASIPP } = require("./to-aa-sipp");
const { Shadowcast } = require("../shadowcasting");
const { BinaryHeap } = require("../binaryHeap");

class ZetastarSIPP_i extends TOAASIPP {
    constructor(graph, speed) {
        super(graph, speed);
        this.boundHeap = new BinaryHeap(node => node.fh);    // Bound heap for bound cells.
        this.shadowcast = new Shadowcast(this.graph.cells);  // Recursive shadowcasting.
        this.shadowcast.inverted = true;                     // Inverted scanning.
        this.scanBuffer = 1.414214 / this.speed;             // Add buffer due to grid aliasing.
        this.costBuffer = 10 / this.speed;                   // Add buffer for expanding the bounding box.
        this.costBound = 0;                                  // Avoid expanding the bounding box frequently.
        this.boundingBox;                                    // Bounding box object.
    }

    // Initialization
    init(start, end) {
        const startCell = this.graph.cells[Math.floor(start[0])][Math.floor(start[1])];
        const endCell = this.graph.cells[Math.floor(end[0])][Math.floor(end[1])];
        const startNode = startCell.nodes[0];
        startNode.g = 0;
        startNode.closed = true;
        startNode.f = startNode.h;
        this.visitCell(startCell, startCell, endCell);
        this.costBound = startNode.f;

        this.initBoundary(startCell, endCell);
        this.nodeExpansion(startNode, startCell, endCell);
        return [startCell, endCell];
    }

    // Different from TO-AA-SIPP, this function adds the "fh"
    visitCell(cell, startCell, endCell) {
        cell.visited = true;
        cell.h = this.heuristic(cell, endCell);
        cell.fh = this.heuristic(startCell, cell) + cell.h;
        cell.nodes.forEach(node => node.h = cell.h);
    }

    // Bounding box
    initBoundary(startCell, endCell) {

        const maxX = this.graph.cells.length - 1;
        const maxY = this.graph.cells[0].length - 1;

        let xMin = Math.min(startCell.x, endCell.x);
        let xMax = Math.max(startCell.x, endCell.x);
        let yMin = Math.min(startCell.y, endCell.y);
        let yMax = Math.max(startCell.y, endCell.y);

        // The coordinates are not integers.
        xMin = Math.max(Math.floor(xMin) - 1, 0);
        xMax = Math.min(Math.ceil(xMax), maxX);
        yMin = Math.max(Math.floor(yMin) - 1, 0);
        yMax = Math.min(Math.ceil(yMax), maxY);

        const cx = (startCell.x + endCell.x) / 2;
        const cy = (startCell.y + endCell.y) / 2;

        const ellipseParams = this.cacheEllipseParam(startCell, endCell);

        this.boundingBox = {
            center: [cx, cy],
            xRange: [xMin, xMax],
            yRange: [yMin, yMax],
            ...ellipseParams
        };

        this.shadowcast.visitCell = cell => this.visitCell(cell, startCell, endCell);
        this.initBoundList(xMin, xMax, yMin, yMax, startCell, endCell);
    }

    // Initialize the bound list
    initBoundList(xMin, xMax, yMin, yMax, startCell, endCell) {
        for (let x = xMin; x <= xMax; x++)
            for (let y = yMin; y <= yMax; y++)
                this.initBoundCell(x, y, startCell, endCell);
    }

    // Initialize the bound cell
    initBoundCell(x, y, startCell, endCell) {
        const cell = this.graph.cells[x][y];
        if (cell === startCell || !cell.weight) return;
        if (!cell.visited) this.visitCell(cell, startCell, endCell);
        this.boundHeap.push(cell);
        this.sortedElements++;
    }

    // Check if the bounding box reaches the map boundary.
    isBoundingBoxAtBoundary() {
        const { xRange, yRange } = this.boundingBox;
        return xRange[0] === 0 && xRange[1] === this.graph.cells.length - 1 &&
            yRange[0] === 0 && yRange[1] === this.graph.cells[0].length - 1;
    }

    // Extend the bounding box one step further.
    extendBoundingbox(step, startCell, endCell) {
        const { xRange, yRange } = this.boundingBox;
        const maxX = this.graph.cells.length - 1;
        const maxY = this.graph.cells[0].length - 1;

        this.boundingBox.xRange = [
            Math.max(0, xRange[0] - step),
            Math.min(maxX, xRange[1] + step)
        ];

        this.boundingBox.yRange = [
            Math.max(0, yRange[0] - step),
            Math.min(maxY, yRange[1] + step)
        ];

        this.updateBoundList(xRange, yRange, this.boundingBox.xRange,
            this.boundingBox.yRange, startCell, endCell);
    }

    // Update the bounding box according to the ellipse.
    extendBoundingBoxEllipse(cost, startCell, endCell) {
        const a2 = cost * cost / 4;
        const [cx, cy] = this.boundingBox.center;
        const halfWidth = Math.sqrt(a2 - this.boundingBox.c2sin2);
        const halfHeight = Math.sqrt(a2 - this.boundingBox.c2cos2);
        const { xRange, yRange } = this.boundingBox;
        const maxX = this.graph.cells.length - 1;
        const maxY = this.graph.cells[0].length - 1;

        this.boundingBox.xRange = [
            Math.max(0, Math.min(xRange[0], Math.floor(cx - halfWidth))),
            Math.min(maxX, Math.max(xRange[1], Math.ceil(cx + halfWidth)))
        ];

        this.boundingBox.yRange = [
            Math.max(0, Math.min(yRange[0], Math.floor(cy - halfHeight))),
            Math.min(maxY, Math.max(yRange[1], Math.ceil(cy + halfHeight)))
        ];

        this.updateBoundList(xRange, yRange, this.boundingBox.xRange,
            this.boundingBox.yRange, startCell, endCell);
    }

    // Parameters of the ellipse's bounding box (c^2 * sin^2)
    cacheEllipseParam(startCell, endCell) {
        const dx = endCell.x - startCell.x;
        const dy = endCell.y - startCell.y;
        return {
            c2sin2: (dy * dy) / 4,
            c2cos2: (dx * dx) / 4
        };
    }

    // Add new nodes to the bound list.
    updateBoundList(xRangeOld, yRangeOld, xRangeNew, yRangeNew, startCell, endCell) {

        const [xMinOld, xMaxOld] = xRangeOld;
        const [yMinOld, yMaxOld] = yRangeOld;
        const [xMinNew, xMaxNew] = xRangeNew;
        const [yMinNew, yMaxNew] = yRangeNew;

        for (let x = xMinNew; x <= xMaxNew; x++)
            for (let y = yMinNew; y <= yMaxNew; y++)
                if (x < xMinOld || x > xMaxOld || y < yMinOld || y > yMaxOld)
                    this.initBoundCell(x, y, startCell, endCell);
    }

    // Elliptical forward expansion + inverted expansion.
    nodeExpansion(currentNode, startCell, endCell) {

        // Inverted expansion
        if (currentNode) {
            const currentCell = this.graph.getCellFromNode(currentNode);
            currentCell.partClosed = true; // Contain closed nodes.

            // The current node serves as a potential parent.
            for (const cell of currentCell.visibleCells) {
                cell.nodes.forEach(node => {
                    if (node.closed) return;
                    this.addPotentialParent(currentNode, node);
                });
            }
            // No need to keep visibleCells
            if (currentCell.nodes.every(node => node.closed)) {
                currentCell.visibleCells = [];
                currentCell.closed = true;
            }
        }

        // Forward expansion
        while (this.openHeap.size() === 0 || this.openHeap.content[0].f === Infinity) {
            this.updateOpenList();
            if (this.boundHeap.size() === 0) {
                if (this.isBoundingBoxAtBoundary()) break;
                this.extendBoundingbox(this.costBuffer * this.speed, startCell, endCell);
            }
        }
        if (this.openHeap.size() > 0 && this.openHeap.content[0].f < Infinity) {
            if (this.openHeap.content[0].f > this.costBound) {
                this.costBound = this.openHeap.content[0].f + this.costBuffer;
                this.extendBoundingBoxEllipse(this.costBound * this.speed, startCell, endCell);
            }
            this.updateOpenList();
        }
    }

    updateOpenList() {
        while (this.boundHeap.size() > 0 && (this.openHeap.size() === 0 ||
            this.boundHeap.content[0].fh <= this.openHeap.content[0].f)) {
            const newCell = this.boundHeap.shift();
            this.invertedScan(newCell);
        }
    }

    // Additional condition check in the findNextClosedNode.
    isCostBounded(realCost) {
        return this.boundHeap.size() === 0 || realCost <= this.boundHeap.content[0].fh;
    }

    // Inverted scanning
    invertedScan(scanCell) {
        this.shadowcast.maxCost = scanCell.fh + this.scanBuffer;
        const visibleCells = this.shadowcast.scan(scanCell);
        this.scannedNodes += visibleCells.length;

        for (const cell of visibleCells) {
            if (cell.fh > scanCell.fh + 1e-10) continue;

            if (!cell.closed) {
                cell.visibleCells.push(scanCell);
                scanCell.visibleCells.push(cell);
            }
            if (!cell.partClosed) continue;

            cell.nodes.forEach(parent => {
                if (!parent.closed) return;
                scanCell.nodes.forEach(node =>
                    this.addPotentialParent(parent, node));
            });
        }
    }
}

module.exports = { ZetastarSIPP_i };