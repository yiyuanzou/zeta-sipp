const { TOAASIPP } = require("./to-aa-sipp");
const { quadrantScan } = require("../shadowcasting");
const { MinHeap } = require("../binaryHeap");

class ZetastarSIPP_f extends TOAASIPP {
    constructor(graph, speed) {
        super(graph, speed);
        this.scanHeap = new MinHeap("fLow");                                         // Scan heap for storing scan ranges.
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

        this.nodeExpansion(startNode, startCell, endCell);
        return [startCell, endCell];
    }

    // Elliptical forward expansion.
    nodeExpansion(currentNode, _, endCell) {
        if (currentNode)
            this.initScanRange(currentNode, endCell);
        this.forwardScan(endCell);
    }

    // Additional condition check in the findNextClosedNode.
    isCostBounded(realCost) {
        return this.scanHeap.size() === 0 || realCost <= this.scanHeap.content[0].fLow;
    }

    // Forward scanning from a closed node.
    forwardScan(endCell) {
        while (this.scanHeap.size() > 0 && (this.openHeap.size() === 0 ||
            this.scanHeap.content[0].fLow <= this.openHeap.content[0].f)) {
            const range = this.scanHeap.shift();
            this.quadrantScan(range.origin, range.deltaX, range.ymax, range.ymin, range.top, range.bottom, range.quadrant, endCell);
        }
    }

    // Initialize the scan range.
    initScanRange(origin, endCell) {
        for (let i = 0; i < 4; i++)
            this.quadrantScan(origin, 0, Infinity, 0, Infinity, 0, i, endCell);
    }

    // Use the same geometry as ordinary and inverted shadowcasting.
    quadrantScan(origin, deltaX, ymax, ymin, top, bottom, i, endCell) {
        quadrantScan(this, this.graph.cells, origin, deltaX, ymax, ymin, top, bottom, i, endCell);
    }

    // Add the scan range to the scan heap.
    pushScan(origin, deltaX, ymax, ymin, top, bottom, i, endCell) {
        const cost = ScanRange.computeCost(origin, deltaX, ymax, ymin, i, this.speed, endCell);
        if (cost === -1) this.quadrantScan(origin, deltaX, ymax, ymin, top, bottom, i, endCell);
        else {
            this.scanHeap.push(new ScanRange(origin, deltaX, ymax, ymin, top, bottom, i, cost));
            this.sortedElements++;
        }
        this.scannedNodes += Math.ceil(ymax - ymin);
    }

    // Inverted expansion.
    record(origin, cell, endCell) {
        if (!cell.visited) this.visitCell(cell, endCell);

        for (const node of cell.nodes) {
            if (node.closed) continue;
            this.addPotentialParent(origin, node);
        }
    }

    axisRecord(origin, cell, i, endCell) {
        if (i === 0 || i === 2)
            this.record(origin, cell, endCell);
    }

    // Check if the cell is blocked
    blocked(cell) {
        return !cell || !cell.weight;
    }
}

class ScanRange {
    constructor(origin, deltaX, ymax, ymin, top, bottom, i, cost) {
        this.origin = origin;
        this.deltaX = deltaX;
        this.ymax = ymax;
        this.ymin = ymin;
        this.top = top;
        this.bottom = bottom;
        this.quadrant = i;
        this.fLow = cost;
    }

    // Anya's heuristic function.
    static computeCost(origin, deltaX, ymax, ymin, i, speed, endCell) {
        const dx = deltaX - 0.5;
        const yminNew = ymin - 0.5;
        const ymaxNew = ymax - 0.5;

        let endX, endY;

        switch (i) {
            case 0:
                endX = endCell.x - origin.x;
                endY = endCell.y - origin.y;
                break;
            case 1:
                endX = origin.x - endCell.x;
                endY = endCell.y - origin.y;
                break;
            case 2:
                endX = origin.x - endCell.x;
                endY = origin.y - endCell.y;
                break;
            case 3:
                endX = endCell.x - origin.x;
                endY = origin.y - endCell.y;
                break;
        }

        const mirroredEndX = endX >= dx ? endX : 2 * dx - endX;
        const mirroredEndY = endY;

        const intersection = mirroredEndX === 0 ? mirroredEndY : (mirroredEndY / mirroredEndX) * dx;
        if (intersection >= yminNew && intersection <= ymaxNew) {
            if (endX > dx) return -1;
            return Math.sqrt(mirroredEndX * mirroredEndX + mirroredEndY * mirroredEndY) / speed + origin.g;
        }

        const scanY = intersection > ymaxNew ? ymaxNew : yminNew;

        const t1 = Math.sqrt(dx * dx + scanY * scanY) / speed;
        const dx2 = mirroredEndX - dx;
        const dy2 = mirroredEndY - scanY;
        const t2 = Math.sqrt(dx2 * dx2 + dy2 * dy2) / speed;

        return t1 + t2 + origin.g;
    }
}

module.exports = { ZetastarSIPP_f };
