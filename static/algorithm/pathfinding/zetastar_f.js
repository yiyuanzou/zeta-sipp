const { Astar } = require("./astar");
const { BinaryHeap } = require("../binaryHeap");

class Zetastar_f extends Astar {
    constructor(graph) {
        super(graph);                                                             // Class inherits from A*.
        this.scanHeap = new BinaryHeap(range => range.f);                         // Scan heap for storing scan ranges.

        // The rightdown and rightup cells of each quadrant.
        this.rightDown = [graph.rightdown, graph.leftdown, graph.leftup, graph.rightup];
        this.rightUp = [graph.rightup, graph.leftup, graph.leftdown, graph.rightdown];
    }

    heuristic(node1, node2) {
        return this.euclidean(node1, node2);
    }

    // Elliptical forward expansion.
    forwardExpansion(currentNode, _, endNode) {
        this.initScanRange(currentNode, endNode);
        this.forwardScan(endNode);
    }

    forwardScan(endNode) {
        while (this.scanHeap.size() > 0 && (this.openHeap.size() === 0 ||
            this.scanHeap.content[0].f <= this.openHeap.content[0].f)) {
            const range = this.scanHeap.shift();
            this.quadrantScan(range.origin, range.deltaX, range.ymax, range.ymin, range.top, range.bottom, range.quadrant, endNode);
        }
    }

    // Initialize the scan range.
    initScanRange(node, endNode) {

        // General case
        if (!node.parent) {
            if (!this.isRightUpBlocked(node.x, node.y))
                this.quadrantScan(node, 0, Infinity, 0, Infinity, 0, 0, endNode);
            if (!this.isLeftUpBlocked(node.x, node.y))
                this.quadrantScan(node, 0, Infinity, 0, Infinity, 0, 1, endNode);
            if (!this.isLeftDownBlocked(node.x, node.y))
                this.quadrantScan(node, 0, Infinity, 0, Infinity, 0, 2, endNode);
            if (!this.isRightDownBlocked(node.x, node.y))
                this.quadrantScan(node, 0, Infinity, 0, Infinity, 0, 3, endNode);
        }

        // Narrow the scan ranges according to taut paths.
        else {
            const dx = node.parent.x - node.x;
            const dy = node.parent.y - node.y;
            let slope = dx !== 0 ? Math.abs(dy / dx) : Infinity;

            let idx, bottom, top;
            if (this.isLeftUpBlocked(node.x, node.y))
                [idx, bottom, top] = dx < 0 || dy < 0 ? [0, slope, Infinity] : [2, 0, slope];
            else if (this.isRightDownBlocked(node.x, node.y))
                [idx, bottom, top] = dx < 0 || dy < 0 ? [0, 0, slope] : [2, slope, Infinity];
            else if (this.isLeftDownBlocked(node.x, node.y))
                [idx, bottom, top] = dx < 0 || dy > 0 ? [3, slope, Infinity] : [1, 0, slope];
            else[idx, bottom, top] = dx < 0 || dy > 0 ? [3, 0, slope] : [1, slope, Infinity];

            if (idx === undefined) return;
            this.quadrantScan(node, 0, top === Infinity ? Infinity : 0, 0, top, bottom, idx, endNode);
        }
    }

    // Quadrant-based shadowcasting: compute the unblocked slope range for the next column.
    quadrantScan(origin, deltaX, ymax, ymin, top, bottom, i, endNode) {
        if (top === null) top = ymax / deltaX;
        ymax = this.rounded(ymax);
        ymin = this.rounded(ymin);
        const scanUp = i === 0 || i === 1;

        const realx = (i === 0 || i === 3) ? origin.x + deltaX : origin.x - deltaX;
        const col = this.graph.vertices[realx];
        if (!col) return;

        // Bit-packing grids
        const rightDownBits = this.rightDown[i][realx];
        const rightUpBits = this.rightUp[i][realx];
        const cornerBits = this.graph.isCorner[realx];

        // Bitmask for endNode
        let endNormY = Infinity;
        let endWord = -1;
        let endBitMask = 0;

        if (endNode && endNode.x === realx) {
            endNormY = scanUp
                ? endNode.y - origin.y
                : origin.y - endNode.y;

            endWord = endNode.y >> 5;
            endBitMask = 1 << (endNode.y & 31);
        }

        const ymaxNext = ymax + top;
        const maxRealY = (cornerBits.length << 5) - 1;
        const yEnd = scanUp
            ? Math.min(Math.ceil(ymaxNext), maxRealY - origin.y)
            : Math.min(Math.ceil(ymaxNext), origin.y);
        const yStart = Math.ceil(ymin);

        if (yEnd < yStart) return;

        // Special case: the first column
        if (deltaX === 0) {
            const yminLimit = ymin === 0 ? 1 : yStart;

            // Next event
            let y = this.nextEventY(origin, yminLimit, yEnd, scanUp, rightDownBits, rightUpBits, cornerBits, endNormY, endWord, endBitMask);

            while (y <= yEnd) {
                const realy = scanUp ? origin.y + y : origin.y - y;
                const isBlocked = this.bit(rightDownBits, realy);

                if (isBlocked) {
                    this.lowerTopSlope(origin, deltaX, y - 1, ymin + bottom, bottom, i, endNode);
                    this.scannedNodes += Math.ceil(y - ymin);
                    return;
                }
                if (y <= ymax) this.record(origin, col[realy], endNode);

                // Next event
                y = this.nextEventY(origin, y + 1, yEnd, scanUp, rightDownBits, rightUpBits, cornerBits, endNormY, endWord, endBitMask);
            }
        }

        // General case
        else {
            let wasBlocked = false;

            // Next event
            let y = this.nextEventY(origin, yStart, yEnd, scanUp, rightDownBits, rightUpBits, cornerBits, endNormY, endWord, endBitMask);

            while (y <= yEnd) {
                const realy = scanUp ? origin.y + y : origin.y - y;

                if (y <= ymax) this.record(origin, col[realy], endNode);

                if (y === 0) {
                    y = this.nextEventY(origin, y + 1, yEnd, scanUp, rightDownBits, rightUpBits, cornerBits, endNormY, endWord, endBitMask);
                    continue;
                }

                const isBlocked = this.bit(rightDownBits, realy);
                if (isBlocked) {
                    if (!wasBlocked)
                        this.lowerTopSlope(origin, deltaX, y - 1, ymin + bottom, bottom, i, endNode);
                    ymin = y;
                    wasBlocked = true;
                }
                else {
                    if (wasBlocked) bottom = ymin / deltaX;
                    wasBlocked = false;
                }

                // Next event
                y = this.nextEventY(origin, y + 1, yEnd, scanUp, rightDownBits, rightUpBits, cornerBits, endNormY, endWord, endBitMask);
            }

            if (wasBlocked) bottom = ymin / deltaX;
        }

        // Push to the next column.
        const yminNext = ymin + bottom;
        const ymaxNextNew = Math.min(ymaxNext, yEnd);
        this.lowerTopSlope(origin, deltaX, ymaxNextNew, yminNext, bottom, i, endNode);
    }

    // Return the next normalized y that has an event.
    // y and yEnd are normalized scan-space coordinates.
    // Packed bits are stored in real grid coordinates.
    nextEventY(origin, y, yEnd, scanUp, rightDownBits, rightUpBits, cornerBits, endNormY, endWord, endBitMask) {

        // Convert normalized bounds into real grid y.
        let realy = scanUp ? origin.y + y : origin.y - y;
        const realEnd = scanUp ? origin.y + yEnd : origin.y - yEnd;

        // Quadrants 0 and 1: normalized y increases as realy increases.
        if (scanUp) {
            while (realy <= realEnd) {
                const word = realy >> 5;
                const bit = realy & 31;
                const endMask = (word === endWord && endNormY >= y && endNormY <= yEnd) ? endBitMask : 0;

                // Keep only event bits at or above the current bit.
                let mask = (rightDownBits[word] | rightUpBits[word] | cornerBits[word] | endMask) &
                    this.maskFrom(bit);

                // Clamp the final word so we do not scan past realEnd.
                if (word === (realEnd >> 5))
                    mask &= this.maskUpTo(realEnd & 31);

                // No event in this word; jump to the next word.
                if (mask === 0) {
                    realy = (word + 1) << 5;
                    continue;
                }

                // Lowest set bit = next event.
                const lowest = mask & -mask;
                const hitBit = 31 - Math.clz32(lowest);
                const hitRealY = (word << 5) + hitBit;

                // Convert real y back to normalized y.
                return hitRealY - origin.y;
            }
        }

        // Quadrants 2 and 3: normalized y increases as realy decreases.
        else {
            while (realy >= realEnd) {
                const word = realy >> 5;
                const bit = realy & 31;
                const endMask = (word === endWord && endNormY >= y && endNormY <= yEnd) ? endBitMask : 0;

                // Keep only event bits at or below the current bit.
                let mask = (rightDownBits[word] | rightUpBits[word] | cornerBits[word] | endMask) &
                    this.maskUpTo(bit);

                // Clamp the final word so we do not scan past realEnd.
                if (word === (realEnd >> 5))
                    mask &= this.maskFrom(realEnd & 31);

                // No event in this word; jump to the previous word.
                if (mask === 0) {
                    realy = (word << 5) - 1;
                    continue;
                }

                // Highest set bit = next event while scanning downward.
                const hitBit = 31 - Math.clz32(mask);
                const hitRealY = (word << 5) + hitBit;

                // Convert real y back to normalized y.
                return origin.y - hitRealY;
            }
        }

        // No more events in [y, yEnd].
        return Infinity;
    }

    rounded(value) {
        const rounded = Math.round(value);
        if (Math.abs(value - rounded) < 1e-10) return rounded;
        return value;
    }

    // Create a bitmask where all bits from 0 up to "bit" are 1.
    // If bit === 5, then it returns 00111111.
    maskUpTo(bit) {
        return 0xffffffff >>> (31 - bit);
    }

    maskFrom(bit) {
        return 0xffffffff << bit;
    }

    // Record the visible node.
    record(origin, node, endNode) {
        if (node && !node.closed && this.isTautPath(origin, node, endNode))
            this.updateParent(origin, node, endNode);
    }

    // Add the scan range to the scan heap.
    pushScan(origin, deltaX, ymax, ymin, top, bottom, i, endNode) {
        const cost = ScanRange.computeCost(origin, deltaX, ymax, ymin, i, endNode);
        if (cost === -1) this.quadrantScan(origin, deltaX, ymax, ymin, top, bottom, i, endNode);
        else {
            this.scanHeap.push(new ScanRange(origin, deltaX, ymax, ymin, top, bottom, i, cost));
            this.sortedElements++;
        }
        this.scannedNodes += Math.ceil(ymax - ymin);
    }

    // Lower top slope for the next column.
    lowerTopSlope(origin, deltaX, ymaxNext, yminNext, bottom, i, endNode) {
        if (ymaxNext <= 0 || yminNext >= ymaxNext) return;
        this.pushScan(origin, deltaX + 1, ymaxNext, yminNext, null, bottom, i, endNode);
    }

    // Check whether the path from node1 to node2 is taut.
    isTautPath(node1, node2, endNode) {
        if (node2 === endNode) return true;
        if (!this.isCornerPoint(node2.x, node2.y)) return false;

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

class ScanRange {
    constructor(origin, deltaX, ymax, ymin, top, bottom, i, cost) {
        this.origin = origin;
        this.deltaX = deltaX;
        this.ymax = ymax;
        this.ymin = ymin;
        this.top = top;
        this.bottom = bottom;
        this.quadrant = i;
        this.f = cost;
    }

    // Anya's heuristic function.
    // If the end node falls within the slope range, the cost of the next scan will not increase.  
    // Therefore, instead of inserting it into the scan heap, push it directly.
    static computeCost(origin, deltaX, ymax, ymin, i, endNode) {
        let endX, endY;

        switch (i) {
            case 0:
                endX = endNode.x - origin.x;
                endY = endNode.y - origin.y;
                break;
            case 1:
                endX = origin.x - endNode.x;
                endY = endNode.y - origin.y;
                break;
            case 2:
                endX = origin.x - endNode.x;
                endY = origin.y - endNode.y;
                break;
            case 3:
                endX = endNode.x - origin.x;
                endY = origin.y - endNode.y;
                break;
        }

        const mirroredEndX = endX >= deltaX ? endX : 2 * deltaX - endX;
        const mirroredEndY = endY;

        const intersection = mirroredEndX === 0 ? mirroredEndY : (mirroredEndY / mirroredEndX) * deltaX;
        if (intersection >= ymin && intersection <= ymax) {
            if (endX > deltaX) return -1;
            return Math.sqrt(mirroredEndX * mirroredEndX + mirroredEndY * mirroredEndY) + origin.g;
        }

        const scanY = intersection > ymax ? ymax : ymin;

        const d1 = Math.sqrt(deltaX * deltaX + scanY * scanY);
        const dx2 = mirroredEndX - deltaX;
        const dy2 = mirroredEndY - scanY;
        const d2 = Math.sqrt(dx2 * dx2 + dy2 * dy2);

        return d1 + d2 + origin.g;
    }
}

module.exports = { Zetastar_f };