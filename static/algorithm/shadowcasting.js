// Quadrant-based symmetric recursive shadowcasting
// Adapted from https://gist.github.com/370417/59bb06ced7e740e11ec7dda9d82717f6

class Shadowcast {
    constructor(graph) {
        this.graph = graph;                           // Grid
        this.inverted = false;                        // Indicator for the inverted scanning in Zeta*
        this.maxCost = Infinity;                      // Indicate the elliptical boundary in Zeta*
        this.depth = Infinity;                        // Scan depth
        this.visible = [];                            // An array to store the results
        this.axisVisible = new Set();                 // A set used to store nodes on the axis.
        this.startNode;                               // Start node
        this.endNode;                                 // End node

        // The rightdown and rightup cells of each quadrant.
        this.rightDown = [graph.rightdown, graph.leftdown, graph.leftup, graph.rightup];
        this.rightUp = [graph.rightup, graph.leftup, graph.leftdown, graph.rightdown];
    }

    // Record the visible node
    quadRecord(node) {
        if (!node) return;
        this.visible.push(node);
    }

    quadRecordCorner(node) {
        if (!node) return;
        if (this.isCornerPoint(node.x, node.y) || node === this.startNode || node === this.endNode)
            this.visible.push(node);
    }

    axisRecord(node) {
        if (!node) return;
        this.axisVisible.add(node);
    }

    axisRecordCorner(node) {
        if (!node) return;
        if (this.isCornerPoint(node.x, node.y) || node === this.startNode || node === this.endNode)
            this.axisVisible.add(node);
    }

    euclidean(node1, node2) {
        const d1 = node2.x - node1.x;
        const d2 = node2.y - node1.y;
        return Math.sqrt(d1 * d1 + d2 * d2);
    }

    // Filter quadrants according to taut paths
    quadrantIdx(origin) {
        if (!this.inverted || origin === this.endNode) return [0, 1, 2, 3];

        const x = origin.x;
        const y = origin.y;

        if (this.isLeftDownBlocked(x, y) || this.isRightUpBlocked(x, y)) return [1, 3];
        if (this.isLeftUpBlocked(x, y) || this.isRightDownBlocked(x, y)) return [0, 2];

        return [];
    }

    // Scan the entire space
    scan(origin) {
        this.scannedNodes = 0;
        this.visible.length = 0;
        this.axisVisible.clear();
        const indices = this.quadrantIdx(origin);

        // Scan each quadrant
        if (this.inverted)
            for (const i of indices)
                this.invertedScan(origin, 0, Infinity, 0, Infinity, 0, i);
        else
            for (const i of indices)
                this.quadrantScan(origin, 0, Infinity, 0, Infinity, 0, i);

        // Axis nodes
        for (const node of this.axisVisible)
            this.visible.push(node);
        return this.visible;
    }

    // Quadrant-based shadowcasting: compute the unblocked slope range for the next column.
    quadrantScan(origin, deltaX, ymax, ymin, top, bottom, i) {
        if (deltaX > this.depth || ymin > this.depth) return;
        ymax = this.rounded(Math.min(ymax, this.depth));
        if (top === null) top = ymax / deltaX;
        ymin = this.rounded(ymin);

        const scanUp = i === 0 || i === 1;
        const realx = (i === 0 || i === 3) ? origin.x + deltaX : origin.x - deltaX;
        const col = this.graph.vertices[realx];
        if (!col) return;

        const ymaxNext = Math.min(ymax + top, this.depth);
        const yStart = Math.ceil(ymin);
        const yEnd = scanUp
            ? Math.min(Math.ceil(ymaxNext), this.graph.length - origin.y)
            : Math.min(Math.ceil(ymaxNext), origin.y);
        const rightDownBits = this.rightDown[i][realx];

        // Special case: the first column
        if (deltaX === 0) {
            let y = ymin === 0 ? 1 : yStart;
            while (y <= yEnd) {
                const realy = scanUp ? origin.y + y : origin.y - y;
                const node = col[realy];
                if (!node || this.bit(rightDownBits, realy)) {
                    this.lowerTopSlope(origin, deltaX, y - 1, ymin + bottom, bottom, i);
                    this.scannedNodes += Math.ceil(y - ymin);
                    return;
                }
                if (y <= ymax) this.axisRecord(node);
                y++;
            }
        }

        // General case
        else {
            let wasBlocked = false;
            let y = yStart;

            // No need to push the slope range into the next column.
            if (deltaX >= this.depth) {
                while (y <= yEnd) {
                    const realy = scanUp ? origin.y + y : origin.y - y;
                    const node = col[realy];
                    if (y <= ymax) {
                        if (y === 0) this.axisRecord(node);
                        else this.quadRecord(node);
                    }
                    y++;
                }
                return;
            }

            // Normal cases
            while (y <= yEnd) {
                const realy = scanUp ? origin.y + y : origin.y - y;
                const node = col[realy];
                if (y <= ymax) {
                    if (y === 0) {
                        this.axisRecord(node);
                        y++;
                        continue;
                    }
                    this.quadRecord(node);
                }
                if (!node || this.bit(rightDownBits, realy)) {
                    if (!wasBlocked)
                        this.lowerTopSlope(origin, deltaX, y - 1, ymin + bottom, bottom, i);
                    ymin = y;
                    wasBlocked = true;
                }
                else {
                    if (wasBlocked) bottom = ymin / deltaX;
                    wasBlocked = false;
                }
                y++;
            }
            if (wasBlocked) bottom = ymin / deltaX;
        }

        // No need to push the slope range into the next column.
        if (deltaX >= this.depth) return;

        // Push to the next column.
        const yminNext = ymin + bottom;
        const ymaxNextNew = Math.min(ymaxNext, yEnd);
        this.lowerTopSlope(origin, deltaX, ymaxNextNew, yminNext, bottom, i);
    }

    // Quadrant-based shadowcasting for inverted scanning
    invertedScan(origin, deltaX, ymax, ymin, top, bottom, i) {
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

        // Bitmask for startNode and endNode
        let startNormY = Infinity;
        let startWord = -1;
        let startBitMask = 0;
        let endNormY = Infinity;
        let endWord = -1;
        let endBitMask = 0;

        if (this.startNode && this.startNode.x === realx) {
            startNormY = scanUp ? this.startNode.y - origin.y : origin.y - this.startNode.y;
            startWord = this.startNode.y >> 5;
            startBitMask = 1 << (this.startNode.y & 31);
        }
        if (this.endNode && this.endNode.x === realx) {
            endNormY = scanUp ? this.endNode.y - origin.y : origin.y - this.endNode.y;
            endWord = this.endNode.y >> 5;
            endBitMask = 1 << (this.endNode.y & 31);
        }

        const ymaxNext = ymax + top;
        let yEnd = scanUp
            ? Math.min(Math.ceil(ymaxNext), this.graph.length - origin.y)
            : Math.min(Math.ceil(ymaxNext), origin.y);
        let yStart = Math.ceil(ymin);

        // Compute the column range bounded by the ellipse.
        const [ellipseMinY, ellipseMaxY] = this.ellipseYBoundsForColumn(realx);
        const normMin = scanUp ? ellipseMinY - origin.y : origin.y - ellipseMaxY;
        const normMax = scanUp ? ellipseMaxY - origin.y : origin.y - ellipseMinY;
        yStart = Math.max(yStart, Math.ceil(normMin));
        yEnd = Math.min(yEnd, Math.floor(normMax));

        if (yEnd < yStart) return;

        // Special case: the first column
        if (deltaX === 0) {
            const yminLimit = ymin === 0 ? 1 : yStart;

            // Next event
            let y = this.nextEventY(origin, yminLimit, yEnd, scanUp, rightDownBits, rightUpBits, cornerBits,
                startNormY, startWord, startBitMask, endNormY, endWord, endBitMask);

            while (y <= yEnd) {
                const realy = scanUp ? origin.y + y : origin.y - y;
                const node = col[realy];
                if (!node || this.bit(rightDownBits, realy)) {
                    this.lowerTopSlopeInverted(origin, deltaX, y - 1, ymin + bottom, bottom, i);
                    this.scannedNodes += Math.ceil(y - ymin);
                    return;
                }
                if (y <= ymax) this.axisRecordCorner(node);

                // Next event
                y = this.nextEventY(origin, y + 1, yEnd, scanUp, rightDownBits, rightUpBits, cornerBits,
                    startNormY, startWord, startBitMask, endNormY, endWord, endBitMask);
            }
        }

        // General case
        else {
            let wasBlocked = false;

            // Next event
            let y = this.nextEventY(origin, yStart, yEnd, scanUp, rightDownBits, rightUpBits, cornerBits,
                startNormY, startWord, startBitMask, endNormY, endWord, endBitMask);

            while (y <= yEnd) {
                const realy = scanUp ? origin.y + y : origin.y - y;
                const node = col[realy];
                if (y <= ymax) {
                    if (y === 0) {
                        this.axisRecordCorner(node);
                        y = this.nextEventY(origin, y + 1, yEnd, scanUp, rightDownBits, rightUpBits, cornerBits,
                            startNormY, startWord, startBitMask, endNormY, endWord, endBitMask);
                        continue;
                    }
                    this.quadRecordCorner(node);
                }
                if (!node || this.bit(rightDownBits, realy)) {
                    if (!wasBlocked)
                        this.lowerTopSlopeInverted(origin, deltaX, y - 1, ymin + bottom, bottom, i);
                    ymin = y;
                    wasBlocked = true;
                }
                else {
                    if (wasBlocked) bottom = ymin / deltaX;
                    wasBlocked = false;
                }
                // Next event
                y = this.nextEventY(origin, y + 1, yEnd, scanUp, rightDownBits, rightUpBits, cornerBits,
                    startNormY, startWord, startBitMask, endNormY, endWord, endBitMask);
            }
            if (wasBlocked) bottom = ymin / deltaX;
        }

        // Push to the next column.
        const yminNext = ymin + bottom;
        const ymaxNextNew = Math.min(ymaxNext, yEnd);
        this.lowerTopSlopeInverted(origin, deltaX, ymaxNextNew, yminNext, bottom, i);
    }

    // Compute the min/max Y (in real grid coordinates) for a given column `realx`
    // such that the point (realx, realy) lies inside the ellipse defined by:
    // dist(start, p) + dist(p, end) <= maxCost
    ellipseYBoundsForColumn(realx) {

        // If not using elliptical constraint, allow full column
        if (!this.inverted) return [-Infinity, Infinity];

        const startX = this.startNode.x;
        const startY = this.startNode.y;
        const endX = this.endNode.x;
        const endY = this.endNode.y;
        const maxDistance = this.maxCost;

        // Vector from start to end
        const dx = endX - startX;
        const dy = endY - startY;

        // Distance between the two foci
        const focalDistance = Math.sqrt(dx * dx + dy * dy);

        // If maxCost is smaller than distance between foci, ellipse is empty
        if (maxDistance < focalDistance) {
            return [Infinity, -Infinity];
        }

        // Special case: start and end are the same point → circle
        if (focalDistance === 0) {
            const radius = maxDistance / 2;

            const offsetX = realx - startX;
            const remaining = radius * radius - offsetX * offsetX;

            if (remaining < 0) return [Infinity, -Infinity];

            const yOffset = Math.sqrt(remaining);
            return [startY - yOffset, startY + yOffset];
        }

        // Center of ellipse (midpoint between start and end)
        const centerX = (startX + endX) / 2;
        const centerY = (startY + endY) / 2;

        // Semi-major axis (half total distance)
        const semiMajor = maxDistance / 2;

        // Semi-minor axis squared: b² = a² - (c/2)²
        const semiMinorSq = semiMajor * semiMajor - (focalDistance / 2) * (focalDistance / 2);

        if (semiMinorSq < 0) {
            return [Infinity, -Infinity];
        }

        // Unit vector along the major axis (start → end)
        const unitX = dx / focalDistance;
        const unitY = dy / focalDistance;

        // Offset of this column from ellipse center
        const columnOffsetX = realx - centerX;

        // Coefficients of quadratic in Y
        const A = (unitY * unitY) / (semiMajor * semiMajor) + (unitX * unitX) / semiMinorSq;
        const B = (2 * columnOffsetX * unitX * unitY) / (semiMajor * semiMajor) - (2 * columnOffsetX * unitY * unitX) / semiMinorSq;
        const C = (columnOffsetX * unitX) * (columnOffsetX * unitX) / (semiMajor * semiMajor) + (-columnOffsetX * unitY) * (-columnOffsetX * unitY) / semiMinorSq - 1;

        // Solve quadratic A*Y² + B*Y + C = 0
        const discriminant = B * B - 4 * A * C;

        // No intersection with this column
        if (discriminant < 0)
            return [Infinity, -Infinity];

        const sqrtDisc = Math.sqrt(discriminant);

        // Two intersection points (relative to centerY)
        const yOffset1 = (-B - sqrtDisc) / (2 * A);
        const yOffset2 = (-B + sqrtDisc) / (2 * A);

        // Convert back to real grid Y coordinates
        const yMin = yOffset1 + centerY;
        const yMax = yOffset2 + centerY;

        return [yMin, yMax];
    }

    // Return the next normalized y that has an event.
    // y and yEnd are normalized scan-space coordinates.
    // Packed bits are stored in real grid coordinates.
    nextEventY(origin, y, yEnd, scanUp, rightDownBits, rightUpBits, cornerBits, startNormY, startWord, startBitMask, endNormY, endWord, endBitMask) {

        // Convert normalized bounds into real grid y.
        let realy = scanUp ? origin.y + y : origin.y - y;
        const realEnd = scanUp ? origin.y + yEnd : origin.y - yEnd;

        // Quadrants 0 and 1: normalized y increases as realy increases.
        if (scanUp) {
            while (realy <= realEnd) {
                const word = realy >> 5;
                const bit = realy & 31;

                const startMask = (word === startWord && startNormY >= y && startNormY <= yEnd) ? startBitMask : 0;
                const endMask = (word === endWord && endNormY >= y && endNormY <= yEnd) ? endBitMask : 0;

                // Keep only event bits at or above the current bit.
                let mask = (rightDownBits[word] | rightUpBits[word] | cornerBits[word] | startMask | endMask) &
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

                const startMask = (word === startWord && startNormY >= y && startNormY <= yEnd) ? startBitMask : 0;
                const endMask = (word === endWord && endNormY >= y && endNormY <= yEnd) ? endBitMask : 0;

                // Keep only event bits at or below the current bit.
                let mask = (rightDownBits[word] | rightUpBits[word] | cornerBits[word] | startMask | endMask) &
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

    // Create a bitmask where all bits from 0 up to "bit" are 1.
    // If bit === 5, then it returns 00111111.
    maskUpTo(bit) {
        return 0xffffffff >>> (31 - bit);
    }

    maskFrom(bit) {
        return 0xffffffff << bit;
    }

    rounded(value) {
        const rounded = Math.round(value);
        if (Math.abs(value - rounded) < 1e-10) return rounded;
        return value;
    }

    // Lower top slope for the next column.
    lowerTopSlope(origin, deltaX, ymaxNext, yminNext, bottom, i) {
        if (ymaxNext <= 0 || yminNext >= ymaxNext) return;
        this.scannedNodes += Math.ceil(ymaxNext - yminNext);
        this.quadrantScan(origin, deltaX + 1, ymaxNext, yminNext, null, bottom, i);
    }

    lowerTopSlopeInverted(origin, deltaX, ymaxNext, yminNext, bottom, i) {
        if (ymaxNext <= 0 || yminNext >= ymaxNext) return;
        this.scannedNodes += Math.ceil(ymaxNext - yminNext);
        this.invertedScan(origin, deltaX + 1, ymaxNext, yminNext, null, bottom, i);
    }
}

module.exports = { Shadowcast };