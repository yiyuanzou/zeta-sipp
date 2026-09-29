// Quadrant-based symmetric recursive shadowcasting.
// Adapted from https://gist.github.com/370417/59bb06ced7e740e11ec7dda9d82717f6

class Shadowcast {
    constructor(cells) {
        this.cells = cells;
        this.inverted = false;
        this.maxCost = Infinity;
        this.visible = [];
        this.visitCell;  // Supplied by inverted scanning in Zeta*-SIPP-i.
    }

    // The callback arguments match Zeta*-SIPP-f's scan interface.
    record(origin, cell) {
        this.visible.push(cell);
    }

    axisRecord(origin, cell, i) {
        if (i === 0 || i === 2)
            this.record(origin, cell);
    }

    blocked(cell) {
        return !cell || !cell.weight || !this.inRange(cell);
    }

    // The elliptical boundary is treated as blocked during inverted scanning.
    inRange(cell) {
        if (!this.inverted) return true;
        if (!cell.visited) this.visitCell(cell);
        return cell.fh <= this.maxCost;
    }

    scan(origin) {
        this.visible = [];
        for (let i = 0; i < 4; i++)
            this.quadrantScan(origin, 0, Infinity, 0, Infinity, 0, i);
        return this.visible;
    }

    quadrantScan(origin, deltaX, ymax, ymin, top, bottom, i) {
        quadrantScan(this, this.cells, origin, deltaX, ymax, ymin, top, bottom, i);
    }

    // Ordinary shadowcasting continues immediately in depth-first order.
    // Zeta*-SIPP-f instead schedules the same continuation through its SCAN heap.
    pushScan(origin, deltaX, ymax, ymin, top, bottom, i) {
        this.quadrantScan(origin, deltaX, ymax, ymin, top, bottom, i);
    }
}

// One column per call; the caller handles visibility records and continuations.
// [ymin, ymax] maps the slope range to the grid edge relative to the origin cell.
// Quadrant-based shadowcasting for grid centers.
function quadrantScan(scanner, cells, origin, deltaX, ymax, ymin, top, bottom, i, endCell) {
    const x0 = Math.floor(origin.x);
    const y0 = Math.floor(origin.y);
    const scanUp = i === 0 || i === 1;

    const realx = (i === 0 || i === 3) ? x0 + deltaX : x0 - deltaX;
    const col = cells[realx];
    if (!col) return;
    const ymaxNext = ymax + top;

    // Special case: the first column
    // [ymin, ymax] directly represents the scan range.
    if (deltaX === 0) {
        for (let y = 1; y < ymax; y++) {
            const realy = scanUp ? y0 + y : y0 - y;
            const cell = col[realy];

            if (scanner.blocked(cell)) {
                scanner.pushScan(origin, 1, y, 0, (y - 0.5) / 0.5, 0, i, endCell);
                return;
            }
            scanner.axisRecord(origin, cell, i, endCell);
        }
    }
    // General case
    else {

        // The y-range for grid centers.
        let intervalMin = Math.floor(rounded(deltaX * bottom) + 0.9999999999);
        const intervalMax = Math.round(rounded(deltaX * top) + 0.5) - 1;
        const yEnd = Math.ceil(ymaxNext) - 1;

        let wasBlocked = false;
        for (let y = Math.floor(ymin); y <= yEnd; y++) {
            const realy = scanUp ? y0 + y : y0 - y;
            const cell = col[realy];

            if (scanner.blocked(cell)) {
                if (!wasBlocked) {
                    const newTop = (y - 0.5) / (deltaX + 0.5);
                    if (newTop > bottom)
                        scanner.pushScan(origin, deltaX + 1, y, ymin + bottom, newTop, bottom, i, endCell);
                }
                ymin = y + 1;
                wasBlocked = true;
                if (ymin >= ymax) return;
            }
            else {
                if (wasBlocked) {
                    bottom = (ymin - 0.5) / (deltaX - 0.5);
                    if (intervalMax >= intervalMin)
                        intervalMin = Math.floor(rounded(deltaX * bottom) + 0.9999999999);
                }
                if (intervalMin <= y && y <= intervalMax) {
                    if (y === 0) scanner.axisRecord(origin, cell, i, endCell);
                    else scanner.record(origin, cell, endCell);
                }
                wasBlocked = false;
            }
        }
        if (wasBlocked) bottom = (ymin - 0.5) / (deltaX - 0.5);
    }
    // Push to the next column.
    if (top > bottom)
        scanner.pushScan(origin, deltaX + 1, ymaxNext, ymin + bottom, top, bottom, i, endCell);
}

function rounded(value) {
    const rounded = Math.round(value);
    if (Math.abs(value - rounded) < 1e-10) return rounded;
    return value;
}

module.exports = { Shadowcast, quadrantScan };
