// Quadrant-based symmetric recursive shadowcasting
// Adapted from https://gist.github.com/370417/59bb06ced7e740e11ec7dda9d82717f6

class Shadowcast {
    constructor(cells) {
        this.cells = cells;                           // Grid cells
        this.inverted = false;                        // Indicator for the inverted scanning in Zeta*-SIPP
        this.maxCost = Infinity;                      // Indicate the elliptical boundary in Zeta*-SIPP
        this.visible = [];                            // An array to store the results
        this.visitCell;                               // A function passed from Zeta*-SIPP
    }

    // Record the visible cell
    quadRecord(cell) {
        this.visible.push(cell);
    }

    axisRecord(cell, i) {
        if (i === 0 || i === 2)
            this.visible.push(cell);
    }

    // Check if the cell is blocked
    blocked(cell) {
        if (!cell) return true;
        return !cell.weight || !this.inRange(cell);
    }

    // Check if the cell is within the elliptical range (Zeta*-SIPP)
    inRange(cell) {
        if (!this.inverted) return true;
        if (!cell.visited) this.visitCell(cell);
        return cell.fh <= this.maxCost;
    }

    // Scan the entire space
    scan(origin) {
        this.visible = [];
        // Scan each quadrant
        for (let i = 0; i < 4; i++)
            this.quadrantScan(Math.floor(origin.x), Math.floor(origin.y), 0, Infinity, 0, Infinity, 0, i);
        return this.visible;
    }

    // Quadrant-based shadowcasting for grid centers.
    // [ymin, ymax] is the range obtained by mapping the slope range to the grid edge.
    // [x0, y0] is the lower left corner of the origin cell.
    // Both deltaX and [ymin, ymax] are relative to [x0, y0].
    quadrantScan(x0, y0, deltaX, ymax, ymin, top, bottom, i) {
        const scanUp = i === 0 || i === 1;
        const realx = (i === 0 || i === 3) ? x0 + deltaX : x0 - deltaX;

        const col = this.cells[realx];
        if (!col) return;
        const ymaxNext = ymax + top;

        // Special case: the first column
        // [ymin, ymax] directly represents the scan range.
        if (deltaX === 0) {
            for (let y = 1; y < ymax; y++) {
                const realy = scanUp ? y0 + y : y0 - y;
                const cell = col[realy];

                if (this.blocked(cell)) {
                    this.quadrantScan(x0, y0, 1, y, 0, (y - 0.5) / 0.5, 0, i);
                    return;
                }
                this.axisRecord(cell, i);
            }
        }
        // General case
        else {

            // The y-range for grid centers.
            const interval = [
                Math.floor(this.rounded(deltaX * bottom) + 0.9999999999),
                Math.round(this.rounded(deltaX * top) + 0.5) - 1
            ];

            let wasBlocked = false;
            for (let y = Math.floor(ymin); y <= Math.ceil(ymaxNext) - 1; y++) {
                const realy = scanUp ? y0 + y : y0 - y;
                const cell = col[realy];

                if (this.blocked(cell)) {
                    if (!wasBlocked) {
                        const newTop = (y - 0.5) / (deltaX + 0.5);
                        if (newTop > bottom)
                            this.quadrantScan(x0, y0, deltaX + 1, y, ymin + bottom, newTop, bottom, i);
                    }
                    ymin = y + 1;
                    wasBlocked = true;
                    if (ymin >= ymax) return;
                }
                else {
                    if (wasBlocked) {
                        bottom = (ymin - 0.5) / (deltaX - 0.5);
                        if (interval[1] >= interval[0])
                            interval[0] = Math.floor(this.rounded(deltaX * bottom) + 0.9999999999);
                    }
                    if (interval[0] <= y && y <= interval[1]) {
                        if (y === 0) this.axisRecord(cell, i)
                        else this.quadRecord(cell);
                    }
                    wasBlocked = false;
                }
            }
            if (wasBlocked) bottom = (ymin - 0.5) / (deltaX - 0.5);
        }
        // Push to the next column.
        if (top > bottom)
            this.quadrantScan(x0, y0, deltaX + 1, ymaxNext, ymin + bottom, top, bottom, i);
    }

    rounded(value) {
        const rounded = Math.round(value);
        if (Math.abs(value - rounded) < 1e-10) return rounded;
        return value;
    }
}

module.exports = { Shadowcast };