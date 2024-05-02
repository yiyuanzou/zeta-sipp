// Symmetric recursive shadowcasting
// Adapted from https://gist.github.com/370417/59bb06ced7e740e11ec7dda9d82717f6
class Shadowcast {
    constructor(grids) {
        this.grids = grids;                           // The grids generated in pathfinding
        this.inverted = false;                        // Indicator for the inverted scanning in Zeta*-SIPP
        this.maxCost = Infinity;                      // Indicate the elliptical boundary in Zeta*-SIPP
        this.visible = [];                            // An array to store the results
        this.transforms = [                           // Transforms the slope range of each octant into [0,1]
            { xx: 1, xy: 0, yx: 0, yy: 1 },
            { xx: 1, xy: 0, yx: 0, yy: -1 },
            { xx: -1, xy: 0, yx: 0, yy: 1 },
            { xx: -1, xy: 0, yx: 0, yy: -1 },
            { xx: 0, xy: 1, yx: 1, yy: 0 },
            { xx: 0, xy: -1, yx: 1, yy: 0 },
            { xx: 0, xy: 1, yx: -1, yy: 0 },
            { xx: 0, xy: -1, yx: -1, yy: 0 }
        ];
    }

    // Find the node according to the coordinates
    find(x, y) {
        const row = this.grids[Math.floor(x)];
        if (!row) return null;
        const node = row[Math.floor(y)];
        return node;
    }

    // Record the visible node
    record(node) {
        this.visible.push(node);
    }

    // Check if the node is blocked
    blocked(node) {
        if (!node) return true;
        return !node.weight || !this.inRange(node);
    }

    // Check if the node is within the elliptical range (Zeta*-SIPP)
    inRange(node) {
        if (!this.inverted) return true;
        return node.fh <= this.maxCost;
    }

    // Scan the entire space
    scan(origin) {
        this.visible = [];
        // Scan each octant
        for (let i = 0; i < 8; i++)
            this.compute(origin, 1, 1, 0, this.transforms[i]);
        return [...new Set(this.visible)];
    }

    /**
     * Scan each slope range at each column.
     * @param origin - scanning origin
     * @param deltaX - current column
     * @param topSlope - top slope
     * @param bottomSlope - bottom slope
     * @param transform - transformed into the real coordinates.
    */
    compute(origin, deltaX, topSlope, bottomSlope, transform) {
        if (bottomSlope >= topSlope) return;

        let ymin = deltaX * bottomSlope;
        const ymax = deltaX * topSlope;
        for (let y = Math.floor(ymin); y <= Math.ceil(ymax); y++) {

            let wasBlocked = false;
            const [realx, realy] = this.transformCoords(origin, deltaX, y, transform);
            const node = this.find(realx, realy);
            if (!this.blocked(node)) {
                if (y >= ymin && y <= ymax)
                    this.record(node);
                wasBlocked = false;
            }
            else {
                // Check if a new slope range needs to be generated for the next column.
                if (!wasBlocked) {
                    const newTopSlope = (y - 0.5) / (deltaX + 0.5);
                    this.compute(origin, deltaX + 1, Math.min(newTopSlope, topSlope), bottomSlope, transform);
                    wasBlocked = true;
                }
                const newBottomSlope = (y + 0.5) / (deltaX - 0.5);
                bottomSlope = Math.max(bottomSlope, newBottomSlope);
                if (bottomSlope >= topSlope) return;
                // Update the current visible range
                ymin = deltaX * bottomSlope;
            }
        }
        // Push the current slope range into the next column.
        this.compute(origin, deltaX + 1, topSlope, bottomSlope, transform);
    }

    // Transformed into the real coordinates.
    transformCoords(origin, x, y, transform) {
        const realx = origin.x + transform.xx * x + transform.xy * y;
        const realy = origin.y + transform.yx * x + transform.yy * y;
        return [realx, realy];
    }
}

module.exports = { Shadowcast };