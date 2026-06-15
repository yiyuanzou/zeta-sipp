
const { Astar } = require("./astar");

class Thetastar extends Astar {
    constructor(graph) {
        super(graph);
    }

    findParent(currentNode, neighbor) {
        return this.lineOfSightGrid(currentNode.parent, neighbor) ? currentNode.parent : currentNode;
    }

    heuristic(node1, node2) {
        return this.euclidean(node1, node2);
    }

    // Line-of-sight checks: coordinates are all integers.
    lineOfSightGrid(node1, node2) {
        if (!node1 || !node2) return false;

        const [x0, y0] = [node1.x, node1.y];
        const [x1, y1] = [node2.x, node2.y];

        // Start node
        const initHit = node1.parent ? true : false;

        // Same x
        if (x1 === x0) {
            if (y1 > y0) {
                if (this.isHitWallUp(x0, y0, initHit)) return false;
                for (let y = y0 + 1; y <= y1 - 1; y++)
                    if (this.isHitWallUp(x0, y, true)) {
                        this.scannedNodes += y - y0;
                        return false;
                    }
            }
            else {
                if (this.isHitWallDown(x0, y0, initHit)) return false;
                for (let y = y0 - 1; y >= y1 + 1; y--)
                    if (this.isHitWallDown(x0, y, true)) {
                        this.scannedNodes += y0 - y;
                        return false;
                    }
            }
            this.scannedNodes += Math.abs(y1 - y0);
            return true;
        }

        // Same y
        if (y1 === y0) {
            if (x1 > x0) {
                if (this.isHitWallRight(x0, y0, initHit)) return false;
                for (let x = x0 + 1; x <= x1 - 1; x++)
                    if (this.isHitWallRight(x, y0, true)) {
                        this.scannedNodes += x - x0;
                        return false;
                    }
            }
            else {
                if (this.isHitWallLeft(x0, y0, initHit)) return false;
                for (let x = x0 - 1; x >= x1 + 1; x--)
                    if (this.isHitWallLeft(x, y0, true)) {
                        this.scannedNodes += x0 - x;
                        return false;
                    }
            }
            this.scannedNodes += Math.abs(x1 - x0);
            return true;
        }

        // General case
        const eps = 1e-10;
        const sx = (x1 > x0) ? eps : (x1 < x0) ? -eps : 0;
        const sy = (y1 > y0) ? eps : (y1 < y0) ? -eps : 0;

        const x0e = x0 + sx;
        const y0e = y0 + sy;
        const x1e = x1 - sx;
        const y1e = y1 - sy;

        let x = Math.floor(x0e);
        let y = Math.floor(y0e);

        // The start point is blocked
        if (this.isRightUpBlocked(x, y)) return false;

        const dx = Math.abs(x1e - x0e);
        const dy = Math.abs(y1e - y0e);

        let n = 1;         // The number of intersecting grids.
        let x_inc, y_inc;  // The increment (direction) of x and y.
        let error;         // Determine horizontal or vertical expansion.

        // If the slope is infinity
        if (dx === 0) {
            x_inc = 0;
            error = Infinity;
        }
        else if (x1e > x0e) {
            x_inc = 1;
            n += Math.floor(x1e) - x;
            error = (x + 1 - x0e) * dy;
        }
        else {
            x_inc = -1;
            n += x - Math.floor(x1e);
            error = (x0e - x) * dy;
        }

        if (dy === 0) {
            y_inc = 0;
            error = -Infinity;
        }
        else if (y1e > y0e) {
            y_inc = 1;
            n += Math.floor(y1e) - y;
            error -= (y + 1 - y0e) * dx;
        }
        else {
            y_inc = -1;
            n += y - Math.floor(y1e);
            error -= (y0e - y) * dx;
        }

        const maxN = n;

        while (--n > 0) {
            if (error > 0) {
                y += y_inc;
                error -= dx;
            }
            else if (error < 0) {
                x += x_inc;
                error += dy;
            }
            else {
                if (this.isRightUpBlocked(x + x_inc, y) &&
                    this.isRightUpBlocked(x, y + y_inc)) {
                    this.scannedNodes += maxN - n + 1;
                    return false;
                }
                x += x_inc;
                error += dy;
                y += y_inc;
                error -= dx;
                n--;
            }
            if (this.isRightUpBlocked(x, y)) {
                this.scannedNodes += maxN - n;
                return false;
            }
        }
        this.scannedNodes += maxN;
        return true;
    }

    getMask(x, y) {
        const word = y >> 5;
        const bit = 1 << (y & 31);

        return (
            ((this.graph.leftdown[x][word] & bit) ? 1 : 0) |
            ((this.graph.leftup[x][word] & bit) ? 2 : 0) |
            ((this.graph.rightup[x][word] & bit) ? 4 : 0) |
            ((this.graph.rightdown[x][word] & bit) ? 8 : 0)
        );
    }

    // Check if the vertical ray hits a wall.
    isHitWallDown(x, y, isHit) {
        const m = this.getMask(x, y);
        return (m & 1) && ((m & 8) || (isHit && (m & 4))) ||
            (isHit && (m & 8) && (m & 2));
    }

    isHitWallUp(x, y, isHit) {
        const m = this.getMask(x, y);
        return (m & 2) && ((m & 4) || (isHit && (m & 8))) ||
            (isHit && (m & 4) && (m & 1));
    }

    isHitWallLeft(x, y, isHit) {
        const m = this.getMask(x, y);
        return (m & 2) && ((m & 1) || (isHit && (m & 8))) ||
            (isHit && (m & 1) && (m & 4));
    }

    isHitWallRight(x, y, isHit) {
        const m = this.getMask(x, y);
        return (m & 4) && ((m & 8) || (isHit && (m & 1))) ||
            (isHit && (m & 8) && (m & 2));
    }
}

module.exports = { Thetastar };