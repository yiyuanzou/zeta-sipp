// Adapted from https://playtechs.blogspot.com/2007/03/raytracing-on-grid.html
// All endpoints are cell centers. Cells touched only at a corner are excluded.
// Crossing-order arithmetic uses exact integers (within Number safe-integer range).
function lineSegOnGrid(x0, y0, x1, y1, time) {
    return traceLineOnGrid(x0, y0, x1, y1, time)[1];
}

function lineOfSightSegOnGrid(x0, y0, x1, y1, time, cells) {
    return traceLineOnGrid(x0, y0, x1, y1, time, cells);
}

function lineOfSightGrid(x0, y0, x1, y1, grids) {

    const dx = Math.abs(x1 - x0), dy = Math.abs(y1 - y0);
    let x = x0 - 0.5, y = y0 - 0.5;
    const x_inc = Math.sign(x1 - x0), y_inc = Math.sign(y1 - y0);
    let n = 1 + dx + dy;

    if (!grids[x][y].weight) return [false, 0];
    const maxN = n;
    let error = dy - dx;
    const stepX = 2 * dy, stepY = 2 * dx;

    // An axial segment never crosses a diagonal corner.
    if (dx === 0 || dy === 0) {
        while (--n > 0) {
            x += x_inc;
            y += y_inc;
            if (!grids[x][y].weight) return [false, maxN - n];
        }
        return [true, maxN];
    }

    while (--n > 0) {
        if (error > 0) {
            y += y_inc;
            error -= stepY;
        }
        else if (error < 0) {
            x += x_inc;
            error += stepX;
        }
        else {
            // A diagonal is blocked only when both adjacent cells are walls.
            if (!grids[x + x_inc][y].weight &&
                !grids[x][y + y_inc].weight)
                return [false, maxN - n + 1];
            x += x_inc;
            error += stepX;
            y += y_inc;
            error -= stepY;
            n--;
        }
        if (!grids[x][y].weight) return [false, maxN - n];
    }
    return [true, maxN];
}

// Share occupancy times and corner stepping; supplying cells also checks visibility.
function traceLineOnGrid(x0, y0, x1, y1, time, cells) {

    const grids = [];
    const dx = Math.abs(x1 - x0), dy = Math.abs(y1 - y0);
    let x = x0 - 0.5, y = y0 - 0.5;
    const x_inc = Math.sign(x1 - x0), y_inc = Math.sign(y1 - y0);
    let n = 1 + dx + dy;

    let d0 = 0, d1 = 0;

    // Axis-aligned rays need no crossing-order or corner tests. Keep the
    // original reciprocal additions so dynamic collision times stay bit-identical.
    if (dx === 0 || dy === 0) {
        const step = 1 / (dx + dy);
        let next = 0.5 * step;
        for (; n > 0; n--) {
            if (cells && !cells[x][y].weight) return [false, grids];
            d1 = time * Math.min(next, 1);
            grids.push({ x, y, time: [d0, d1] });
            d0 = d1;
            x += x_inc;
            y += y_inc;
            next += step;
        }
        return [true, grids];
    }

    const dtdx = 1 / dx, dtdy = 1 / dy;
    let t_next_horizontal = 0.5 * dtdx;
    let t_next_vertical = 0.5 * dtdy;

    // Integer error is nextX - nextY in the scaled crossing domain.
    // Zero means an exact corner; no floating-point tolerance is needed.
    let error = dy - dx;
    const stepX = 2 * dy, stepY = 2 * dx;

    for (; n > 0; n--) {

        if (cells && !cells[x][y].weight) return [false, grids];
        d1 = time * Math.min(t_next_horizontal, t_next_vertical, 1);
        grids.push({ x: x, y: y, time: [d0, d1] });
        d0 = d1;

        if (error === 0 && n > 1) {
            if (cells && !cells[x + x_inc][y].weight && !cells[x][y + y_inc].weight) return [false, grids];
            y += y_inc;
            t_next_vertical += dtdy;
            error -= stepY;
            x += x_inc;
            t_next_horizontal += dtdx;
            error += stepX;
            n--;
        }
        else if (error > 0) {
            y += y_inc;
            t_next_vertical += dtdy;
            error -= stepY;
        }
        else if (error < 0) {
            x += x_inc;
            t_next_horizontal += dtdx;
            error += stepX;
        }
    }
    return [true, grids];
}

module.exports = { lineOfSightGrid, lineSegOnGrid, lineOfSightSegOnGrid };
