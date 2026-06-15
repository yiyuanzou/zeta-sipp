// Adapted from https://playtechs.blogspot.com/2007/03/raytracing-on-grid.html
// A grid cell is not counted if the path segment passes only through its corners.
function lineSegOnGrid(x0, y0, x1, y1, time) {

    const grids = [];
    const dx = Math.abs(x1 - x0);
    const dy = Math.abs(y1 - y0);

    let x = Math.floor(x0);
    let y = Math.floor(y0);

    const dtdx = 1 / dx;
    const dtdy = 1 / dy;

    let d0 = 0, d1 = 0;                     // The times from the start to the intersections
    let n = 1;                              // The number of intersecting grids.
    let x_inc, y_inc;                       // The increment (direction) of x and y.
    let t_next_vertical, t_next_horizontal; // Determine horizontal or vertical expansion.

    // If the slope is infinity
    if (dx === 0) {
        x_inc = 0;
        t_next_horizontal = Infinity;
    }
    else if (x1 > x0) {
        x_inc = 1;
        n += Math.floor(x1) - x;
        t_next_horizontal = (x + 1 - x0) * dtdx;
    }
    else {
        x_inc = -1;
        n += x - Math.floor(x1);
        t_next_horizontal = (x0 - x) * dtdx;
    }

    if (dy === 0) {
        y_inc = 0;
        t_next_vertical = Infinity;
    }
    else if (y1 > y0) {
        y_inc = 1;
        n += Math.floor(y1) - y;
        t_next_vertical = (y + 1 - y0) * dtdy;
    }
    else {
        y_inc = -1;
        n += y - Math.floor(y1);
        t_next_vertical = (y0 - y) * dtdy;
    }

    for (; n > 0; n--) {

        d1 = time * Math.min(t_next_horizontal, t_next_vertical, 1);
        grids.push({ x: x, y: y, time: [d0, d1] });
        d0 = d1;

        const error = Math.abs(t_next_horizontal - t_next_vertical);
        if (error < 1e-10) {
            y += y_inc;
            t_next_vertical += dtdy;
            x += x_inc;
            t_next_horizontal += dtdx;
            n--;
        }
        else if (t_next_horizontal > t_next_vertical) {
            y += y_inc;
            t_next_vertical += dtdy;
        }
        else if (t_next_horizontal < t_next_vertical) {
            x += x_inc;
            t_next_horizontal += dtdx;
        }
    }

    return grids;
}

function lineOfSightGrid(x0, y0, x1, y1, grids) {

    const isBlocked = (x, y) => !grids[x][y].weight;

    const dx = Math.abs(x1 - x0);
    const dy = Math.abs(y1 - y0);

    let x = Math.floor(x0);
    let y = Math.floor(y0);

    // The start point is blocked
    if (isBlocked(x, y)) return [false, 0];

    let n = 1;         // The number of intersecting grids.
    let x_inc, y_inc;  // The increment (direction) of x and y.
    let error;         // Determine horizontal or vertical expansion.

    // If the slope is infinity
    if (dx === 0) {
        x_inc = 0;
        error = Infinity;
    }
    else if (x1 > x0) {
        x_inc = 1;
        n += Math.floor(x1) - x;
        error = (x + 1 - x0) * dy;
    }
    else {
        x_inc = -1;
        n += x - Math.floor(x1);
        error = (x0 - x) * dy;
    }

    if (dy === 0) {
        y_inc = 0;
        error = -Infinity;
    }
    else if (y1 > y0) {
        y_inc = 1;
        n += Math.floor(y1) - y;
        error -= (y + 1 - y0) * dx;
    }
    else {
        y_inc = -1;
        n += y - Math.floor(y1);
        error -= (y0 - y) * dx;
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
            if (isBlocked(x + x_inc, y) &&
                isBlocked(x, y + y_inc))
                return [false, maxN - n + 1];
            x += x_inc;
            error += dy;
            y += y_inc;
            error -= dx;
            n--;
        }
        if (isBlocked(x, y)) return [false, maxN - n];
    }
    return [true, maxN];
}

function lineOfSightSegOnGrid(x0, y0, x1, y1, time, cells) {

    const isBlocked = (x, y) => !cells[x][y].weight;

    const grids = [];
    const dx = Math.abs(x1 - x0);
    const dy = Math.abs(y1 - y0);

    let x = Math.floor(x0);
    let y = Math.floor(y0);

    const dtdx = 1 / dx;
    const dtdy = 1 / dy;

    let d0 = 0, d1 = 0;                     // The times from the start to the intersections
    let n = 1;                              // The number of intersecting grids.
    let x_inc, y_inc;                       // The increment (direction) of x and y.
    let t_next_vertical, t_next_horizontal; // Determine horizontal or vertical expansion.

    // If the slope is infinity
    if (dx === 0) {
        x_inc = 0;
        t_next_horizontal = Infinity;
    }
    else if (x1 > x0) {
        x_inc = 1;
        n += Math.floor(x1) - x;
        t_next_horizontal = (x + 1 - x0) * dtdx;
    }
    else {
        x_inc = -1;
        n += x - Math.floor(x1);
        t_next_horizontal = (x0 - x) * dtdx;
    }

    if (dy === 0) {
        y_inc = 0;
        t_next_vertical = Infinity;
    }
    else if (y1 > y0) {
        y_inc = 1;
        n += Math.floor(y1) - y;
        t_next_vertical = (y + 1 - y0) * dtdy;
    }
    else {
        y_inc = -1;
        n += y - Math.floor(y1);
        t_next_vertical = (y0 - y) * dtdy;
    }

    for (; n > 0; n--) {

        if (isBlocked(x, y)) return [false, grids];
        d1 = time * Math.min(t_next_horizontal, t_next_vertical, 1);
        grids.push({ x: x, y: y, time: [d0, d1] });
        d0 = d1;

        const error = Math.abs(t_next_horizontal - t_next_vertical);
        if (error < 1e-10 && n > 1) {
            if (isBlocked(x + x_inc, y) && isBlocked(x, y + y_inc)) return [false, grids];
            y += y_inc;
            t_next_vertical += dtdy;
            x += x_inc;
            t_next_horizontal += dtdx;
            n--;
        }
        else if (t_next_horizontal > t_next_vertical) {
            y += y_inc;
            t_next_vertical += dtdy;
        }
        else if (t_next_horizontal < t_next_vertical) {
            x += x_inc;
            t_next_horizontal += dtdx;
        }
    }
    return [true, grids];
}

module.exports = { lineOfSightGrid, lineSegOnGrid, lineOfSightSegOnGrid };