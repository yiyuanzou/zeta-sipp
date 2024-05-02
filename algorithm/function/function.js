// Subtract two arrays
function subtract(array1, array2) {
    return array1.map((a, i) => a - array2[i]);
}

// Calculate the norm of an array
function norm(array) {
    return Math.sqrt(array.reduce((a, b) => a + b * b, 0));
}

// Calculate the distance between two points
function getDist(start, end) {
    return norm(subtract(start, end));
}

// Calculate the intersection of two sets
function intersectSets(set1, set2) {
    if (set1[0] <= set2[0] && set2[1] <= set1[1]) return set2;
    if (set2[0] <= set1[0] && set1[1] <= set2[1]) return set1;
    if (set1[0] <= set2[0] && set2[0] <= set1[1] && set1[1] <= set2[1]) return [set2[0], set1[1]];
    if (set2[0] <= set1[0] && set1[0] <= set2[1] && set2[1] <= set1[1]) return [set1[0], set2[1]];
    return [];
}

// Calculate the union of a big continuous sets and a small continuous set: [A,B,C,D...] and [E,F]
function unionAll(sets, set) {
    let middleSet = [];
    const firstIndex = sortedIndex(sets, set[0]);
    const secondIndex = sortedIndex(sets, set[1]);
    if (firstIndex % 2 && !(secondIndex % 2)) middleSet = [set[1]];
    else if (!(firstIndex % 2) && !(secondIndex % 2)) middleSet = set;
    else if (!(firstIndex % 2) && secondIndex % 2) middleSet = [set[0]];
    return sets.slice(0, firstIndex).concat(middleSet, sets.slice(secondIndex));
}

// Insert a value into a sorted array
function sortedIndex(array, value) {
    let low = 0, high = array.length;
    while (low < high) {
        const mid = low + high >>> 1;
        if (array[mid] < value) low = mid + 1;
        else high = mid;
    }
    return low;
}

// Adapted from https://playtechs.blogspot.com/2007/03/raytracing-on-grid.html
function lineSegOnGrid(x0, y0, x1, y1, distance) {

    const grids = [];
    const dx = Math.abs(x1 - x0);
    const dy = Math.abs(y1 - y0);

    let x = Math.floor(x0);
    let y = Math.floor(y0);

    const dtdx = 1 / dx;
    const dtdy = 1 / dy;

    let d0 = 0, d1 = 0;                     // The distances from the start to the intersections
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

        d1 = distance * Math.min(t_next_horizontal, t_next_vertical, 1);
        grids.push({ x: x, y: y, dist: [d0, d1] });
        d0 = d1;

        if (t_next_horizontal > t_next_vertical) {
            y += y_inc;
            t_next_vertical += dtdy;
        }
        else {
            x += x_inc;
            t_next_horizontal += dtdx;
        }
    }
    return grids;
}

function lineOfSightGrid(x0, y0, x1, y1, grids) {

    let gridNumber = 0;
    const isBlocked = (x, y) => {
        gridNumber++;
        if (!grids[x] || !grids[x][y] || !grids[x][y].weight)
            return true;
        return false;
    };

    const dx = Math.abs(x1 - x0);
    const dy = Math.abs(y1 - y0);

    let x = Math.floor(x0);
    let y = Math.floor(y0);

    // The start point is blocked
    if (!Number.isInteger(x0) && !Number.isInteger(y0) && isBlocked(x, y)) return [false, gridNumber];
    if (Number.isInteger(x0) && Number.isInteger(y0) && x1 > x0 && y1 > y0 && isBlocked(x0, y0)) return [false, gridNumber];

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
                return [false, gridNumber];
            x += x_inc;
            error += dy;
            y += y_inc;
            error -= dx;
            n--;
        }
        if (isBlocked(x, y)) return [false, gridNumber];
    }
    return [true, gridNumber];
}

module.exports = { unionAll, lineOfSightGrid, getDist, lineSegOnGrid, intersectSets };