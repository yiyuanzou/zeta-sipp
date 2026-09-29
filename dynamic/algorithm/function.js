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

// Return a terminal result for invalid endpoints or an already reached goal.
// A null result means that normal search is required. The graph must be reset
// before constructing a planner, as for every other search in this module.
function trivialPath(graph, start, end) {
    const from = graph.cells[Math.floor(start[0])]?.[Math.floor(start[1])];
    const to = graph.cells[Math.floor(end[0])]?.[Math.floor(end[1])];
    if (!from?.weight || !to?.weight) return [];

    // Searches start at t = 0; a later safe interval cannot make that start valid.
    const node = from.nodes[0];
    if (!node || node.safeInterval[0] > 0 || node.safeInterval[1] <= 0) return [];
    if (from !== to) return null;

    node.g = 0;
    node.f = 0;
    node.closed = true;
    return [node];
}

module.exports = { unionAll, getDist, intersectSets, trivialPath };
