const { BinaryHeap } = require("../binaryHeap");

// Since Cartesian and grid coordinates are aligned, scanning is done column by column rather than row by row.
class Anya {
    constructor(graph) {
        this.graph = graph;                                         // Graph class
        this.openHeap = new BinaryHeap(node => node.f);             // Open heap instead of open list.

        this.treeNodes = 0;                                         // Number of tree nodes
        this.searchSteps = 0;                                       // Number of search steps
        this.scannedNodes = 0;                                      // Number of scanned nodes
    }

    // Initialization
    init(start, end) {

        const startRoot = this.graph.vertices[start[0]][start[1]];
        const endRoot = this.graph.vertices[end[0]][end[1]];

        startRoot.g = 0;
        const startNode = new SuperNode(startRoot, startRoot.x, []);
        startNode.f = this.euclidean(startRoot, endRoot);

        this.openHeap.push(startNode);
        this.treeNodes++;

        return endRoot;
    }

    // Search the path
    search(start, end) {

        // Initialization
        const endRoot = this.init(start, end);

        // Main loop
        while (this.openHeap.size() > 0) {

            // Get the node with the lowest f-value for next step. The heap keeps it sorted.
            const currentNode = this.openHeap.shift();
            this.searchSteps++;

            // Move currentNode from open to closed.
            currentNode.closed = true;

            // The target has been found.
            if (this.rootInInterval(endRoot, currentNode.colIndex, currentNode.interval)) {
                endRoot.g = currentNode.root.g + this.euclidean(currentNode.root, endRoot);
                return this.pathTo(currentNode, endRoot);
            }

            // Anya forward expansion.
            this.forwardExpansion(currentNode, endRoot);
        }
        // Fail to find a path.
        return [];
    }

    // Anya forward expansion.
    forwardExpansion(currentNode, endRoot) {

        // Find all successors for the current node.
        const successors = this.findSuccessors(currentNode, endRoot);
        currentNode.successors = [];

        for (let i = 0; i < successors.length; i++) {
            let successor = successors[i];

            // Scanned nodes
            this.scannedNodes += Math.ceil(successor.interval[1] - successor.interval[0]);

            // g-value of the successor’s root
            const gScore = currentNode.root.g + this.euclidean(successor.root, currentNode.root);

            // Check if the successor can provide a better parent root.
            if (gScore > successor.root.g) continue;

            // Pruning 
            successor = this.pruning(successor, endRoot);
            if (!successor) continue;

            // Update the f-value of the successor.
            successor.f = gScore + this.heuristic(successor, endRoot);

            // Update the root's parent and g-value.
            if (successor.root !== currentNode.root) {
                successor.root.parent = currentNode.root;
                successor.root.g = gScore;
            }

            // Update the open heap.
            this.openHeap.push(successor);
            this.treeNodes++;
        }
    }

    // Dead-end pruning and intermediate pruning
    pruning(node, endRoot) {
        if (this.rootInInterval(endRoot, node.colIndex, node.interval))
            return node;
        if (this.isDeadEnd(node)) return;
        const newNode = this.intermediatePruning(node);
        if (!newNode) return;
        if (node === newNode) return node;
        return this.pruning(newNode, endRoot);
    }

    pathTo(node, end) {
        let curr = node.root;
        const path = end ? [end] : [];
        while (curr) {
            path.unshift(curr);
            curr = curr.parent;
        }
        return path;
    }

    // Check if the target root is reached.
    rootInInterval(root, colIndex, interval) {
        if (!interval.length) return false;
        return root.x === colIndex && root.y >= interval[0] && root.y <= interval[1];
    }

    euclidean(root1, root2) {
        if (root1 === root2) return 0;
        const d1 = root2.x - root1.x;
        const d2 = root2.y - root1.y;
        return Math.sqrt(d1 * d1 + d2 * d2);
    }

    euclideanXY(x1, y1, x2, y2) {
        const d1 = x2 - x1;
        const d2 = y2 - y1;
        return Math.sqrt(d1 * d1 + d2 * d2);
    }

    heuristic(node, endRoot) {
        const colIndex = node.colIndex;
        const root = node.root;
        const endpoint1 = node.interval[0];
        const endpoint2 = node.interval[1];

        // Check if the node root and the end root are on the same side of the interval line.
        let mirroredEndX = endRoot.x;
        let mirroredEndY = endRoot.y;

        if ((endRoot.x > colIndex && root.x > colIndex) ||
            (endRoot.x < colIndex && root.x < colIndex)) {
            mirroredEndX = 2 * colIndex - endRoot.x;
        }

        // Flat and cone nodes
        const intersection = mirroredEndX === root.x ? mirroredEndY :
            (mirroredEndY - root.y) / (mirroredEndX - root.x) * (colIndex - root.x) + root.y;
        if (intersection < endpoint1) return this.euclideanXY(root.x, root.y, colIndex, endpoint1) + this.euclideanXY(colIndex, endpoint1, mirroredEndX, mirroredEndY);
        if (intersection > endpoint2) return this.euclideanXY(root.x, root.y, colIndex, endpoint2) + this.euclideanXY(colIndex, endpoint2, mirroredEndX, mirroredEndY);
        return this.euclideanXY(root.x, root.y, mirroredEndX, mirroredEndY);
    }

    // Find the successors of the current node.
    findSuccessors(node, endRoot) {

        // Find the successors of the start node.
        if (!node.interval.length)
            return this.findStartSuccessors(node, endRoot);

        if (node.successors.length)
            return node.successors;

        const endpoint1 = node.interval[0];
        const endpoint2 = node.interval[1];
        let successors = [];

        // Flat node
        if (node.isFlat) {
            const col = this.graph.vertices[node.colIndex];
            if (endpoint1 < node.root.y) {

                // Flat node's flat successors.
                const flatSuccessor = this.findFlatSuccessorDown(endpoint1, node.colIndex, node.root, endRoot);
                if (flatSuccessor) successors.push(flatSuccessor);

                // Obstacle following.
                if (this.isCornerPoint(node.colIndex, endpoint1) && this.isBetterPath(node.root, col[endpoint1])) {
                    const newSuccessors = this.findConeSuccessorsFlatNodeDown(endpoint1, node.colIndex);
                    for (let i = 0; i < newSuccessors.length; i++)
                        successors.push(newSuccessors[i]);
                }
            }
            else {

                // Flat node's flat successors.
                const flatSuccessor = this.findFlatSuccessorUp(endpoint2, node.colIndex, node.root, endRoot);
                if (flatSuccessor) successors.push(flatSuccessor);

                // Obstacle following.
                if (this.isCornerPoint(node.colIndex, endpoint2) && this.isBetterPath(node.root, col[endpoint2])) {
                    const newSuccessors = this.findConeSuccessorsFlatNodeUp(endpoint2, node.colIndex);
                    for (let i = 0; i < newSuccessors.length; i++)
                        successors.push(newSuccessors[i]);
                }
            }
        }
        else {

            // Find observable successors.
            successors = this.findObservableConeSuccessors(endpoint1, endpoint2, node.colIndex, node.root);

            // Find unobservable successors from endpoint1.
            if (this.isTurningPoint(node.root, node.colIndex, endpoint1) && this.isBetterPath(node.root, this.graph.vertices[node.colIndex][endpoint1])) {
                const flatSuccessor = this.findFlatSuccessorDown(endpoint1, node.colIndex, node.root, endRoot);
                if (flatSuccessor) successors.push(flatSuccessor);
                const newSuccessors = this.findUnobservableConeSuccessorsDown(endpoint1, node.colIndex, node.root);
                for (let i = 0; i < newSuccessors.length; i++)
                    successors.push(newSuccessors[i]);
            }

            // Find unobservable successors from endpoint2
            if (this.isTurningPoint(node.root, node.colIndex, endpoint2) && this.isBetterPath(node.root, this.graph.vertices[node.colIndex][endpoint2])) {
                const flatSuccessor = this.findFlatSuccessorUp(endpoint2, node.colIndex, node.root, endRoot);
                if (flatSuccessor) successors.push(flatSuccessor);
                const newSuccessors = this.findUnobservableConeSuccessorsUp(endpoint2, node.colIndex, node.root);
                for (let i = 0; i < newSuccessors.length; i++)
                    successors.push(newSuccessors[i]);
            }
        }
        return successors;
    }

    // The special case for the start node.
    findStartSuccessors(node, endRoot) {
        const successors = [];
        const root = node.root;
        const leftCol = this.graph.vertices[root.x - 1];
        const rightCol = this.graph.vertices[root.x + 1];

        // Down
        if (!this.isHitWallDown(root.x, root.y, false)) {
            const flatSuccessor = this.findFlatSuccessorDown(root.y, root.x, root, endRoot);
            if (flatSuccessor) successors.push(flatSuccessor);
        }

        // Up
        if (!this.isHitWallUp(root.x, root.y, false)) {
            const flatSuccessor = this.findFlatSuccessorUp(root.y, root.x, root, endRoot);
            if (flatSuccessor) successors.push(flatSuccessor);
        }

        // Left
        if (leftCol) {
            const x = root.x - 1;
            const intervalsDown = this.scanLeftColDown(x, root.y, 0);
            const intervalsUp = this.scanLeftColUp(x, root.y, leftCol.length - 1);
            this.pushMergedIntervals(successors, root, x, intervalsDown, intervalsUp, !this.isCornerPoint(x, root.y));
        }

        // Right
        if (rightCol) {
            const x = root.x + 1;
            const intervalsDown = this.scanRightColDown(x, root.y, 0);
            const intervalsUp = this.scanRightColUp(x, root.y, rightCol.length - 1);
            this.pushMergedIntervals(successors, root, x, intervalsDown, intervalsUp, !this.isCornerPoint(x, root.y));
        }
        return successors;
    }

    // Merge two intervals
    pushMergedIntervals(successors, root, x, down, up, shouldMerge) {
        let downStart = 0, upStart = 0;

        if (shouldMerge && down.length && up.length) {
            successors.push(new SuperNode(root, x, [down[0][0], up[0][1]]));
            downStart = 1; upStart = 1;
        }

        for (let i = downStart; i < down.length; i++)
            successors.push(new SuperNode(root, x, down[i]));

        for (let i = upStart; i < up.length; i++)
            successors.push(new SuperNode(root, x, up[i]));
    }

    makeSuperNodes(root, colIndex, intervals) {
        const nodes = new Array(intervals.length);
        for (let i = 0; i < intervals.length; i++)
            nodes[i] = new SuperNode(root, colIndex, intervals[i]);
        return nodes;
    }

    // Find the flat successor.
    findFlatSuccessorDown(endpoint, colIndex, root, endRoot) {
        const lowerEnd = this.graph.vertices[colIndex][endpoint];

        // The successor node of a flat node.
        if (colIndex === root.x) {
            const interval = this.scanCurrColDown(colIndex, endpoint, endRoot);
            if (!interval.length) return;
            return new SuperNode(root, colIndex, interval);
        }

        // The successor node of a cone node.
        if (endpoint <= root.y) {
            if ((colIndex < root.x && this.isRightDownBlocked(colIndex, endpoint)) ||
                (colIndex > root.x && this.isLeftDownBlocked(colIndex, endpoint))) {
                const interval = this.scanCurrColDown(colIndex, endpoint, endRoot);
                if (!interval.length) return;
                return new SuperNode(lowerEnd, colIndex, interval);
            }
        }
        return;
    }

    findFlatSuccessorUp(endpoint, colIndex, root, endRoot) {
        const upperEnd = this.graph.vertices[colIndex][endpoint];

        // The successor node of a flat node.
        if (colIndex === root.x) {
            const interval = this.scanCurrColUp(colIndex, endpoint, endRoot);
            if (!interval.length) return;
            return new SuperNode(root, colIndex, interval);
        }

        // The successor node of a cone node.
        if (endpoint >= root.y) {
            if ((colIndex < root.x && this.isRightUpBlocked(colIndex, endpoint)) ||
                (colIndex > root.x && this.isLeftUpBlocked(colIndex, endpoint))) {
                const interval = this.scanCurrColUp(colIndex, endpoint, endRoot);
                if (!interval.length) return;
                return new SuperNode(upperEnd, colIndex, interval);
            }
        }
        return;
    }

    // Find cone successors for a flat node (obstacle following).
    findConeSuccessorsFlatNodeDown(endpoint, colIndex) {
        const lowerEnd = this.graph.vertices[colIndex][endpoint];

        if (this.isLeftUpBlocked(colIndex, endpoint)) {
            const intervalsLeftDown = this.scanLeftColDown(colIndex - 1, endpoint, 0);
            return this.makeSuperNodes(lowerEnd, colIndex - 1, intervalsLeftDown);
        }
        if (this.isRightUpBlocked(colIndex, endpoint)) {
            const intervalsRightDown = this.scanRightColDown(colIndex + 1, endpoint, 0);
            return this.makeSuperNodes(lowerEnd, colIndex + 1, intervalsRightDown);
        }
        return [];
    }

    findConeSuccessorsFlatNodeUp(endpoint, colIndex) {
        const leftCol = this.graph.vertices[colIndex - 1];
        const rightCol = this.graph.vertices[colIndex + 1];
        const upperEnd = this.graph.vertices[colIndex][endpoint];

        if (this.isLeftDownBlocked(colIndex, endpoint)) {
            const intervalsLeftUp = this.scanLeftColUp(colIndex - 1, endpoint, leftCol.length - 1);
            return this.makeSuperNodes(upperEnd, colIndex - 1, intervalsLeftUp);
        }
        if (this.isRightDownBlocked(colIndex, endpoint)) {
            const intervalsRightUp = this.scanRightColUp(colIndex + 1, endpoint, rightCol.length - 1);
            return this.makeSuperNodes(upperEnd, colIndex + 1, intervalsRightUp);
        }
        return [];
    }

    // Find observable cone successors for a cone node.
    findObservableConeSuccessors(endpoint1, endpoint2, colIndex, root) {
        const leftCol = this.graph.vertices[colIndex - 1];
        const rightCol = this.graph.vertices[colIndex + 1];

        // Left column
        if (colIndex < root.x) {
            const nextEndpoint1 = this.projectToCol(endpoint1, colIndex, colIndex - 1, root);
            const nextEndpoint2 = this.projectToCol(endpoint2, colIndex, colIndex - 1, root);

            if (endpoint2 <= root.y) {
                // Check whether the interval between endpoint2 and nextEndpoint2 is clear.
                // If the following condition is met, there is no need to check the interval between endpoint2 and endpoint1.
                if (nextEndpoint2 < endpoint1) {
                    const newEndPoint2 = this.clearLeftColDown(colIndex - 1, endpoint1, nextEndpoint2);
                    if (newEndPoint2 > nextEndpoint2) return [];
                }
                const intervals = this.scanLeftColDown(colIndex - 1, nextEndpoint2, Math.max(nextEndpoint1, 0));
                return this.makeSuperNodes(root, colIndex - 1, intervals);
            }
            if (endpoint1 >= root.y) {
                if (nextEndpoint1 > endpoint2) {
                    const newEndPoint1 = this.clearLeftColUp(colIndex - 1, endpoint2, nextEndpoint1);
                    if (newEndPoint1 < nextEndpoint1) return [];
                }
                const intervals = this.scanLeftColUp(colIndex - 1, nextEndpoint1, Math.min(nextEndpoint2, leftCol.length - 1));
                return this.makeSuperNodes(root, colIndex - 1, intervals);
            }

            const x = colIndex - 1;
            const intervalsDown = this.scanLeftColDown(x, root.y, Math.max(nextEndpoint1, 0));
            const intervalsUp = this.scanLeftColUp(x, root.y, Math.min(nextEndpoint2, leftCol.length - 1));
            const successors = [];
            this.pushMergedIntervals(successors, root, x, intervalsDown, intervalsUp, !this.isCornerPoint(x, root.y));
            return successors;
        }

        // Right column
        else {
            const nextEndpoint1 = this.projectToCol(endpoint1, colIndex, colIndex + 1, root);
            const nextEndpoint2 = this.projectToCol(endpoint2, colIndex, colIndex + 1, root);

            if (endpoint2 <= root.y) {
                if (nextEndpoint2 < endpoint1) {
                    const newEndPoint2 = this.clearRightColDown(colIndex + 1, endpoint1, nextEndpoint2);
                    if (newEndPoint2 > nextEndpoint2) return [];
                }
                const intervals = this.scanRightColDown(colIndex + 1, nextEndpoint2, Math.max(nextEndpoint1, 0));
                return this.makeSuperNodes(root, colIndex + 1, intervals);
            }

            if (endpoint1 >= root.y) {
                if (nextEndpoint1 > endpoint2) {
                    const newEndPoint1 = this.clearRightColUp(colIndex + 1, endpoint2, nextEndpoint1);
                    if (newEndPoint1 < nextEndpoint1) return [];
                }
                const intervals = this.scanRightColUp(colIndex + 1, nextEndpoint1, Math.min(nextEndpoint2, rightCol.length - 1));
                return this.makeSuperNodes(root, colIndex + 1, intervals);
            }

            const x = colIndex + 1;
            const intervalsDown = this.scanRightColDown(x, root.y, Math.max(nextEndpoint1, 0));
            const intervalsUp = this.scanRightColUp(x, root.y, Math.min(nextEndpoint2, rightCol.length - 1));
            const successors = [];
            this.pushMergedIntervals(successors, root, x, intervalsDown, intervalsUp, !this.isCornerPoint(x, root.y));
            return successors;
        }
    }

    // Find unobservable cone successors for a cone node.
    findUnobservableConeSuccessorsDown(endpoint, colIndex, root) {
        const col = this.graph.vertices[colIndex];
        const leftCol = this.graph.vertices[colIndex - 1];
        const rightCol = this.graph.vertices[colIndex + 1];
        let intervals = [];

        // Left column
        if (colIndex < root.x) {
            const nextEndpoint = this.projectToCol(endpoint, colIndex, colIndex - 1, root);
            if (endpoint > root.y && this.isLeftDownBlocked(colIndex, endpoint))
                intervals = this.scanLeftColUp(colIndex - 1, endpoint, Math.min(nextEndpoint, leftCol.length - 1));
            else if (endpoint <= root.y && this.isRightDownBlocked(colIndex, endpoint)) {
                const newEndPoint = this.clearLeftColDown(colIndex - 1, endpoint, nextEndpoint);
                intervals = this.scanLeftColDown(colIndex - 1, newEndPoint, 0);
            }
            return this.makeSuperNodes(col[endpoint], colIndex - 1, intervals);
        }
        // Right column
        if (colIndex > root.x) {
            const nextEndpoint = this.projectToCol(endpoint, colIndex, colIndex + 1, root);
            if (endpoint > root.y && this.isRightDownBlocked(colIndex, endpoint))
                intervals = this.scanRightColUp(colIndex + 1, endpoint, Math.min(nextEndpoint, rightCol.length - 1));
            else if (endpoint <= root.y && this.isLeftDownBlocked(colIndex, endpoint)) {
                const newEndPoint = this.clearRightColDown(colIndex + 1, endpoint, nextEndpoint);
                intervals = this.scanRightColDown(colIndex + 1, newEndPoint, 0);
            }
            return this.makeSuperNodes(col[endpoint], colIndex + 1, intervals);
        }
        return [];
    }

    findUnobservableConeSuccessorsUp(endpoint, colIndex, root) {
        const col = this.graph.vertices[colIndex];
        const leftCol = this.graph.vertices[colIndex - 1];
        const rightCol = this.graph.vertices[colIndex + 1];
        let intervals = [];

        // Left column
        if (colIndex < root.x) {
            const nextEndpoint = this.projectToCol(endpoint, colIndex, colIndex - 1, root);
            if (endpoint < root.y && this.isLeftUpBlocked(colIndex, endpoint))
                intervals = this.scanLeftColDown(colIndex - 1, endpoint, Math.max(nextEndpoint, 0));
            if (endpoint >= root.y && this.isRightUpBlocked(colIndex, endpoint)) {
                const newEndpoint = this.clearLeftColUp(colIndex - 1, endpoint, nextEndpoint);
                intervals = this.scanLeftColUp(colIndex - 1, newEndpoint, leftCol.length - 1);
            }
            return this.makeSuperNodes(col[endpoint], colIndex - 1, intervals);
        }
        // Right column
        if (colIndex > root.x) {
            const nextEndpoint = this.projectToCol(endpoint, colIndex, colIndex + 1, root);
            if (endpoint < root.y && this.isRightUpBlocked(colIndex, endpoint))
                intervals = this.scanRightColDown(colIndex + 1, endpoint, Math.max(nextEndpoint, 0));
            if (endpoint >= root.y && this.isLeftUpBlocked(colIndex, endpoint)) {
                const newEndPoint = this.clearRightColUp(colIndex + 1, endpoint, nextEndpoint);
                intervals = this.scanRightColUp(colIndex + 1, newEndPoint, rightCol.length - 1);
            }
            return this.makeSuperNodes(col[endpoint], colIndex + 1, intervals);
        }
        return [];
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

    // Scan a column (both upward and downward cases).
    // If collectIntervals === true, return intervals.
    // Otherwise, return the first blocked position (for clear... functions).
    scanColumn(x, start, end, blockedBits, cornerBits, dir, collectIntervals) {

        const intervals = [];

        // Determine scanning direction.
        const scanUp = dir === 1;

        // Initialize scanning bounds.
        let y = scanUp ? Math.ceil(start) : Math.floor(start);
        const limit = scanUp ? Math.floor(end) : Math.ceil(end);

        // Continue scanning while within bounds.
        while (scanUp ? y <= limit : y >= limit) {

            // Identify which 32-bit word and bit position.
            const word = y >> 5;
            const bit = y & 31;

            // Keep only relevant bits depending on direction.
            let mask = scanUp
                ? (blockedBits[word] | cornerBits[word]) & this.maskFrom(bit)
                : (blockedBits[word] | cornerBits[word]) & this.maskUpTo(bit);

            // Clamp mask to remain within the scanning range.
            if ((limit >> 5) === word) {
                mask &= scanUp
                    ? this.maskUpTo(limit & 31)
                    : this.maskFrom(limit & 31);
            }

            // No relevant bits in this segment, skip it.
            if (mask === 0) {
                y = scanUp ? ((word + 1) << 5) : ((word << 5) - 1);
                continue;
            }

            // Get the hit bit and the hitY.
            const hitBit = scanUp
                ? 31 - Math.clz32(mask & -mask)     // lowest bit
                : 31 - Math.clz32(mask);            // highest bit

            const hitY = (word << 5) + hitBit;

            // Check the status.
            const isCorner = this.bit(cornerBits, hitY);
            const isBlocked = this.bit(blockedBits, hitY);

            // For clear... functions: return immediately when hitting a wall.
            if (!collectIntervals && isBlocked)
                return hitY;

            // Add a new interval (corner splits).
            if (collectIntervals && isCorner) {
                if (scanUp ? start < hitY : hitY < start)
                    intervals.push(scanUp ? [start, hitY] : [hitY, start]);
                start = hitY;
            }

            // Hit a wall.
            if (isBlocked) {
                if (collectIntervals && !isCorner && (scanUp ? start < hitY : hitY < start))
                    intervals.push(scanUp ? [start, hitY] : [hitY, start]);
                break;
            }

            // Continue scanning.
            y = scanUp ? hitY + 1 : hitY - 1;
        }

        // Return for clear... functions.
        if (!collectIntervals)
            return end;

        // Add remaining interval.
        if (scanUp ? y > limit && start < end : y < limit && end < start)
            intervals.push(scanUp ? [start, end] : [end, start]);

        return intervals;
    }

    // From the start point, scan downward along the left column.
    scanLeftColDown(x, start, end) {
        return this.scanColumn(x, start, end,
            this.graph.rightdown[x], this.graph.isCorner[x], -1, true);
    }

    // From the start point, scan upward along the left column.
    scanLeftColUp(x, start, end) {
        return this.scanColumn(x, start, end,
            this.graph.rightup[x], this.graph.isCorner[x], 1, true);
    }

    // From the start point, scan downward along the right column.
    scanRightColDown(x, start, end) {
        return this.scanColumn(x, start, end,
            this.graph.leftdown[x], this.graph.isCorner[x], -1, true);
    }

    // From the start point, scan upward along the right column.
    scanRightColUp(x, start, end) {
        return this.scanColumn(x, start, end,
            this.graph.leftup[x], this.graph.isCorner[x], 1, true);
    }

    clearLeftColDown(x, start, end) {
        return this.scanColumn(x, start, end,
            this.graph.rightdown[x], this.graph.isCorner[x], -1, false);
    }

    clearLeftColUp(x, start, end) {
        return this.scanColumn(x, start, end,
            this.graph.rightup[x], this.graph.isCorner[x], 1, false);
    }

    clearRightColDown(x, start, end) {
        return this.scanColumn(x, start, end,
            this.graph.leftdown[x], this.graph.isCorner[x], -1, false);
    }

    clearRightColUp(x, start, end) {
        return this.scanColumn(x, start, end,
            this.graph.leftup[x], this.graph.isCorner[x], 1, false);
    }

    // Scan from a point to a wall or obstacle corner along the column.
    scanCurrColDown(x, point, endRoot) {
        const ld = this.graph.leftdown[x];
        const lu = this.graph.leftup[x];
        const rd = this.graph.rightdown[x];
        const ru = this.graph.rightup[x];
        const ic = this.graph.isCorner[x];

        let y = point - 1;

        while (y >= 0) {
            const word = y >> 5;
            const bit = y & 31;

            // Wall bits matching isHitWallDown(root, true).
            const wall =
                (ld[word] & (rd[word] | ru[word])) |
                (rd[word] & lu[word]);

            // Keep only wall/corner bits at or below the current position.
            let mask = (wall | ic[word]) & this.maskUpTo(bit);

            // No relevant bits in this segment, skip to previous word.
            if (mask === 0) {
                y = (word << 5) - 1;
                continue;
            }

            // Get the highest bit and the hitY.
            const hitBit = 31 - Math.clz32(mask);
            const hitY = (word << 5) + hitBit;

            const interval = [hitY, point];

            const isWall =
                (this.bit(ld, hitY) && (this.bit(rd, hitY) || this.bit(ru, hitY))) ||
                (this.bit(rd, hitY) && this.bit(lu, hitY));

            // Dead-end pruning.
            if (isWall) {
                if (this.rootInInterval(endRoot, x, interval))
                    return interval;
                return [];
            }

            // Intermediate pruning.
            const hasDownBlock = this.bit(ld, hitY) || this.bit(rd, hitY);

            if (hasDownBlock) {
                if (!this.rootInInterval(endRoot, x, interval))
                    return this.scanCurrColDown(x, hitY, endRoot);
            }

            return interval;
        }
    }

    scanCurrColUp(x, point, endRoot) {
        const ld = this.graph.leftdown[x];
        const lu = this.graph.leftup[x];
        const rd = this.graph.rightdown[x];
        const ru = this.graph.rightup[x];
        const ic = this.graph.isCorner[x];

        const maxY = this.graph.vertices[x].length - 1;
        let y = point + 1;

        while (y <= maxY) {
            const word = y >> 5;
            const bit = y & 31;

            // Wall bits matching isHitWallUp(root, true).
            const wall =
                (lu[word] & (ru[word] | rd[word])) |
                (ru[word] & ld[word]);

            // Keep only wall/corner bits at or above the current position.
            let mask = (wall | ic[word]) & this.maskFrom(bit);

            // If the current word contains the upper bound, remove bits above maxY.
            if ((maxY >> 5) === word)
                mask &= this.maskUpTo(maxY & 31);

            // No relevant bits in this segment, skip to next word.
            if (mask === 0) {
                y = (word + 1) << 5;
                continue;
            }

            // Get the lowest bit and the hitY.
            const lowest = mask & -mask;
            const hitBit = 31 - Math.clz32(lowest);
            const hitY = (word << 5) + hitBit;

            const interval = [point, hitY];

            const isWall =
                (this.bit(lu, hitY) && (this.bit(ru, hitY) || this.bit(rd, hitY))) ||
                (this.bit(ru, hitY) && this.bit(ld, hitY));

            // Dead-end pruning.
            if (isWall) {
                if (this.rootInInterval(endRoot, x, interval))
                    return interval;
                return [];
            }

            // Intermediate pruning.
            const hasUpBlock = this.bit(lu, hitY) || this.bit(ru, hitY);

            if (hasUpBlock) {
                if (!this.rootInInterval(endRoot, x, interval))
                    return this.scanCurrColUp(x, hitY, endRoot);
            }

            return interval;
        }
    }

    // Check if the vertical ray hits a wall.
    isHitWallDown(x, y, isHit) {
        const ld = this.isLeftDownBlocked(x, y);
        const lu = this.isLeftUpBlocked(x, y);
        const rd = this.isRightDownBlocked(x, y);
        const ru = this.isRightUpBlocked(x, y);
        return (ld && (rd || (isHit && ru))) || (isHit && rd && lu);
    }

    isHitWallUp(x, y, isHit) {
        const ld = this.isLeftDownBlocked(x, y);
        const lu = this.isLeftUpBlocked(x, y);
        const rd = this.isRightDownBlocked(x, y);
        const ru = this.isRightUpBlocked(x, y);
        return (lu && (ru || (isHit && rd))) || (isHit && ru && ld);
    }

    // Linear projection
    projectToCol(endpoint, colIndex, newColIndex, root) {
        if (root.x === colIndex) return endpoint;
        const value = (root.y - endpoint) / (root.x - colIndex) * (newColIndex - root.x) + root.y;
        const rounded = Math.round(value);
        if (Math.abs(value - rounded) < 1e-10) return rounded;
        return value;
    }

    // Dead-end pruning
    isDeadEnd(node) {
        // Flat nodes have been pruned during expansion.
        if (node.isFlat) return false;
        const x = node.colIndex;
        const y = Math.floor(node.interval[0]);
        if (x < node.root.x)
            return this.isLeftUpBlocked(x, y);
        return this.isRightUpBlocked(x, y);
    }

    // Intermediate pruning
    intermediatePruning(node) {
        // Flat node have been pruned during expansion.
        if (node.isFlat) return node;

        if (this.isTurningPoint(node.root, node.colIndex, node.interval[0]) ||
            this.isTurningPoint(node.root, node.colIndex, node.interval[1])) return node;
        const successors = this.findObservableConeSuccessors(node.interval[0], node.interval[1], node.colIndex, node.root);

        if (!successors.length) return;
        if (successors.length === 1) {
            const successor = successors[0];
            this.scannedNodes += Math.ceil(successor.interval[1] - successor.interval[0]);
            return successor;
        }
        node.successors = successors;
        return node;
    }

    // Check whether the path from the root to the endpoint is taut.
    isTurningPoint(root, colIndex, endpoint) {
        if (!Number.isInteger(endpoint)) return false;
        if (!this.isCornerPoint(colIndex, endpoint)) return false;

        // From left to right
        if (root.x < colIndex) {
            if (root.y < endpoint)
                return !this.isRightUpBlocked(colIndex, endpoint);
            if (root.y === endpoint)
                return !this.isRightUpBlocked(colIndex, endpoint) && !this.isRightDownBlocked(colIndex, endpoint);
            return !this.isRightDownBlocked(colIndex, endpoint);
        }

        // From right to left
        if (root.x > colIndex) {
            if (root.y < endpoint)
                return !this.isLeftUpBlocked(colIndex, endpoint);
            if (root.y === endpoint)
                return !this.isLeftUpBlocked(colIndex, endpoint) && !this.isLeftDownBlocked(colIndex, endpoint);
            return !this.isLeftDownBlocked(colIndex, endpoint);
        }

        // Up and down
        if (root.x === colIndex) {
            if (root.y < endpoint)
                return !this.isLeftUpBlocked(colIndex, endpoint) && !this.isRightUpBlocked(colIndex, endpoint);
            return !this.isLeftDownBlocked(colIndex, endpoint) && !this.isRightDownBlocked(colIndex, endpoint);
        }

        return false;
    }

    isBetterPath(root1, root2) {
        const limit = root2.g - root1.g;
        if (limit <= 0) return false;
        const dx = root2.x - root1.x;
        const dy = root2.y - root1.y;
        return dx * dx + dy * dy < limit * limit;
    }
}

// Anya nodes
class SuperNode {
    constructor(root, colIndex, interval) {
        this.root = root;
        this.colIndex = colIndex;
        this.f = Infinity;
        this.closed = false;
        this.interval = interval;
        this.isFlat = root.x === colIndex;
        this.successors = [];  // Record the results of the intermediate pruning
    }
}

module.exports = { Anya };