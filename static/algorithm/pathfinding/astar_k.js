const { Astar } = require("./astar");
const { Shadowcast } = require("../shadowcasting");

// 2^k-neighborhoods A*
class Astar_k extends Astar {
    constructor(graph, k) {
        super(graph);
        this.shadowcast = new Shadowcast(graph);
        this.shadowcast.depth = k - 2;
    }

    findNeighbors(node) {
        if (node.parent && this.isDiagBlocked(node.x, node.y)) return []; // Dead end
        const visibleNodes = this.shadowcast.scan(node);
        this.scannedNodes += visibleNodes.length;
        return visibleNodes;
    }

    heuristic(node1, node2) {
        return this.euclidean(node1, node2);
    }

    isDiagBlocked(x, y) {
        return this.bit(this.graph.diagBlocked[x], y);
    }
}

module.exports = { Astar_k };