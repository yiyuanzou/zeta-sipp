const { TOAASIPP } = require("./to-aa-sipp");
const { Shadowcast } = require("../shadowcasting");

class TOAAFoVSIPP extends TOAASIPP {
    constructor(graph, speed) {
        super(graph, speed);
        this.shadowcast = new Shadowcast(this.graph.cells);
    }

    // Initialization
    init(start, end) {
        const startCell = this.graph.cells[Math.floor(start[0])][Math.floor(start[1])];
        const endCell = this.graph.cells[Math.floor(end[0])][Math.floor(end[1])];
        const startNode = startCell.nodes[0];
        startNode.g = 0;

        const visibleCells = this.shadowcast.scan(startCell);
        for (const cell of visibleCells) {
            if (!cell.visited) this.visitCell(cell, endCell);
            cell.nodes.forEach(node => this.initNodes(node, startNode));
        }
        return [startCell, endCell];
    }

    nodeExpansion(currentNode, _, endCell) {
        if (!currentNode) return;
        const visibleCells = this.shadowcast.scan(currentNode);
        this.scannedNodes += visibleCells.length;

        for (const cell of visibleCells) {
            if (!cell.visited) this.visitCell(cell, endCell);
            cell.nodes.forEach(node => {
                if (node.closed) return;
                this.addPotentialParent(currentNode, node);
            });
        }
    }
}

module.exports = { TOAAFoVSIPP };