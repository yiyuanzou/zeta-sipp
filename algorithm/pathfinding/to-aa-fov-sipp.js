const { TOAASIPP } = require("./to-aa-sipp");
const { Shadowcast } = require("./shadowcasting");

class TOAAFoVSIPP extends TOAASIPP {
    constructor(mesh, speed, isSIPP) {
        super(mesh, speed, isSIPP);
        this.shadowcast = new Shadowcast(this.graph.grids);
    }

    // TO-AA-FoV-SIPP Initialization
    init(start, end) {

        // Initialize the start and end.
        const startCell = this.graph.grids[Math.floor(start[0])][Math.floor(start[1])];
        const endCell = this.graph.grids[Math.floor(end[0])][Math.floor(end[1])];

        // Initialize the start cell.
        this.initStart(startCell, endCell);

        return [startCell, endCell];
    }

    invertedExpansion(currentNode, currentCell, endCell) {
        if (!currentCell.visibleCells.length) {
            currentCell.visibleCells = this.shadowcast.scan(currentCell);
            this.graph.scannedGrids += currentCell.visibleCells.length;
        }

        currentCell.visibleCells.forEach(cell => {
            // Add new visited nodes
            if (!cell.visited) this.visitCell(cell, endCell);
            // Set the current node as a potential parent
            cell.nodes.forEach(node => {
                if (node.closed) return;
                this.addPotentialParent(currentNode, node);
            });
        });
    }
}

module.exports = { TOAAFoVSIPP };