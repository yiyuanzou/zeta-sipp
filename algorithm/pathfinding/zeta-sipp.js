const { TOAASIPP } = require("./to-aa-sipp");

class ZetaSIPP extends TOAASIPP {
    constructor(mesh, speed, isSIPP) {
        super(mesh, speed, isSIPP);
        this.closeCells = [];
    }

    // Zeta-SIPP Initialization
    init(start, end) {

        // Initialize the start and end.
        const startCell = this.graph.grids[Math.floor(start[0])][Math.floor(start[1])];
        const endCell = this.graph.grids[Math.floor(end[0])][Math.floor(end[1])];

        // Initialize the start cell.
        this.initStart(startCell, endCell);

        // Initialize the bound heap
        this.initBound(startCell, endCell);

        return [startCell, endCell];
    }

    initBound(startCell, endCell) {
        startCell.fh = startCell.h;
        this.graph.cells.forEach(cell => {
            if (!cell.weight || cell === startCell) return;
            this.boundCell(cell, startCell, endCell);
        });
    }

    boundCell(cell, startCell, endCell) {
        cell.h = this.heuristic(cell, endCell);
        cell.fh = this.heuristic(startCell, cell) + cell.h;
        this.boundHeap.push(cell);
    }

    // Add the "closed" property to cells
    // Once one of a cell's nodes is closed, the cell is pushed into closeCells.
    updateCloseCell(currentCell) {
        if (!currentCell.closed) {
            currentCell.closed = true;
            this.closeCells.push(currentCell);
        }
    }

    // Combine forward expansion and inverted expansion.
    forwardExpansion(startCell, endCell) {
        while (this.boundHeap.size() > 0 && (this.openHeap.size() === 0 ||
            this.boundHeap.content[0].fh <= this.openHeap.content[0].f)) {
            const newCell = this.boundHeap.shift();
            this.visitCell(newCell, endCell);
            // Build visibility connections using line-of-sight or shadowcasting.
            this.invertedScan(newCell, startCell, endCell);
        }
    }

    // Add all the closed nodes to the potential parents.
    invertedScan(newCell) {
        this.closeCells.forEach(cell => {
            if (this.graph.lineOfSightGrid(cell, newCell)) {
                cell.visibleCells.push(newCell);
                cell.nodes.forEach(parent => {
                    if (!parent.closed) return;
                    newCell.nodes.forEach(node =>
                        this.addPotentialParent(parent, node));
                });
            }
        });
    }
}

module.exports = { ZetaSIPP };