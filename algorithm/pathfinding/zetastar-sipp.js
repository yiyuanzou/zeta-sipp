const { ZetaSIPP } = require("./zeta-sipp");
const { Shadowcast } = require("./shadowcasting");

class ZetastarSIPP extends ZetaSIPP {
    constructor(mesh, speed, isSIPP) {
        super(mesh, speed, isSIPP);
        this.shadowcast = new Shadowcast(this.graph.grids);
        this.shadowcast.inverted = true;
        this.buffer = this.getScanBuffer();
    }

    invertedScan(scanCell) {
        // Scan to find visible cells.
        this.shadowcast.maxCost = scanCell.fh + this.buffer;
        const visibleCells = this.shadowcast.scan(scanCell);
        this.graph.scannedGrids += visibleCells.length;
        scanCell.visibleCells = visibleCells.filter(cell => cell.visited);

        scanCell.visibleCells.forEach(cell => {
            // Symmetric recursive shadowcasting
            cell.visibleCells.push(scanCell);
            // This node and closed nodes.
            if (cell.closed) {
                cell.nodes.forEach(parent => {
                    if (!parent.closed) return;
                    scanCell.nodes.forEach(node =>
                        this.addPotentialParent(parent, node));
                });
            }
        });
    }

    invertedExpansion(currentNode, currentCell) {
        currentCell.visibleCells.forEach(cell =>
            cell.nodes.forEach(node => {
                if (node.closed) return;
                this.addPotentialParent(currentNode, node);
            }));
    }

    getScanBuffer() {
        return 1.414214 / this.speed;
    }
}

module.exports = { ZetastarSIPP };