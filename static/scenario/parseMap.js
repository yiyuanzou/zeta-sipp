//  All maps begin with the lines:
//  type octile, height x, width y, map

const fs = require('fs');
function parseMap(filename) {
    const content = fs.readFileSync(filename).toString();
    const lines = content.split(/\r?\n/);
    return parseGrid(lines.slice(4));
}

const passable = ['.', 'G', 'S'];
function parseGrid(lines) {
    const grid = [];
    lines.forEach(line => {
        if (!line.length) return;
        const chars = line.split('');
        const row = chars.map(char =>
            passable.includes(char) ? 1 : 0);
        grid.push(row);
    });
    return grid;
}

module.exports = { parseMap };