// Each line of a scenario has 9 fields:
// Bucket, map, map width, map height, start x, start y, goal x, goal y, optimal length

const fs = require('fs');
function parseScen(filename) {
    const content = fs.readFileSync(filename).toString();
    const lines = content.split(/\r?\n/);
    return lines.slice(1, lines.length - 1).map(parseLine);
}

function parseLine(line) {
    const fields = line.split(/\s+/);
    return {
        start: convert(parseInt(fields[4]), parseInt(fields[5])),
        end: convert(parseInt(fields[6]), parseInt(fields[7])),
        shortest: parseFloat(fields[8])
    };
}

// Unify the grid coordinates and the real coordinates
function convert(x, y) {
    return [y, x];
}

module.exports = { parseScen };