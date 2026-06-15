// Load pathfinding algorithms
const { Astar, Graph } = require('./algorithm/pathfinding/astar');
const { Astar_k } = require('./algorithm/pathfinding/astar_k');
const { Thetastar } = require('./algorithm/pathfinding/thetastar');
const { Anya } = require('./algorithm/pathfinding/anya');
const { Zetastar_i } = require('./algorithm/pathfinding/zetastar_i');
const { Zetastar_f } = require('./algorithm/pathfinding/zetastar_f');

const filepath = require('path');
const fs = require('fs');
const { parseMap } = require('./scenario/parseMap');
const { parseScen } = require('./scenario/parseScen');

// Record the experiment outputs.
const header = ['mapset', 'map', 'scen', 'algorithm', 'refLength', 'pathLength', 'treeNodes', 'searchSteps', 'sortedElements', 'scannedNodes', 'runtime'];

// A list of algorithms to test.
const algorithms = ['Astar', 'Astar_16', 'Astar_32', 'Thetastar', 'Anya', 'Zetastar_i', 'Zetastar_f'];

// A list of mapsets.
const mapsets = ['city512-exp', 'rand10-exp'];

// Experiments.
mapsets.forEach(mapset => {

    // Read the mapset
    const mapfolder = filepath.join(__dirname, `./dataset/map/${mapset}`);
    const scenfolder = filepath.join(__dirname, `./dataset/scen/${mapset}`);
    const outputfolder = filepath.join(__dirname, './output');
    const resultsfolder = filepath.join(__dirname, './output/results');
    if (!fs.existsSync(outputfolder)) fs.mkdirSync(outputfolder);
    if (!fs.existsSync(resultsfolder)) fs.mkdirSync(resultsfolder);

    const outputFiles = [];

    fs.readdirSync(mapfolder).forEach(map => {

        // Load the map.
        const grid = parseMap(filepath.join(mapfolder, map));
        const graph = new Graph(grid);

        // Load the scenarios.
        const scenarios = parseScen(filepath.join(scenfolder, map + '.scen'));

        // Search the paths.
        for (const algorithm of algorithms) {

            const outputPath = filepath.join(resultsfolder, `benchmarks_${algorithm}_${map}.csv`);
            outputFiles.push(outputPath);

            // Continue testing
            const output = fs.existsSync(outputPath) ? fs.readFileSync(outputPath, 'utf8').split("\n") : [header];
            if (output.length >= scenarios.length + 1) continue;

            scenarios.forEach((scen, id) => {
                if (!grid[scen.start[0]][scen.start[1]] ||
                    !grid[scen.end[0]][scen.end[1]]) return;
                if (id < output.length - 1) return;

                graph.reset();
                const planner = initPlanner(graph, algorithm);
                const start = [scen.start[0], scen.start[1]];
                const end = [scen.end[0], scen.end[1]];

                const t0 = process.hrtime.bigint();
                const path = planner.search(start, end);
                const t1 = process.hrtime.bigint();

                const runtime = Number(t1 - t0) / 1e6;
                const pathLength = path.length ? path[path.length - 1].g : 0;
                const sortedElements = planner.sortedElements ? planner.sortedElements + planner.treeNodes : planner.treeNodes;

                output.push([mapset, map, id, algorithm, scen.shortest, pathLength, planner.treeNodes, planner.searchSteps, sortedElements, planner.scannedNodes, runtime]);
                console.log([map, id, algorithm, pathLength.toFixed(2), (runtime).toFixed(2)]);
                // fs.writeFileSync(outputPath, output.join("\n"), { encoding: 'utf8', flag: 'w' });
            });
            fs.writeFileSync(outputPath, output.join("\n"), { encoding: 'utf8', flag: 'w' });
        }
    });
    mergecsv(outputFiles, filepath.join(outputfolder, `benchmarks_${mapset}.csv`));
});

function mergecsv(filepaths, outputpath) {
    const csv = filepaths.map(path => fs.readFileSync(path, 'utf8'));
    for (let i = 1; i < csv.length; i++) {
        const [, ...spl] = csv[i].split("\n");
        csv[i] = spl.join("\n");
    }
    const result = csv.join("\n");
    fs.writeFileSync(outputpath, result);
}

// Initialize the pathfinding planner
function initPlanner(graph, algorithm) {
    switch (algorithm) {
        case "Astar": return new Astar(graph);
        case "Astar_16": return new Astar_k(graph, 4);
        case "Astar_32": return new Astar_k(graph, 5);
        case "Thetastar": return new Thetastar(graph);
        case "Anya": return new Anya(graph);
        case "Zetastar_i": return new Zetastar_i(graph);
        case "Zetastar_f": return new Zetastar_f(graph);
    }
}