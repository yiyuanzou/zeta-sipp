// Load pathfinding algorithms
const { SIPP, Graph } = require('./algorithm/pathfinding/sipp');
const { SIPP_k } = require('./algorithm/pathfinding/sipp-k');
const { AASIPP } = require('./algorithm/pathfinding/aa-sipp');
const { TOAASIPP } = require('./algorithm/pathfinding/to-aa-sipp');
const { TOAAFoVSIPP } = require('./algorithm/pathfinding/to-aa-fov-sipp');
const { ZetastarSIPP_i } = require('./algorithm/pathfinding/zetastar-sipp_i');
const { ZetastarSIPP_f } = require('./algorithm/pathfinding/zetastar-sipp_f');

const filepath = require('path');
const fs = require('fs');
const { parseMap } = require('./scenario/parseMap');
const { parseScen } = require('./scenario/parseScen');
const { createScen } = require('./scenario/createScen');

// Object moving speed (can be set arbitrarily)
const speed = 0.1;

// Record the experiment outputs.
const header = ['map', 'scenSet', 'dynamicObs', 'scenId', 'algorithm', 'refTime', 'pathTime', 'treeNodes', 'searchSteps', 'sortedElements', 'scannedNodes', 'sippGrids', 'runtime'];

// A list of algorithms to test.
const algorithms = ['SIPP', 'SIPP_16', 'SIPP_32', 'AASIPP', 'TOAASIPP', 'TOAAFoVSIPP', 'ZetastarSIPP_i', 'ZetastarSIPP_f'];

// A list of maps.
const maps = ['random-64-64-10.map', 'warehouse-10-20-10-2-2.map', 'Berlin_1_256.map'];

// Folders
const mapfolder = filepath.join(__dirname, `./dataset/map`);
const scenfolder = filepath.join(__dirname, `./dataset/scen`);
const outputfolder = filepath.join(__dirname, './output');
const resultsfolder = filepath.join(__dirname, './output/results');
if (!fs.existsSync(outputfolder)) fs.mkdirSync(outputfolder);
if (!fs.existsSync(resultsfolder)) fs.mkdirSync(resultsfolder);

// Experiments.
const finalResultPath = [];
maps.forEach(map => {

    const outputFiles = [];

    // Load the map.
    const grid = parseMap(filepath.join(mapfolder, map));
    const graph = new Graph(grid);

    // Load the scenarios (random scenarios).
    const scenSet = filepath.join(scenfolder, map.slice(0, -4) + '-random');
    fs.readdirSync(scenSet).forEach(scenfile => {
        if (scenfile.slice(-4) !== 'scen') return;
        const string = scenfile.slice(0, -5).split('-');
        const index = string[string.length - 1];
        const scenarios = parseScen(filepath.join(scenSet, scenfile));

        // Create scenarios: dynamic environments
        const dynamicScenfolder = filepath.join(scenSet, './dynamicScen');
        if (!fs.existsSync(dynamicScenfolder)) fs.mkdirSync(dynamicScenfolder);

        for (const dynamicObs of [32, 64, 96, 128]) {

            // Reset the intervals
            graph.resetIntervals();
            const dynamicScen = filepath.join(dynamicScenfolder, `${scenfile.slice(0, -5)}_${dynamicObs}.json`);
            const newScenarios = createScen(dynamicScen, scenarios, graph, dynamicObs, speed);

            // Search the paths.
            for (const algorithm of algorithms) {

                const outputPath = filepath.join(resultsfolder, `benchmarks_${algorithm}_${scenfile.slice(0, -5)}_${dynamicObs}.csv`);
                outputFiles.push(outputPath);
                if (fs.existsSync(outputPath)) continue;

                // Continue testing
                const output = fs.existsSync(outputPath) ? fs.readFileSync(outputPath, 'utf8').split("\n") : [header];
                if (output.length >= scenarios.length + 1) continue;

                newScenarios.forEach((scen, id) => {
                    if (!grid[scen.start[0]][scen.start[1]] ||
                        !grid[scen.end[0]][scen.end[1]]) return;
                    if (id < output.length - 1) return;

                    graph.reset();
                    const planner = initPlanner(graph, speed, algorithm);

                    // Search nodes are located at the centers of grid cells.
                    const start = [scen.start[0] + 0.5, scen.start[1] + 0.5];
                    const end = [scen.end[0] + 0.5, scen.end[1] + 0.5];

                    const t0 = process.hrtime.bigint();
                    const path = planner.search(start, end);
                    const t1 = process.hrtime.bigint();

                    const runtime = Number(t1 - t0) / 1e6;
                    const pathTime = path.length ? path[path.length - 1].g : 0;
                    const sortedElements = planner.sortedElements ? planner.sortedElements + planner.treeNodes : planner.treeNodes;

                    output.push([map, index, dynamicObs, id, algorithm, scen.shortest / speed, pathTime, planner.treeNodes,
                        planner.searchSteps, sortedElements, planner.scannedNodes, planner.sippGrids, runtime]);
                    console.log([map.slice(0, 4), index, algorithm, (pathTime).toFixed(2), (runtime).toFixed(2)]);
                    // fs.writeFileSync(outputPath, output.join("\n"), { encoding: 'utf8', flag: 'w' });
                });
                fs.writeFileSync(outputPath, output.join("\n"));
            }
        }
    });
    const outputPathMap = filepath.join(outputfolder, `benchmarks_${map}.csv`);
    finalResultPath.push(outputPathMap);
    mergecsv(outputFiles, outputPathMap);
});
mergecsv(finalResultPath, filepath.join(outputfolder, `benchmarks_dynamic.csv`));

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
function initPlanner(graph, speed, algorithm) {
    switch (algorithm) {
        case "SIPP": return new SIPP(graph, speed);
        case "SIPP_16": return new SIPP_k(graph, speed, 4);
        case "SIPP_32": return new SIPP_k(graph, speed, 5);
        case "AASIPP": return new AASIPP(graph, speed);
        case "TOAASIPP": return new TOAASIPP(graph, speed);
        case "TOAAFoVSIPP": return new TOAAFoVSIPP(graph, speed);
        case "ZetastarSIPP_i": return new ZetastarSIPP_i(graph, speed);
        case "ZetastarSIPP_f": return new ZetastarSIPP_f(graph, speed);
    }
}