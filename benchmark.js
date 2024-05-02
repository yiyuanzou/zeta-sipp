// Load pathfinding algorithms
const { TOAASIPP } = require('./algorithm/pathfinding/to-aa-sipp');
const { TOAAFoVSIPP } = require('./algorithm/pathfinding/to-aa-fov-sipp');
const { ZetaSIPP } = require('./algorithm/pathfinding/zeta-sipp');
const { ZetastarSIPP } = require('./algorithm/pathfinding/zetastar-sipp');

const filepath = require('path');
const fs = require('fs');
const { parseMap } = require('./scenario/parseMap');
const { parseScen } = require('./scenario/parseScen');
const { createScen } = require('./scenario/createScen');

// Object moving speed
const speed = 0.1;

// Record the experiment outputs.
const header = ['map', 'scenIndex', 'dynamicObs', 'scenId', 'algorithm', 'refTime', 'pathTime', 'treeNodes',
    'searchSteps', 'LOSchecks', 'scannedNodes', 'runtime_total'];

// A list of algorithms to test.
const algorithms = ['TOAASIPP', 'TOAAFoVSIPP', 'ZetaSIPP', 'ZetastarSIPP'];

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
    let mesh = grid2Mesh(grid);

    // Load the scenarios.
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

            // Clean the interval
            mesh.forEach(row => row.forEach(cell => cell.riskInterval = []));

            const dynamicScen = filepath.join(dynamicScenfolder, `${scenfile.slice(0, -5)}_${dynamicObs}.json`);
            const newScenarios = createScen(dynamicScen, scenarios, mesh, dynamicObs);

            // Search the paths.
            for (const algorithm of algorithms) {

                const outputPath = filepath.join(resultsfolder, `benchmarks_${algorithm}_${scenfile.slice(0, -5)}_${dynamicObs}.csv`);
                outputFiles.push(outputPath);
                if (fs.existsSync(outputPath)) continue;

                const output = [header];
                newScenarios.forEach((scen, id) => {
                    if (!grid[scen.start[0]][scen.start[1]] ||
                        !grid[scen.end[0]][scen.end[1]]) return;
                    if (id < 0) return;

                    const planner = initPlanner(mesh, algorithm);
                    const start = [scen.start[0] + 0.5, scen.start[1] + 0.5];
                    const end = [scen.end[0] + 0.5, scen.end[1] + 0.5];

                    const startTime = performance.now();
                    const path = planner.search(start, end);  // The center of the grid
                    const endTime = performance.now();

                    const runtime = endTime - startTime;
                    const pathTime = path.length ? path[path.length - 1].g : 0;
                    const refTime = scen.shortest / speed;
                    const nodes = planner.treeCells.reduce((prev, curr) => prev + curr.nodes.length, 0);
                    const steps = planner.steps;
                    const LOSchecks = planner.graph.checks ? planner.graph.checks : 0;
                    const scannedNodes = planner.graph.scannedGrids ? planner.graph.scannedGrids : 0;

                    output.push([map, index, dynamicObs, id, algorithm, refTime, pathTime,
                        nodes, steps, LOSchecks, scannedNodes, runtime]);
                    console.log([map.slice(0, 4), index, dynamicObs, id, algorithm, (pathTime).toFixed(2), (runtime).toFixed(2)]);
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

// Convert the grid map into the desired mesh.
function grid2Mesh(grid) {
    const mesh = [];
    for (let i = 0; i < grid.length; i++) {
        const row = [];
        for (let j = 0; j < grid[i].length; j++) {
            const cell = {};
            cell.weight = grid[i][j];
            cell.center = [i + 0.5, j + 0.5];
            cell.coords = [[i, j], [i + 1, j], [i + 1, j + 1], [i, j + 1], [i, j]];
            cell.riskInterval = [];   // For SIPP.
            row.push(cell);
        }
        mesh.push(row);
    }
    return mesh;
}

// Initialize the pathfinding planner
function initPlanner(mesh, algorithm) {
    switch (algorithm) {
        case "TOAASIPP": return new TOAASIPP(mesh, speed, true);
        case "TOAAFoVSIPP": return new TOAAFoVSIPP(mesh, speed, true);
        case "ZetaSIPP": return new ZetaSIPP(mesh, speed, true);
        case "ZetastarSIPP": return new ZetastarSIPP(mesh, speed, true);
    }
}