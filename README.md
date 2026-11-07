# Warehouse Lab

A grid-based multi-robot warehouse simulator comparing BFS baseline, STL-guided search, route-timing ADMM and crossing-allocation ADMM. The browser provides the warehouse animation, mission queue, benchmark aggregates, STL observations and execution history.

## Run locally

Install Node.js 20 or newer, then run:

```sh
npm start
```

Open http://127.0.0.1:4173/ in Chrome or Edge. Keep the terminal running. No npm dependencies or installation step are required. On Windows, `Start-Simulator.ps1` is an alternative launcher using port 4188.

## Build a portable simulator

```sh
npm run build
```

This generates `Open Warehouse Lab.html`. Save it and open it directly in Chrome or Edge. Attachment previews may block scripts and workers. The generated HTML is ignored by Git because it duplicates the source.

## Project layout

- `src/`: world, robot state, planners, peer-message simulation, workers, metrics and STL monitoring.
- `index.html`, `style.css`: browser interface.
- `tools/`: local server, offline build, benchmark runner and export validation.
- `benchmarks/latest/`: compact results from the latest recorded 80-run campaign.

All 20 source modules are used by the browser or its workers. Workers run planning separately from rendering. Simulated agent messages are copied between robot endpoints; this is not a deployed network of robots.

## Benchmarks

Use matched seeds and settings when comparing methods. To reproduce the 80-run design:

```sh
npm run benchmark -- --presentation
node tools/summarize-benchmark.mjs benchmarks/current-runs
```

The design uses 30 and 50 robots, two orders per robot, medium shared-resource demand, a fixed 54 by 22 warehouse, and ten seeds: 7, 11, 23, 42, 71, 101, 211, 509, 2026 and 3001. All four coordination methods are included. Planning latency consumes mission ticks, so hardware and process scheduling affect results.

Use `npm run benchmark -- --pilot` for eight smaller worker-based runs. Omit both flags for the full matrix. Set `BENCHMARK_OUT` to a new directory to keep campaigns separate. The runner stores compressed execution exports, source snapshots and workload hashes. The summarizer expects the complete 80-run design and its raw exports; the compact checked-in results do not include these large files.

To validate a simulation export downloaded from the browser:

```sh
npm run verify -- path/to/export.json
```
