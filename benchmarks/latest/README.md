# Recorded benchmark results

Recorded on 7 November 2026 using the simulator before this source-only cleanup. The cleanup removes unused helpers, a redundant local variable, unnecessary exports and repetitive comments; controller behavior is retained.

The campaign contains 80 validated worker executions: four methods, two fleet sizes (30 and 50), and ten matched seeds. Each run uses two orders per robot, medium shared-resource demand and a fixed 54 by 22 world. Ten percent of generated orders have deliberately impossible deadlines. Rates use all generated orders; delivered-order timing statistics exclude undelivered orders. Waiting includes service and scheduled holds.

`summary.csv` and `summary.json` contain aggregates; `runs.csv` contains per-run measurements. The raw exports, historical source snapshots, presentation files and old benchmark campaigns are excluded from this code repository.

STL-guided search and both ADMM methods recorded 90% on-time delivery at both fleet sizes. Route-timing ADMM recorded the lowest fleet waiting. BFS recorded one stalled run at 30 robots and three at 50 robots. All runs recorded zero collisions and zero execution-guard interventions. A large BFS planning-time outlier remains in the mean; inspect median and P90 values too. These comparisons concern complete controllers and do not isolate the effect of the optimization solver.
