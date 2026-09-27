# XY residual with fixed100m decay: positive bounded pilot

Completed2026-09-27 before14:30 Moscow. Stricter preregistered criterion PASS. Native sequential job took 28.668868399981875s at BelowNormal and one numerical thread. mainline source unchanged during our runs; both global and decay alternatives retained. Ready for independent nonauthor review, not yet deployment acceptance.

## Fixed design

The earlier global-shift pilot improved checker XYZ but worsened30618 validation quality5.11%. mainline then specified exactly one alternative:100m spatial decay, chosen as order of terminal geometry scale (~200m), before this alternative was scored. No length/gain/gate search and no map/model fitting occurred.

Optional gnss_xy_residual_correction remains false by default. Residual XY after full antenna phase projection updates distinct gnss_xy_bias_ with gain0.25 and vector cap5m. Gates remain innovation30m, residual30m in optional mode, age1s. Before each GNSS update, learned XY bias decays exp(-max(0,delta integrated distance)/100m). It does not decay at standstill. Permanent initial map_shift_ is never modified by bias learning/decay, including align_initial_position=true. Bias Z is always zero; reset clears it. Both predicted antenna and output position use map_shift+bias.

Phase correction remains unchanged. Core observer, speed and integrated distance are unchanged. Phase and bias have independent5m caps; simultaneous along20m/cross20m innovation gives7.071m total straight displacement.100m is fixed in the sealed source, not a new ROS/config setting.

## New checker: unchanged native independent scorer

Absolute map XYZ, core scalar output, assumed publication latency0/3ms. These are native scorer results, not actual DDS/ATS results. Reference labels were used only by scoring. Disabled output equals every one of20 mainline baseline columns exactly; enabled timestamps, core/bogie speed and distance equal phase-only exactly.

| Latency | Position pairs | Phase XYZ RMSE | Decay XYZ RMSE | Phase max | Decay max |
|---|---:|---:|---:|---:|---:|
| 0ms | 26188 | 6.953558997 | 6.114866554 | 50.994917956 | 43.088026895 |
| 3ms | 26188 | 6.953249289 | 6.115977597 | 50.994917956 | 43.088026895 |

0ms scalar RMSE stays0.056838323435, max0.323603712090,26233 pairs. Z RMSE0.381393725→0.392432213, max1.370232058→1.385119214. Map translation Z is unchanged; changed phase can indirectly alter map Z.

## Same26 sparse-regression schedule

First3s all fixes; master1s every120s starting120s; rover1s every120s starting60s. No schedule change. Every output stamp, core speed, integrated distance and initialized mask matches frozen phase baseline; every raw/quality reference sample count matches after3s. The split named holdout is exposed audit data, not an independent holdout.

| Group | Quality n | Phase quality RMSE | Decay quality RMSE | Phase quality max | Decay quality max | Phase raw RMSE | Decay raw RMSE |
|---|---:|---:|---:|---:|---:|---:|---:|
| all_30618 | 347778 | 4.751047597 | 4.064027897 | 47.165496172 | 47.154750937 | 4.899455439 | 4.290680343 |
| validation_30618 | 252362 | 3.627299938 | 2.748229881 | 30.078890621 | 26.065268596 | 3.880960410 | 3.114838523 |
| holdout_30618 | 95416 | 6.890152867 | 6.342213741 | 47.165496172 | 47.154750937 | 6.915460455 | 6.450016185 |
| all_30639 | 161465 | 7.732106417 | 7.293431078 | 86.653748725 | 86.650014091 | 7.849423205 | 7.534260002 |
| all_all | 509243 | 5.862724182 | 5.305252911 | 86.653748725 | 86.650014091 | 6.010842648 | 5.547624554 |

Only1/26 quality RMSE regresses:30618_e9a34502,1.396613→1.431811m. All8 raw/quality RMSE/max worsening entries are listed below; per_bag.csv contains all26 rows and maxima.

| Bag | Split | Metric | Statistic | Phase | Decay |
|---|---|---|---|---:|---:|
| 30618_27e994fc | holdout | xyz_valid | max | 48.156314923 | 48.156631796 |
| 30618_33bec73f | validation | xyz_valid | max | 19.504984356 | 19.507453706 |
| 30618_88548b02 | holdout | xyz_valid | max | 16.947527050 | 16.970475298 |
| 30618_88548b02 | holdout | xyz_quality | max | 16.947527050 | 16.970475298 |
| 30618_e9a34502 | validation | xyz_valid | rmse | 1.396612943 | 1.431810800 |
| 30618_e9a34502 | validation | xyz_quality | rmse | 1.396612943 | 1.431810800 |
| 30639_0be558e2 | validation | xyz_valid | max | 52.928439262 | 52.930414421 |
| 30639_50956d6e | validation | xyz_valid | rmse | 5.365898942 | 5.977390747 |

## Verification and limitations

30 native assertions PASS:19 legacy phase controls,8 residual controls,3 decay controls. Single cross3m produces0.75m;100m further travel divides bias by e; stationary bias holds; initial4m permanent alignment survives100m; reset clears residual; mode-off invariant; stale/future/invalid/31m outlier rejection and both correction caps verified. Native compilation and all replay/scoring commands completed. No local ROS environment: node parameter declaration has not been compiled/executed in ROS here.

Post-run report-generation strict Python-dict equality failed because scorer aggregate scalar RMSE differed by1.39e-17 (0.05683832343503348 vs0.05683832343503347), despite the exact raw-speed/output invariant passing. This was a report-only assertion; no source, executable, input, parameter, metric mask or replay changed/reran. Aggregate equality is reported to displayed precision.

GNSS accepted/rejected counters are not exported by20-column CLI, so no bag counter totals are claimed. Gate outcomes are covered analytically. Risks: absorbed GNSS bias, phase/bias ambiguity, larger residual acceptance, persistent bias at standstill and fixed locality scale. Bias used for delayed-fix projection is current-epoch (max1s), without a historical bias buffer. Cumulative bias has no explicit total cap; per-fix cap and movement decay are the only bounds. These are review items, not tuned away.

## Exact sources, commands, hashes

Only3 runtime files changed: candidate/src/reserve_odometry/include/reserve_odometry/pipeline.hpp, candidate/src/reserve_odometry/src/pipeline_cli.cpp, candidate/src/reserve_odometry/src/odometry_node.cpp. The diff against original mainline is xy_decay100.patch. Native tests are xy_controls.cpp plus included legacy_native_controls.cpp; compile.cmd builds/runs them. Full native replay command:

```powershell
python -B historical-work/experiments/development/xy_decay100/run_pilot.py
```

The driver refuses existing output directories: retain these results and reproduce in a new copy. Exact estimator commands are in report.json; scorer commands and schedule in run_pilot.py.97-file pre-outcome seal: `ce8e488b2afd751bacd9fe9f1ec9bd9cf4cf7226890c035690edac7788b48f18`. Source/control metadata: verification.json. Full scored metrics: report.json. Final result file hashes: results.sha256.json. No post-seal snapshot mutation; parent handles integration and two nonauthor reviews.
