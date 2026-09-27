# Fixed six-arm GNSS stress: invariants pass

Completed2026-09-27 before14:45 Moscow. Artificial stress of selected immutable XY-decay100 runtime on an already exposed checker bag. No changes or selection of runtime parameters, GNSS schedules or perturbation magnitudes followed outcomes. Runtime12.571s, one BelowNormal numerical thread. No acceptance claim.

## Scores at native assumed0ms publication latency

| Arm | Late master/rover kept | Late dropped | Injected | XYZ RMSE,m | XYZ max,m | Scalar speed RMSE,m/s |
|---|---:|---:|---:|---:|---:|---:|
| provided | 408/402 | 0 | 0 | 6.114866554 | 43.088026895 | 0.056838323435 |
| no_late | 0/0 | 810 | 0 | 8.283306247 | 52.766909242 | 0.056838323435 |
| even_bursts | 362/337 | 111 | 0 | 6.048313196 | 43.087479119 | 0.056838323435 |
| odd_bursts | 46/65 | 699 | 0 | 7.059735201 | 50.987062165 | 0.056838323435 |
| outlier_north | 408/402 | 0 | 810 | 8.283306247 | 52.766909242 | 0.056838323435 |
| bias_north | 408/402 | 0 | 810 | 6.355400345 | 43.394127549 | 0.056838323435 |

All33 declared invariants PASS. Outlier_north and no_late are exactly identical across all20 output columns, consistent with rejecting the large latitude perturbation. Every arm has identical int64 timestamps, scalar/core velocities, integrated distance, initialized masks, coverage and first-position publication metadata. Reference bytes are unchanged. Zero nonfinite values in all reported scalar/XYZ metrics.

All arms use26249 outputs, 26233 velocity pairs and26188 position pairs. Initialized outputs:26188; initial relative positions intentionally suppressed:61. Scalar maximum error is0.323603712090m/s for all arms. Full axes, initialization-window masks, matching queues and first-position metadata are retained in run/report.json and each arm/score_0ms.json.

## Input construction and provenance

Each arm preserves all30 master+33 rover fixes with header<=first command header+3s, including pre-command fixes. There are810 late fixes:408 master+402 rover. GNSS fields other than selected latitude, all header/record times/status values and all non-GNSS inputs remain unchanged. Original record/id order was checked against inputs.npz before editing and retained by source-line subsetting. Every arm saves exact source-line index, record_ns and message_id arrays. Provided input is byte-for-byte original.

Late burst windows are formed from jointly header-sorted master/rover fixes; strictly>2s gap starts a new burst. There are19 zero-based bursts: even windows retain699 late fixes and odd windows111. This is intentionally not a balanced50% sample; the predefined rule was not adjusted. Large perturbation adds exactly0.001degree latitude; small perturbation adds exactly3/111320degree. Reference values were never opened when constructing perturbations. Only its byte hash was computed.

| Burst ID | Start since first command,s | End since first command,s | Master fixes | Rover fixes |
|---|---:|---:|---:|---:|
| 0 | 3.041471 | 32.741471 | 277 | 286 |
| 1 | 201.841471 | 202.841471 | 11 | 0 |
| 2 | 211.841471 | 213.041471 | 0 | 13 |
| 3 | 299.841471 | 301.241471 | 15 | 0 |
| 4 | 312.841471 | 314.541471 | 0 | 18 |
| 5 | 438.741471 | 439.441471 | 0 | 8 |
| 6 | 477.841471 | 479.041471 | 13 | 0 |
| 7 | 531.841471 | 533.541471 | 0 | 18 |
| 8 | 627.841471 | 629.041471 | 13 | 0 |
| 9 | 675.841471 | 676.841471 | 0 | 11 |
| 10 | 742.841471 | 744.141471 | 14 | 0 |
| 11 | 839.841471 | 842.441471 | 0 | 14 |
| 12 | 894.841471 | 896.441471 | 17 | 0 |
| 13 | 979.841471 | 980.341471 | 0 | 6 |
| 14 | 1032.841471 | 1033.641471 | 9 | 0 |
| 15 | 1118.741471 | 1119.441471 | 0 | 8 |
| 16 | 1139.841471 | 1141.641471 | 19 | 0 |
| 17 | 1275.841471 | 1277.741471 | 20 | 0 |
| 18 | 1285.741471 | 1287.641471 | 0 | 20 |

## Interpretation and limitations

Late GNSS helps this exposed trajectory: removing it increases XYZ RMSE6.115→8.283m. The even-burst result is slightly better than provided, so more fixes is not monotonic improvement. That schedule was not selected as a runtime behavior. The small northward bias increases XYZ RMSE to6.355m and max to43.394m. Large faults are gated in this prescribed scenario; this does not prove protection against arbitrary small/moving GNSS biases or other error directions.

These use the unchanged native arrival-ordered oracle with0ms assumed output delay. They are not real DDS measurements. Runtime GNSS accepted/rejected counters are not exported; exact output equality is the large-outlier evidence. Offline future-aware burst segmentation only constructs artificial missing-data arms and is never estimator input logic. Three spatially directional perturbations/drop rules do not exhaust robustness. No assertion, scorer or execution failure occurred.

## Portable reproduction

Prepare snapshots/seal before any replay, then execute the same six arms. Paths are explicit; choose a new empty OUT to preserve existing results. Requires Python+NumPy and the supplied platform-compatible native executable. Current frozen executable is Windows PE; Linux reproduction requires the same source to be built for Linux and a new provenance seal.

```text
python -B stress.py prepare --candidate CANDIDATE --decoded DECODED --harness CHECKER_HARNESS --out OUT
python -B stress.py run --candidate CANDIDATE --decoded DECODED --harness CHECKER_HARNESS --out OUT
```

Prepared runtime executable/routes/calibration are copied under OUT/runtime; replay uses those immutable copies. Exact estimator/scorer commands are in run/report.json. PLAN.md and stress.py were sealed before outcomes. Original input SHA256:4e38a3f8f1d48ffd78933c5af92d05d9d1a10ffd4a0cea9bcf53203b6bf05089. Reference SHA256:e87e8cc53a5059bb34bab60689bf04505ed47d8ad065aec48e57d8144d547ec9. Runtime executable SHA256:6086cb5c11a73be061a5fb2c5eb164ece12444de5e0623c560fdae593ca3ff26. Pre-outcome seal SHA256:d2f808b4993d6ed2cd5be9698c512fc77f20a38a9feefaee0f0b89dd1b10b2a4. Full input/runtime/source/scorer hashes are in run/seal.json and run/design.json; complete results hashes in run/result_hashes.json.

The experiment and integration checks are documented separately; this historical experiment is not a final acceptance report.
