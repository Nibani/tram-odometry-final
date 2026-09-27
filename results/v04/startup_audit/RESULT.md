# Startup audit: initial GNSS alone is an insufficient condition

Selected pre-retry runtime was inspected and tested without edits. All15 fixed synthetic expectations and four artificial record masks completed. This audit identifies a recoverability defect, not a claim that a single static fix determines pose.

Current initialization requires both receiver fixes with nonnegative status, finite valid LLA and timestamps. Both headers must fall within[first command header,first command+3s], both arrivals must meet the corresponding3s arrival window, and paired header separation must be<=50ms. Raw antenna baseline length error<=0.5m is clean; larger error may use degraded fallback only if<=25% of12.436m and at least3 candidates exist, evaluated at window end. Selected map projection median squared cost must be<400. Quality/covariance is otherwise not consumed by this initializer.

A valid clean pair can initialize immediately. At3s the old code freezes and clears initialization state even if absolute=false. Later fix() then rejects every fix because absolute=false. With publish_relative_position=false, scalar speed continues but no XYZ odometry is published. This state persists until reset. A pair at exactly3s succeeds before the closing step and fails if delivered after it.

| Synthetic case | First absolute output,s | Final initialized | Final frozen | Expected behavior verified |
|---|---:|---|---|---|
| clean_pair | 0.1 | True | True | True |
| master_only_initial_later_both | -1 | False | True | True |
| rover_only_initial_later_both | -1 | False | True | True |
| async40ms | 0.15000000000000002 | True | True | True |
| async50ms_boundary | 0.15000000000000002 | True | True | True |
| async50ms_plus1ns | -1 | False | True | True |
| first_pair3point1s | -1 | False | True | True |
| pair_header2point95_arrival3point1s | -1 | False | True | True |
| pair_exact3s_before_step | 3 | True | True | True |
| pair_exact3s_after_step | -1 | False | True | True |
| invalid_master_status | -1 | False | True | True |
| precommand_pair_header | -1 | False | True | True |
| degraded_two_pairs | -1 | False | True | True |
| degraded_three_pairs | 3 | True | True | True |
| midroute500m_clean_pair | 0.1 | True | True | True |

Master-only/rover-only startup cases included a later valid dual pair at4s; it was still rejected.40ms and50ms skew worked,50ms+1ns did not. A moving midroute500m start initialized correctly with a valid pair; absolute starting coordinate is not inherently tied to the route start. All cases kept exactly identical speed/distance.

| Fixed real-bag mask | GNSS fixes dropped | XYZ outputs | XYZ pairs | XYZ RMSE |
|---|---:|---:|---:|---:|
| provided | 0 | 26188 | 26188 | 6.11486655368626 |
| startup_master_only | 33 | 0 | 0 | None |
| startup_rover_only | 30 | 0 | 0 | None |
| no_initial | 63 | 0 | 0 | None |

Initial master-only drops33 initial rover fixes; rover-only drops30 master fixes. Later GNSS remains untouched. No_initial drops63 initial fixes. All timestamps/order/status and reference bytes unchanged. Scalar/core/distance remain exact. Empty XYZ RMSE is undefined (null), not zero success. These are artificial exposed-data masks and native0ms timing.

## Safe fallback recommendation

Retry bounded dual-antenna initialization windows only while no absolute estimate exists and sparse competition mode is enabled. Retain the original core/wheel/global timestamp/distance history; offset must remain projected phase minus global travelled distance at the fix epoch. Anchor a new window on fresh eligible retry data, discard old candidates/queues, preserve receiver monotonic watermarks, reject stale/out-of-history/future/invalid fixes and keep memory bounded. Valid startup and legacy sparse-disabled behavior must be exact. This is implemented separately in ../retry_initialization; no authoritative runtime was edited here.

If only one receiver is ever available, even this fallback cannot establish the required dual-antenna heading. A single static antenna observation plus map can have multiple route/direction/lever-arm hypotheses. A future single-receiver fallback would require observed motion and explicit ambiguity handling; no such inference is part of this audit or fix. The organizer statement needs clearer paired-availability semantics to guarantee absolute initialization in every permitted case.

Artifacts: PLAN.md,pre_outcome_seal.json,startup_controls.cpp,synthetic.log,run_audit.py,report.json and outputs/*/score.json.15 synthetic assertions PASS, no execution failures. No accepted runtime or coordinates/heading were fabricated.
