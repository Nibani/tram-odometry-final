# Fixed six-arm GNSS stress — 2026-09-27 14:33 Moscow

Artificial stress of the selected immutable100m-decay runtime; this checker bag
is already exposed. No runtime/map/model change, parameter selection, fitting,
or acceptance claim. mainline authorizes one numerical CPU thread for under2min.

The same original record/id-ordered events are filtered without reordering.
Initial window is every GNSS fix header <= first command header +3s, including
pre-command fixes. All initial GNSS is kept unchanged in every arm. All non-GNSS
events, header/record times and GNSS status/longitude/altitude stay unchanged.

Six arms, exactly once:
1. provided: original input bytes.
2. no_late: drop all GNSS with header > initial-window boundary.
3. even_bursts: late fixes sorted jointly across both antennas by header; split
   when adjacent header gap >2s; zero-based burst IDs, keep even IDs.
4. odd_bursts: same burst IDs, keep odd IDs. Initial window remains kept.
5. outlier_north: retain all; add exactly0.001deg to every late GNSS latitude.
6. bias_north: retain all; add exactly3/111320deg to every late GNSS latitude.

Burst segmentation is offline input-schedule construction, not estimator logic.
It uses GNSS headers only, never reference truth. Input byte edits touch only
latitude tokens for perturbations. Saved source-line index/record/id audit maps
preserve original order. Count kept/dropped/injected fixes separately per receiver
and initial/late window, with every burst boundary and membership recorded.

Before replay: snapshot current candidate executable, two route maps and map
calibration; seal hashes of script/plan/runtime/input/reference/scorer metadata.
Reference unchanged and scorer-only. Native CLI flags: --loop --speed-scale
1.0003350854241781 --map-transform exact existing calibration --sparse-gnss
--xy-residual, no body-velocity. Scorer unchanged, map absolute position,0ms.

Required checks: all arms have exactly identical core/scalar velocity, integrated
distance and output timestamps; same initialized mask and coverage. Outlier arm
must exactly match no_late position/core if innovation gates reject perturbations.
Report every failed assertion and all six XYZ/speed RMSE/max values; no adjusting
the perturbations/thresholds based on outcomes. This is native scheduling evidence,
not actual ROS/DDS timing and not a certification of arbitrary GNSS faults.
