# Startup guarantee audit —2026-09-27 14:48 Moscow

Read-only audit of selected immutable100m-decay runtime. No runtime changes,
parameter search, new acceptance claim, or reference-driven construction.

Fixed synthetic analytic straight route,5m/s, known dual antenna lever arms,
first command atT0,20Hz calls, default3s initialization, selected sparse+XY flags.
Cases: clean paired fixes; master-only; rover-only;40ms/50ms/50ms+1ns skew;
first pair at3.1s; predeadline pair delivered3.1s; exact3s before-vs-after-step;
invalid status; negative/pre-command stamp-relative pair; two-vs-three mildly
degraded baseline pairs; and clean pair at mid-route500m. Physical positions
are synthesized at each exact header epoch with independent inverse WGS84.
No static one-fix heading/position inference is assumed. Track first absolute
output, final initialized/frozen flags, accepted/rejected queue calls, position
error against analytic truth and core/distance invariants through6s.

Fixed real-bag masks, each one native replay and unchanged0ms scorer:
provided; startup_master_only (drop rover fixes atheader<=T0+3s, later all kept);
startup_rover_only (drop master initial); no_initial (drop both initial, keep
later GNSS). Original order/status/header/record fields unchanged. Reference
never used to construct masks. Report coverage even when no XYZ outputs, avoiding
misleading zero RMSE from empty arrays. Native results, not actual DDS.

Seal code, inputs, selected source/executable/maps and PLAN before outcomes.
One CPU thread/BelowNormal, brief native compile/controls and four replays.
Describe exact required pair/skew/status/time/arrival/baseline/map-fit conditions,
permanent failure state vs delayed initialization, and a safe fallback design.
No runtime implementation of a fallback in this branch. Deadline15:00 Moscow.
