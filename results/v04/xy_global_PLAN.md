# Frozen XY-residual pilot — 2026-09-27 14:10 Moscow

Authorized bounded branch; stop by14:30. mainline candidate remains untouched.
Snapshot competition_v04/candidate src/config/scripts into this owned directory.
No change to maps, learned weights, wheel/core model, input schedule, labels or
existing off/phase-only behavior. One numerical job, one thread, BelowNormal.

Hypothesis: local cross-track map errors remain after GNSS phase projection.
Add optional disabled-by-default gnss_xy_residual_correction. Preserve existing
five-iteration antenna XY projection and phase update gain.25, max5m. In new mode
only, post-projection residual gate equals existing innovation gate30m rather than
legacy8m. Pre-projection innovation gate stays30m; max age1s. After fully projected
antenna prediction p(s), delta_xy=.25*(fix.xy-p.xy), vector-norm-clipped to5m.
Add delta_xy to map_shift_.xy; never change its Z. Keep map selection fixed and
never feed translation into wheel/core speed or integrated distance.

Controls before any real-bag outcome: straight route, valid initialization,
single master/rover cross-track3m=>.75m shift; phase-only output unchanged on this
cross-track observation; >30m outlier rejected; stale/future/negative-status fixes
rejected; large valid20m cross-track innovation has5m bounded shift; Z remains
unchanged; scalar/core velocity and distance identical. New mode off must exactly
match mainline baseline on new-bag command rows. Fix-gate effects and all failures
will be reported; no threshold or gain search.

Then exactly one new-bag replay, mainline frozen inputs, map-transform enabled and
body-velocity disabled. Independent checker_harness score at0ms and3ms assumed
publication delays, absolute-position-only, same official all-pair masks. Compare
phase-only mainline checker_core baseline. Proceed only if official XYZ RMSE improves
at both latencies (mainline indicative0ms6.95356m), all maxima/counters disclosed, and
core speed/distance remain exact. Labels are only scorer inputs, never runtime.

Only after that criterion passes, run the unchanged26bag sparse_regression
schedule: first3s, master1s every120s starting120, rover1s every120s starting60.
Compare frozen phase-only outputs to new mode; off invariance checked separately.
Use exact legacy quality/raw references after3s and initialized only. Primary old
criterion: pooled quality XYZ for30618 across these exposed validation/audit bags
must improve. Publish separate validation/audit,30618/30639/all, per-bag RMSE/max,
counts and every regression. No clean independent holdout claim; no R/E acceptance.

Risk: shifting an uncertain map can absorb GNSS bias; phase/translation are partly
ambiguous, offsets persist through outages, sparse fixes cannot identify geometry
shape, and relaxing residual rejection permits erroneous GNSS under30m. A pilot
PASS is not deployment acceptance. Stop on FAIL without expanding or tuning.
