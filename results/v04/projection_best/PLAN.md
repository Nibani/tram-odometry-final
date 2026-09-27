# Fixed best-so-far projection pilot —2026-09-27 14:38 Moscow

Predeclared independent branch; current release/CI runtime untouched.
One change only: in gnss_xy_residual_correction mode, retain initial phase and
XY residual cost, evaluate each of the existing five Gauss-Newton iterates,
select the phase with minimum XY residual (strict improvement, initial wins ties).
Legacy phase-only mode is strictly unchanged. No extra iterates, gates, gains,
parameters or model/map changes.100m decay and separate gain/caps unchanged.

Reason: verification checks found exact-phase radial+12m antenna observation on circle
R30 gets conservatively rejected because the final nonmonotonic iterate is worse
than the initial phase. Keeping the best iterate cannot worsen projected residual
and avoids dropping otherwise valid measurements. This is algorithmic repair,
not fitting to checker labels. Exact critic initial fixture must reproduce.

Before bag outcomes: original30 native controls, exact-phase circle radial12m
accepted with no-worse residual, world-axis decay, invalid/outlier rejection,
legacy phase behavior invariant. Copy critic's original failing fixture unchanged
for comparison; compile baseline and new candidate, disclose both results.

Snapshot selected mainline candidate src/config/scripts, write hashes, compile and
seal all inputs/code/binary before scoring. Exactly one new checker replay (plus
legacy-disabled control), map transform, core scalar,0ms primary/3ms diagnostic.
Exactly same old26 sparse schedule/references. Compare selected100m-decay outputs,
not global-bias or phase-only quality. Labels used only by unchanged scorer.

Fixed default criterion: newchecker0ms XYZ RMSE <=6.1148666+1e-6;
validation30618 quality XYZ RMSE <=2.7482299+1e-6; speed/distance/coverage exact.
All perbag/max/raw-quality regressions disclosed, including null effect. No
further tuning if criterion fails; mainline decides integration only after review.
One numerical CPU slot, one thread, BelowNormal, finish before14:48 Moscow.
