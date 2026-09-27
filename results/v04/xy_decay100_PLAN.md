# Fixed spatial-decay alternative — frozen 2026-09-27 14:19 Moscow

Predeclared single alternative after mixed global-shift pilot; no grid/tuning.
Hypothesis: persistent global XY bias transports a local map error into later
locations. Fix one decay length100m, chosen as order of local terminal map error
extent (~200m from prior geometry audit), not fitted from score labels.

Keep optional gnss_xy_residual_correction=false. Residual gate30m in enabled mode,
innovation30m, age1s, gain0.25, phase cap5m and XY vector cap5m remain unchanged.
Use separate gnss_xy_bias_ state initially zero; never modify initial map_shift_
learned with align_initial_position. Before GNSS correction at every step, decay
bias by exp(-max(0,delta integrated distance)/100m). No standstill decay. Use
map_shift_+bias for projected antenna and output position; bias Z always zero.
No speed/core/integral changes; reset clears bias. Phase update remains as before.

Before real-bag outcomes run legacy27 controls plus straight singlecross3=>.75m,
then100m travelled=>.75/e; stop/standstill holds bias; reset; default-off and
initial alignment permanence. Failures stop branch before score.

Freeze source/driver/binary hashes before replay. One original new bag at native
0ms/3ms scorer assumptions and same26 frozen sparse-regression runs; scalar core
mode only, new bag map transform, old bags ENU. Existing labels only scorer.
Exactly one fixed alternative; no adjusting100m/gain/gates to observed results.

Stricter criterion: official XYZ RMSE improves at both delays versus phase-only;
validation30618 quality XYZ RMSE <=3.6273009379690125 (original3.6272999379690125
plus1e-6 tolerance), and core/distance unchanged. Publish perbag/all raw-quality
max/RMSE regressions and split results. This remains exposed development evidence,
not independent holdout nor acceptance. mainline handles integration/non-author reviews.

Risks: GNSS bias, phase-vs-bias ambiguity, uncertain local map extent, correction
relaxation30m, combined independent phase+XY cap can give7.071m straight jump,
fixed100m spatial decay can remove valid long-range alignment. No ROS locally.
