# Fixed retry-initialization branch —2026-09-27 14:51 Moscow

Only sparse competition mode, only while no absolute pose exists. After failed
3s dual-GNSS initialization, clear its queues/candidates and wait for a fresh
eligible fix to anchor a separate3s header+arrival window. Keep global t0,
arrival0, features/core/distance/command/history/output clock untouched. Initial
successful and sparse-disabled behavior must remain exact. No single-fix heading.

Retry guards: finite/status/future checks unchanged; enforce retained-history
lower bound, maxage1s, per-receiver monotonic watermark across retries. Clear
previous-window GNSS queues/candidates, keep watermarks. Anchor header and arrival
from first fresh retry fix. Subsequent fix must lie inside those anchored windows.
Strict header start can conservatively reject an earlier-stamped partner arriving
second; no cross-window pairing and no invented heading. Retry only on data,
no repeated map projections while waiting. Bound retry candidate/shift lists to
existing512fix capacity. No-map retains old frozen behavior. Full reset clears
retry state and watermarks.

Tests before score:15startup cases updated for intended late recovery, old30
native controls, noGNSS/no-map/reset, stale-history/future/duplicate/boundary.
Compare native original newbag+26 sparse outputs byte-exact to selected decay100
where previously initialized; disclose any previously-uninitialized changes.
Also legacy sparse-disabled output exact. Existing score labels only evaluation.
Deadline15:03; reject if guard/invariance unproven, no mainline runtime/CI edits.
