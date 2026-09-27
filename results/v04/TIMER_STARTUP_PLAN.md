# Pre-outcome timer-startup correction, 2026-09-27 14:59 Moscow

Actual1x CI36315880930 emitted one velocity at the first bag record timestamp,
which was2.820757189s after the oldest command header, then omitted exactly
commands0..56. All later command headers were present. This is consistent with
a fallback timer advancing past the initial command backlog after a wheel
callback at ROS clock0. Saved outputs do not prove exact callback order.

Narrow integration change: on the first timer callback with valid ROS time,
if the initial input arrival was0, anchor its existing100ms startup grace to
that current time. Apply that grace independent of whether a command has
already started Pipeline. Do not reset Pipeline, skip inputs deliberately,
restamp messages or change wheel/GNSS/command contents. Existing command
callbacks continue to publish immediately using their actual header.

An independent deterministic actual-DDS fixture will deliver a wheel while
/clock is0, advance clock, make the timer runnable, then deliver a backdated
command backlog. Fixed code must not publish a timer stamp ahead of that queue,
and must preserve all expected command stamps. Existing missing-controller
recovery tests and full1x organizer example must still pass. The separate case
of a command itself received before /clock is not claimed solved by this patch;
late dual-GNSS retry addresses recovery, not reconstruction of missing inputs.

Pre-change node SHA256 a5d6a36b9a4fb4958af9df1fb7f00b7b89ff67e66b72d3f4e4e6c9e7fce72181.
Native numerical predictions are unaffected (change is ROS adapter only).
No error/coverage thresholds are loosened; report actual counts after the new
run even if imperfect. Independent reviewers do not author this runtime.
