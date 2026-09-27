# Retry initialization: frozen candidate with exact regression parity

Only owned pipeline.hpp changes; mainline owns integration and final acceptance.15 synthetic startup tests and32 author controls pass. Independent verification checks reports27 controls pass on identical header hash. Original newchecker and26 fixed sparse outputs are exactly unchanged.

Runtime source SHA256:015e44e78196d7c82c34046deb1edfba0c68b63924f63cff957fd6f4b34deab4.

## Behavior and safeguards

After ordinary end-of-window candidate evaluation, only sparse mode with no absolute pose enters retry waiting. Old queues/candidates are cleared. No map projection occurs while waiting with no new fixes. A fresh eligible fix anchors separate init_header0_/init_arrival0_ for a bounded3s window. Global t0/arrival0, core/features/controller timeline, wheel integral and distance history are never reset. Projection offset remains p.s−global travelled_at(fix epoch). Once absolute initialization succeeds, normal closure and sparse correction continue.

Retry data preserves per-receiver watermark across windows and enforces nonnegative status, valid LLA, future tolerance, history lower bound and1s maximum age at arrival and processing. Retry candidates and alignment lists are capped at512, matching existing fix queue capacity; history remains capped1000. No-map preserves prior frozen behavior. Full reset clears retry state and watermarks. No single-receiver heading fallback.

The first fresh header defines a strict window start; an earlier-stamped partner arriving second can be conservatively rejected. Later valid pairs can recover. Discarded cross-window observations are never paired. A still-uninitialized retry emits flags8+16 (relative/open) rather than the old permanent flag8; this affects only intentional failed-start behavior. A default-off legacy initialization failure still freezes exactly as before.

## Native regression proof

Newchecker provided output text is byte-for-byte identical to selected decay100: True. All20 decoded columns are bit-exact. Legacy sparse-disabled replay is bit-exact. Every one of26 fixed sparse-regression bags has every20-column float bit and original int64 stamp unchanged; no previously-uninitialized row changed. This covers exact sample masks, flags, positions, scalar speed, distance and derived body outputs.

Baseline official0ms score therefore remains XYZ RMSE6.114866553686m/max43.088026895273m and scalar RMSE0.056838323435m/s. The parity job copied the unchanged baseline score with provenance; it did not rerun the scorer or claim actual DDS evidence. Old validation30618 quality stays2.748229880791m. No comparison thresholds were tuned.

| Startup scenario | Old first absolute,s | Retry first absolute,s | Pass |
|---|---:|---:|---|
| clean_pair | 0.1 | 0.1 | True |
| master_only_initial_later_both | -1 | 4 | True |
| rover_only_initial_later_both | -1 | 4 | True |
| async40ms | 0.15000000000000002 | 0.15000000000000002 | True |
| async50ms_boundary | 0.15000000000000002 | 0.15000000000000002 | True |
| async50ms_plus1ns | -1 | 4 | True |
| first_pair3point1s | -1 | 3.1 | True |
| pair_header2point95_arrival3point1s | -1 | 3.1 | True |
| pair_exact3s_before_step | 3 | 3 | True |
| pair_exact3s_after_step | -1 | 3.0500000000000003 | True |
| invalid_master_status | -1 | 4 | True |
| precommand_pair_header | -1 | -1 | True |
| degraded_two_pairs | -1 | -1 | True |
| degraded_three_pairs | 3 | 3 | True |
| midroute500m_clean_pair | 0.1 | 0.1 | True |

The adapted old missing-startup test now requires recovery only from a fresh valid pair in sparse=true mode. The original nonrecovery assertion remains a separate sparse=false negative control. A separate pending-retry reset test passes. Single invalid/precommand-only and insufficient degraded observations still do not fabricate an absolute estimate. Native tests are not ROS/DDS execution.

## Artifacts and reproduction

Frozen pre_outcome_seal.json includes source, inputs, binary, tests and baselines before parity results. Runtime diff: retry_initialization.patch. Test contract amendment: legacy_contract_amendment.patch; legacy_native_controls_original.cpp preserves old fixture. Native tests: legacy_native_controls.cpp,xy_controls.cpp,startup_controls.cpp. mainline test filenames/include paths may need rebasing; preserve both sparse-mode assertions.

```powershell
python -B historical-work/experiments/development/retry_initialization/run_parity.py
```

Driver refuses existing outputs; reproduce in a fresh isolated copy. report.json/run.log hold all26 results; compile_controls.log/startup_controls.log hold native checks. No runtime or experimental assertion failed. A report-packaging command had a corrected syntax typo only; no sealed source or experiment was changed or rerun for that correction.

Remaining acceptance work belongs to mainline: actual DDS late-start regression, final clean build and two nonauthor review slots. Repeated single-receiver availability can remain unobservable; this fix only makes valid later dual observations recoverable. No active job remains.
