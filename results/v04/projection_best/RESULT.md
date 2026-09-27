# REJECTED: best-so-far XY projection pilot

Finished2026-09-27 before14:48 Moscow. Keep the previously selected100m-decay runtime. This isolated branch repairs the analytic circle corner case, but fails the preregistered checker non-regression criterion. No tuning, criterion changes, second candidate or mainline integration. Numerical run took28.772993999999017s at one BelowNormal thread.

## Fixed result

| Criterion | Selected decay100 | Best projection | Fixed threshold | Result |
|---|---:|---:|---:|---|
| New checker0ms XYZ RMSE,m | 6.114866553686 | 6.114931317974 | 6.114867600000 | FAIL |
| Validation30618 quality XYZ RMSE,m | 2.748229880791 | 2.748230028137 | 2.748230900000 | PASS |

Checker increase is only0.000064764288m (about0.065mm) in aggregate RMSE. This is not a claim of practically meaningful degradation; it still violates the exact fixed criterion and the branch is rejected. New checker3ms diagnostic similarly changes6.115977597475→6.116042591156m. Newchecker maximum43.088026895273→43.088221703679m at both latencies.

Core/scalar speed, integrated distance and output timestamps remain exact. Legacy phase-only output matches all20 frozen phase-only checker columns exactly. New checker coverage equals selected baseline; all26 old output/init/reference masks and counts match selected baseline. These are native assumed-latency results, not real DDS.

## Change and analytic reproduction

Only pipeline.hpp changes. When gnss_xy_residual_correction=true, retain initial phase/cost and evaluate the existing five bounded Gauss-Newton iterates; choose strictly lower XY residual, keeping initial on ties. Old phase-only path follows its prior five-iteration result. No extra iteration, gate, gain, decay, speed or model/map change.

Exact verification checks original case was preserved: R30 circle, phase=start+20m at4s, master antenna displaced12m radially outward. Original selected runtime reproduces its single expected failure (31 other tests PASS): fix queued but accepted0/rejected1 and no bias. Candidate passes the same32 tests: bias3.000000000080m, accepted1/rejected0, innovation12.000000000321m, route phase49.999999999783m. It also passes all30 existing author controls.62 PASS executions include19 duplicated legacy tests; there are43 distinct named cases. Include path was the only fixture edit.

Input cost is an explicit candidate in the minimization, so selected pre-update residual cannot exceed initial residual; the existing non-worsening guard remains active. Outlier, invalid/stale/future inputs, default-off/no-fix invariance, reset, standstill and permanent initial alignment controls pass. No unanticipated execution/test failures. The baseline radial failure was deliberately reproduced; the scored primary gate failure is the reason for rejection.

## All old26 evidence, selected decay baseline

Fixed sparse schedule unchanged (first3s, master1s/120s starting120s, rover1s/120s starting60s). References/scorers unchanged. Fields named phase in report.json mean selected_decay100 and fields xy mean projection_best, explicitly mapped by arm_names; legacy_phase_control is separate. Exposed holdout is audit data, not independent holdout.

| Group | Quality n | Selected quality RMSE | Best quality RMSE | Selected quality max | Best quality max | Selected raw RMSE | Best raw RMSE |
|---|---:|---:|---:|---:|---:|---:|---:|
| all_30618 | 347778 | 4.064027897277 | 4.064027294668 | 47.154750937357 | 47.154750936584 | 4.290680343466 | 4.290691846493 |
| validation_30618 | 252362 | 2.748229880791 | 2.748230028137 | 26.065268595701 | 26.065173508980 | 3.114838523195 | 3.114857901587 |
| holdout_30618 | 95416 | 6.342213741222 | 6.342212164903 | 47.154750937357 | 47.154750936584 | 6.450016185147 | 6.450019292844 |
| all_30639 | 161465 | 7.293431077757 | 7.293414510351 | 86.650014090818 | 86.650020173743 | 7.534260001887 | 7.534243935709 |
| all_all | 509243 | 5.305252910548 | 5.305245373691 | 86.650014090818 | 86.650020173743 | 5.547624554487 | 5.547623547906 |

Every worsening raw/quality RMSE/max entry follows (37 entries). Tiny floating-point differences are retained, not filtered. Full26 rows in per_bag.csv/report.json.

| Bag | Split | Metric | Statistic | Selected | Best |
|---|---|---|---|---:|---:|
| 30618_01f73500 | validation | xyz_valid | rmse | 3.697297243380 | 3.697387966465 |
| 30618_27e994fc | holdout | xyz_valid | rmse | 4.252689791812 | 4.252719491705 |
| 30618_27e994fc | holdout | xyz_quality | rmse | 3.359197921960 | 3.359203690925 |
| 30618_33bec73f | validation | xyz_valid | rmse | 1.076640428174 | 1.076664918831 |
| 30618_33bec73f | validation | xyz_valid | max | 19.507453705643 | 19.507453705732 |
| 30618_33bec73f | validation | xyz_quality | rmse | 1.060915212762 | 1.060940609123 |
| 30618_33bec73f | validation | xyz_quality | max | 6.360313136849 | 6.360461785094 |
| 30618_3e9f4952 | validation | xyz_valid | rmse | 4.790061793265 | 4.790082317081 |
| 30618_3e9f4952 | validation | xyz_quality | rmse | 4.213117845852 | 4.213144663666 |
| 30618_3e9f4952 | validation | xyz_quality | max | 20.318272180429 | 20.318390347012 |
| 30618_437e855c | validation | xyz_valid | rmse | 3.514700439908 | 3.514710507285 |
| 30618_437e855c | validation | xyz_valid | max | 17.232107521817 | 17.232107565228 |
| 30618_437e855c | validation | xyz_quality | rmse | 3.440669409357 | 3.440673349117 |
| 30618_437e855c | validation | xyz_quality | max | 17.232107521817 | 17.232107565228 |
| 30618_dd8d0741 | validation | xyz_valid | rmse | 3.857708165327 | 3.857714633178 |
| 30618_dd8d0741 | validation | xyz_valid | max | 25.304770172093 | 25.304770388236 |
| 30618_dd8d0741 | validation | xyz_quality | rmse | 3.454783391797 | 3.454790046555 |
| 30618_dd8d0741 | validation | xyz_quality | max | 20.330908419386 | 20.330930859790 |
| 30618_e9a34502 | validation | xyz_valid | rmse | 1.431810800202 | 1.432018418645 |
| 30618_e9a34502 | validation | xyz_quality | rmse | 1.431810800202 | 1.432018418645 |
| 30639_0be558e2 | validation | xyz_valid | rmse | 7.622141006832 | 7.622151099707 |
| 30639_0be558e2 | validation | xyz_valid | max | 52.930414420817 | 52.930414450326 |
| 30639_0be558e2 | validation | xyz_quality | rmse | 5.436622718185 | 5.436645192938 |
| 30639_4285f2bc | validation | xyz_valid | rmse | 17.513455620263 | 17.513472653119 |
| 30639_4285f2bc | validation | xyz_valid | max | 86.650014090818 | 86.650020173743 |
| 30639_4285f2bc | validation | xyz_quality | rmse | 18.256375089540 | 18.256394147436 |
| 30639_4285f2bc | validation | xyz_quality | max | 86.650014090818 | 86.650020173743 |
| 30639_50956d6e | validation | xyz_valid | rmse | 5.977390747000 | 5.977396669980 |
| 30639_50956d6e | validation | xyz_valid | max | 23.492593350576 | 23.492593350650 |
| 30639_50956d6e | validation | xyz_quality | rmse | 2.531361527360 | 2.531369796158 |
| 30639_50956d6e | validation | xyz_quality | max | 18.824747583300 | 18.824747583303 |
| 30639_584b6e32 | validation | xyz_valid | rmse | 2.639318861968 | 2.639335667048 |
| 30639_584b6e32 | validation | xyz_quality | rmse | 2.613142056676 | 2.613160719026 |
| 30639_9f0b519f | validation | xyz_valid | max | 6.873480809614 | 6.873481344420 |
| 30639_9f0b519f | validation | xyz_quality | max | 6.873480809614 | 6.873481344420 |
| 30639_d927f360 | validation | xyz_valid | rmse | 1.523874845755 | 1.523874917614 |
| 30639_d927f360 | validation | xyz_quality | rmse | 1.526041442704 | 1.526041520024 |

## Frozen artifacts and commands

PLAN.md preceded implementation/outcomes. experiment.sha256.json sealed108 files before scoring; all hashes still valid. Source/config snapshot remains unchanged after seal. Diff: projection_best.patch; source hashes and full test lists: verification.json. Original critic fixture: critic_original.cpp; rebased copy: critic_fixture.cpp. Baseline/candidate logs: critic_baseline.log,critic_controls.log; original30 controls: compile_controls.log.

Compile via compile.cmd and compile_critic.cmd. Replay from workspace mainline:

```powershell
python -B historical-work/experiments/development/projection_best/run_pilot.py
```

Replay driver refuses existing output directories; reproduce in a fresh isolated copy. No process remains. Parent owns integration and review; this rejected branch is not release code. Runtime author: speed_30618; fixed design/criterion from mainline; triggering independent regression fixture from verification checks.
