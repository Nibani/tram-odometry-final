# Causal geometry audit — 2026-09-27

Conclusion: the body-frame projection formula is correct for a correct rail
curve, but these inferred terminal maps do not support using their derivatives
as a precise velocity measurement. Retaining core speed as an explicitly
approximate longitudinal estimate, with map-derived body projection opt-in, is
currently more defensible than enabling that projection everywhere. This is a
model/uncertainty decision, not a fitted correction against the new checker bag.

## Cause, distinguished from the drive model

The reported worst location is wrapped hairpin phase501.149412973m, segment507.
Its vertex XYZ values are exactly identical between the old primary loop and
hairpin map. It is outside the six-coefficient hairpin deformation. This artifact
was already in the accepted TRAIN-derived terminal extension; the new body-x
publication exposes it rather than introducing a new drive-model error.

The segment changes(-.326528499,-.802847354,-.383539416)m in.947780153m. Its tangent
pitch is−23.870530°, while the7.55m bogie chord has pitch+7.192867°. The implemented
world-velocity dot body-forward is.846970134455 times core speed; body vertical
velocity is−.514771282037 times core speed. The independent NumPy reconstruction
matches the native diagnostic factor. The derivative norm is effectively1, so
this is not a numerical differentiation attenuation or a units issue.

Holding those two pitches but removing horizontal heading discrepancy gives
cos(−23.870530°−7.192867°)=.856596891471. Thus this controlled decomposition assigns
93.7% of the factor reduction to the vertical direction reversal. The remaining
horizontal heading mismatch is8.354°. A second location, wrapped44.307515m, is
inside the hairpin deformation and has a mostly horizontal29.73° discrepancy;
fixing Z alone cannot address every geometry-induced speed dip.

## Independent map and frozen TRAIN controls

No new labels or thresholds were fitted. geometry_audit.py reads the two frozen
maps, their declared official-span provenance, and the previously frozen TRAIN
kinematic samples. One thread, below-normal priority;1.12s numerical wall time.
A0.25m uniform map scan gives:

| Geometric source | Minimum body-x/core factor | Maximum tangent/chord angle | Local tangent pitch range |
|---|---:|---:|---:|
| Official provided spans, primary | .991786 | 7.35° | −5.12° to4.24° |
| Official provided spans, hairpin | .991283 | 7.57° | −5.12° to4.24° |
| TRAIN terminal extensions/seams, primary | .845867 |32.24°|−24.67° to17.73°|
| TRAIN terminal extensions/seams, hairpin | .840851 |32.77°|−24.67° to17.73°|

All36 sampled locations with factor<.95 on each map are in inferred terminal
geometry; none are on the official provided spans. Threshold.95 is descriptive,
not a proposed runtime gate. Fixed TRAIN central support has39bags/20,861samples,
minimum factor.992946 and maximum angle6.81°. The earlier successful geometry
unit tests and central-only research simply did not establish derivative quality
at the terminals. TRAIN provenance alone does not make a derivative reliable.

Independent analytic controls: a straight constant-grade rail has body-x/core=1,
not cos(grade). On a planar circle, body-x/core=sqrt(1−(7.55/(2R))²):.982025 atR20m,
.997146 atR50m, .999287 atR100m. The native geometry controls already pass these.
A pure change of world-frame yaw/translation cannot fix a body-frame factor.

## Why existing builders permit this

build_terminal_extensions.py builds3D base points from dual-antenna GNSS, samples
against wheel-integrated distance, then applies only a five-point moving mean.
This is position smoothing without rail-grade, curvature, or derivative-error
constraints. GNSS baseline-length quality does not certify the common vertical
position or its spatial derivative. The worst spike is in that inherited data.

The loop has approximate cubic-Hermite terminal joins. The hairpin refinement
minimizes robust XY positional residuals with a six-coefficient normal deformation
and C1 endpoint envelope; it preserves every Z value and imposes no curvature or
acceleration regularity objective. Good positional fit or initial antenna cost
therefore does not certify the local tangent used for speed projection.

## Options and limits

- Core speed fallback: preserve the independently validated wheel/drive estimator
  and keep body projection optional. Core speed approximates chassis-longitudinal
  speed; it is not an exact rigid-body component on all real curves. The current
  new-bag comparison is development-exposed evidence, not hidden generalization.
- A source-based alternative is to permit projection only on official provided
  map spans and use core speed throughout inferred terminals. This boundary comes
  from pre-existing map provenance, not the worst checker timestamps. It still
  needs an integrated replay, boundary behavior checks and review; it was not run
  or adopted by this worker. Do not substitute an error-tuned angle threshold.
- A lasting map repair can be TRAIN-only: robust3D curve estimation using multiple
  trajectories, heading evidence, and independently justified grade/curvature
  regularity; hold out whole TRAIN dates and validate derivatives as well as XYZ.
  Refit/check terminal connectivity, map selection, arclength and initialization
  consistently. No new-checker coordinate or velocity target should enter it.

Simply clipping body factors, increasing a finite-difference span, flattening Z,
or normalizing the derivative may hide this observed dip while making velocity
inconsistent with the position/attitude trajectory. A repaired smooth curve must
supply position, body axes and derivative together. Current sparse GNSS phase
correction can correct along-track offset; it cannot repair local curve shape.

Artifacts: geometry_audit.py/.json/.log, loop_geometry_fields.npz,
hairpin_geometry_fields.npz, existing body_speed_geometry.cpp/.log. No runtime or
map was edited. No process remains. This is a bounded causal diagnostic, not an
acceptance claim or a new geometry candidate.
