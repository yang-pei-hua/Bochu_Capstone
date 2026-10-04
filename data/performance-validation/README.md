# Performance-validation source models

This directory stores persistent source inputs for repeatable performance and
end-to-end reconstruction validation. Generated captures, COLMAP workspaces,
point clouds, and timing reports remain under `outputs/`.

## Cases

| Case | Source model | Purpose |
| --- | --- | --- |
| SK08-limobearing | `SK08-limobearing/SK08_limbearing.step` | STEP import, orbit capture, camera metadata, point-cloud reconstruction, and primitive-fitting validation |
| box-through-hole | `box-through-hole/model.step` | 60 × 40 × 20 mm box, centered Ø12 mm through hole; dimensioned CAD accuracy baseline |
| l-bracket-gusset | `l-bracket-gusset/model.step` | Fused base, upright and triangular gusset; compound-part capability probe |

## Reproduce the standard cases

Enable the repository validation executable in the existing client build (Qt,
VTK, OCCT, CGAL and vcpkg must already be configured):

```powershell
cmake -S "apps/ParamCAD Studio" -B "apps/ParamCAD Studio/build/msvc-debug" -DCAD_CLIENT_BUILD_VALIDATION_TOOLS=ON
cmake --build "apps/ParamCAD Studio/build/msvc-debug" --config Release --target cad_reconstruction_validation
python tools/generate_standard_models.py
```

The generator reads each `case.json`, writes its `model.step`, verifies one valid
solid after STEP round-trip, and checks analytic volume and bounding dimensions.
It also generates `textures/standard_capture.png`: a 1024 × 1024 deterministic
multiscale pattern, seed 4500, using only Python's standard library. Its texture
coordinates remain attached to the model during capture.

Run each case with a **new or empty** output directory:

```powershell
python tools/run_performance_validation.py --model data/performance-validation/box-through-hole/model.step --texture data/performance-validation/textures/standard_capture.png --output outputs/performance-validation-box-through-hole --gpu
python tools/run_performance_validation.py --model data/performance-validation/l-bracket-gusset/model.step --texture data/performance-validation/textures/standard_capture.png --output outputs/performance-validation-l-bracket-gusset --gpu
```

The full path is 62-view capture → camera metadata validation → GPU dense COLMAP
cloud → preprocessing → analytic evidence → production box/through-hole
recognition → atomic ModelingCore patch → valid B-Rep → STEP export and read-back
→ source-model accuracy checks. CAD export and accuracy checks run when semantic
recognition succeeds. `--no-gpu` deliberately selects CPU sparse reconstruction.

## Compact diagnostics, without loading engine logs

```powershell
python tools/diagnose_performance_validation.py outputs/performance-validation-box-through-hole outputs/performance-validation-l-bracket-gusset
```

The diagnostic reads structured stage status and reports only. It includes the
active COLMAP command, completed stages, image/point counts, primitive counts,
recognition reason, hole dimensions, accuracy and STEP availability. It reads
**no logs by default**. Only when needed, explicitly request a bounded excerpt
with `--log-tail 10` (maximum 30 lines).
Use `--brief` for short progress snapshots during long GPU stages.
`--inspect-sparse` analyzes a finished sparse model and its database without
reading logs; `--evidence` includes compact per-surface fitting evidence.

To iterate on CAD fitting without repeating the expensive image reconstruction:

```powershell
python tools/refit_performance_validation.py outputs/performance-validation-l-bracket-gusset
```

This uses saved input snapshots and metric cloud metadata, preserves the initial
reports, and updates the latest fitting diagnostics. Previous STEP candidates
are archived before each replay so a failed replay cannot expose a stale STEP
as its result. A valid exported B-Rep can still fail source-model accuracy checks.

The validator saves its current stage before each expensive/native call. Refit
failures preserve the last completed data and the native exit code, and cannot
reuse stale fitting reports. The source-model boolean comparison uses a recorded
0.00001 mm tolerance to handle nearly coincident faces; the acceptance tolerance
is still 0.5 mm and 5% symmetric difference.

For inexpensive validator/kernel stability checks on mathematically sampled
clouds (these are **not** repeated photo reconstruction runs):

```powershell
python tools/check_cad_validation_stability.py --runs 10
python tools/check_cad_validation_stability.py --exact --runs 10 --output outputs/cad-validator-exact.json
python tools/check_cad_validation_stability.py --exact --seed-sweep --seed 0 --runs 30 --output outputs/cad-validator-exact-seeds.json
python tools/check_cad_validation_stability.py --case outputs/performance-validation-box-through-hole --runs 10 --output outputs/cad-validator-photo-repeat.json
python tools/check_cad_validation_stability.py --case outputs/performance-validation-box-through-hole --seed-sweep --seed 0 --runs 20 --output outputs/cad-validator-photo-seeds.json
```

The default sample adds seeded 0.005 mm position noise and 0.0001 normal-component
noise. Before the stability fix, the exact/noiseless baseline crashed in 12/20
runs (`outputs/stability-before-exact.json`, native exit `3221225477`). A bounded
native stack traced this to CGAL cylinder connectivity: perpendicular box-edge
normals generated a zero-radius cylinder. The project now rejects degenerate
candidates and uses spatial cylinder connectivity, without jittering observations
or modifying SDK headers. The guard alone passed 20/20 before deterministic
sampling was introduced (`outputs/stability-guard-only.json`).

Each detection now owns a seeded stream; full-support cylinder axis/centre/radius
refinement reduces parameter sensitivity. Exact clouds passed 20/20 repeated
runs and seeds 0–29 passed 30/30 with identical geometry. Seeded-noise clouds
passed 10/10 with identical geometry. The exact regression is part of the default
test suite again. Reports are `outputs/stability-exact-repeat.json`,
`outputs/stability-exact-seeds.json` and `outputs/stability-noisy-repeat.json`.

The saved photo box/hole cloud passed 10/10 repeated CAD fits with identical
geometry (`outputs/stability-photo-repeat.json`); its diameter error decreased
from 0.254 mm to 0.059 mm. These replay the existing metric cloud, not image
capture/COLMAP. Photo-cloud seeds 0–19 also passed 20/20 after merging duplicate
cylinder proposals that explain the same source observations
(`outputs/stability-photo-seeds.json`). Across those different seeds, diameter
spread was 0.00344 mm and transverse centre coordinate spreads were 0.02305 mm
and 0.03788 mm. Different seeds need not produce identical noisy-cloud fits;
fixed-seed reruns do. The script reports geometry fingerprints, parameter ranges,
source-model accuracy and failed-run diagnostics, and preserves per-run JSON
reports in a new directory. Fixed-seed checks fail on parameter differences;
`--seed-sweep` checks each seed's accuracy and reports its parameter spread.
All runs must export a valid solid and pass source-model accuracy checks.

On Windows a native crash writes up to 16 symbolized frames to
`fitting_report.json.native-crash.json`; the normal diagnostic shows at most six
frames. A new validator run removes stale crash metadata. Logs remain opt-in.

Studio and the validator were rebuilt. The geometric core test, all four Studio
CTest cases and 21 Python tests passed; the Python suite includes 14 exact and
14 noisy compound cases plus regressions for exact box/hole clouds and duplicate
hole evidence on the saved photo cloud. Both saved standard cases were refitted
and their current STEP exports pass accuracy and STEP round-trip checks.

`workspace/progress.json` is updated before/after each COLMAP command;
`pipeline_status.json` records the overall stage; `fitting_report.json` preserves
compact per-surface evidence without point-ID lists. `validation_report.json`
distinguishes a completed execution from `fitting.complete_cad_fit`: the latter
requires all ground-truth checks to pass. Unsupported geometry is reported
as unrecognized or inaccurate, never as a successful CAD reconstruction.

Outputs contain input snapshots, PNGs, metadata, clouds, full logs and, when
recognized, `reconstructed.step`. Source STEP models and dimensions stay here;
generated runtime data stays under `outputs/`.

The five-type joint primitive counts are diagnostic proposals. The production
box/through-hole path detects planes and cylinders independently, so a joint
diagnostic cylinder count of zero can coexist with an accepted semantic hole.
Check `through_holes` and the ground-truth `accuracy` fields for CAD acceptance.

## Initial reference validation: 2026-10-04

| Case | Dense points | Semantic CAD result |
| --- | --- | --- |
| box-through-hole | 675,791 | Passed: one box + one hole; maximum dimension error 0.031251 mm, hole diameter 11.784618 mm, symmetric difference 0.870324%; valid STEP exported and read back |
| l-bracket-gusset | 544,302 | Failed: initial fitting found no supported primitive; a replay misrecognized an approximately 80 × 8 × 8 mm bar, with 93.980% symmetric difference |

Each case captured 62 images at 1920 × 1080 with known metric poses. This is one
synthetic run per case, not a measured repeated-run success rate or a real-photo
benchmark. Runs overlapped on the GPU, so elapsed times do not measure isolated
GPU performance. One L fitting replay used the same saved cloud; it did not
repeat image reconstruction. The initial L case also exposed low-confidence
cylinder/cone false proposals on an all-planar source; see its README and
`--evidence` diagnostics. Initial reports remain available beside the latest
reports. Rejected STEP candidates were archived before the compound-fitter update.



## Compound fitting update

The saved L-bracket cloud now passes the shared Studio/validator production
entry point: **two boxes plus one triangular prism**, fused into one valid solid.
The maximum world-axis dimension error is 0.041867 mm; boolean symmetric
volume difference is 1.158129%, independently checked by volume sampling with a
conservative upper bound of 1.908613%. The current L `reconstructed.step` is the
accepted reconstruction. Older failed candidates/reports remain archived.

The fitter preserves outward input normals, uses a common orthogonal frame and
observed empty cells, and minimizes the number of admissible covering boxes.
`compound_recognition` reports box/prism counts, surface agreement, occupancy
error, search nodes and whether minimum box count was proven for that grid.
Curved-feature fitting still uses the specialized box/hole path, with a complete
boundary agreement check. Partial scans or unrelated axis frames remain unsupported.

Direct analytic point clouds provide inexpensive regression independent of
capture and COLMAP:

```powershell
python tools/run_compound_fit_tests.py --noise 0
python tools/run_compound_fit_tests.py --noise 0.02 --output outputs/compound-fit-tests-noisy
python -m unittest discover -s tools/tests -p test_compound_fit.py -v
```

Fourteen shapes cover single/merged boxes, L and triangular gussets, T/U brackets,
three/four steps, overlaps, separated solids, overall rotations and shuffled point
order. The sampler removes internal surfaces from the analytic source union;
source STEP, volume and centroid are constructed independently of recognition.
The report checks minimum count, geometry, solid count and STEP readback. Generated
clouds and accepted CAD are available under `outputs/compound-fit-tests*/<case>/`.

Boolean difference volumes can be numerically invalid around nearly coincident
faces. The validator additionally classifies 32,768 seeded uniform volume samples
in both CAD shapes. Invalid or inconsistent booleans use the recorded 3-sigma
Wilson upper bound instead of a negative or whole-body boolean result; acceptance
also requires this independent upper bound to meet the 5% threshold. The report
explicitly states the method and raw boolean result. This does not widen the
0.5 mm dimension tolerance or 5% volume tolerance.
