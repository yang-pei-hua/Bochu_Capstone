# Geometric Reconstruction

This module is the Qt/VTK-free boundary between observed point clouds and the
parametric modeling core. It keeps three kinds of state separate:

1. immutable source observations in `PointStore`;
2. fitted `SurfaceEvidence` that references source `PointId` values;
3. accepted semantic primitives committed to `ModelingCore` through an atomic
   `ModelPatch`.

The baseline vertical slice recognizes one complete, axis-aligned,
millimeter-scale box. The general path uses CGAL Efficient RANSAC,
covariance-based plane refinement, and parallel/perpendicular relation analysis
to recover a fully oriented box from noisy observations with outliers.

Primitive proposal has a project-owned boundary backed directly by CGAL 6.2.1.
CGAL detects planes, cylinders, spheres, cones, and tori in one Efficient RANSAC
run; its reordered support indices are mapped back to stable `PointId` values.
Project-side code normalizes the analytic parameters, refines planes with PCA,
refines cylinder axes from support normals and centres with a transverse circle
fit, refines cylinder and sphere radii over all support points, suppresses duplicate
depth layers, and produces typed evidence for all five primitives. No CGAL type
crosses the public API and there is no native RANSAC fallback.

Each detection uses an isolated random stream (`PrimitiveDetectionOptions::randomSeed`,
default 4500) and restores the caller's CGAL random state. Plane and cylinder
recognition wrappers forward their respective seeds. Cylinder refinement sums
support in source `PointId` order and reassesses full-cloud support after the
first fit. Thus identical input/options reproduce parameters independently of
earlier detections; changing the seed remains available for robustness probes.
Cylinder proposals with matching axes/radii and at least 70% overlap of the
smaller source support are collapsed to the strongest supported proposal. This
prevents one noisy hole wall from becoming several semantic holes, while nearby
real cylinders with distinct source observations remain separate.

The project-owned CGAL cylinder subclass rejects zero-radius/non-finite candidates
and uses CGAL spatial connectivity instead of the angular bitmap. This prevents
an exact box edge's perpendicular normals from producing a degenerate cylinder
whose angular parameterization divides by zero and crashes during connectivity.
The fix changes neither source observations nor installed CGAL headers.

Both paths validate the same contract:

```text
synthetic surface points
  -> six PlaneEvidence records
  -> BoxCandidate
  -> BoxPrimitive model state
  -> OpenCASCADE B-Rep
  -> STEP
```

The client reconstruction path also recognizes cylindrical through holes in a
recognized box. Cylinder evidence must be aligned with a box axis, have enough
angular coverage, remain inside both transverse box bounds, and span between
both opposing exterior faces. Accepted evidence becomes a semantic
`ThroughHolePrimitiveParams` feature and is committed atomically with the box;
short cylinders are deliberately left unclassified rather than mislabeled as
through holes.

`CurvedSolidRecognizer` completes independent spheres, capped cylinders, cones,
conical frusta and non-self-intersecting ring tori. The shared entry point keeps
the compound and box/hole paths, then tries curved completion. Each curved
family is proposed independently, avoiding consumption of valid observations
by a competing nearly degenerate cone/torus. Complete-solid scoring chooses
the accepted family. Sphere centre/radius and cone/torus parameters undergo
robust nonlinear least squares in stable source-PointId order.

Cylinder and cone extents come from observed, outward-facing planar cap groups,
not support extrema. A pointed cone also requires observations near its apex.
Acceptance requires at least 90% outward-normal boundary agreement and 90%
angular/cap-bin coverage. Sphere uses 12 azimuth × 6 equal-height bins; torus
uses 24 major × 12 minor angular bins; capped bodies use 24 azimuth × 6 axial
side bins and 8 azimuth × 2 radial bins on each nonzero-radius cap. These tests
reject hemispheres, partial rings and missing end caps; they are a bounded
coverage check rather than proof that every arbitrarily small gap was observed.

Accepted solids retain typed Sphere/Cylinder/Cone/Torus feature parameters and
can be edited through `EditFeatureCommand` in an atomic patch. They are new
independent solids; curved boolean combinations, trimmed patches and automatic
completion of occluded surfaces are outside this first implementation.

`reconstructParametricSolid()` is now the shared Studio/validator entry point.
It first reconstructs a common orthogonal frame from dominant normal families,
refines it from planar position covariance, and builds a plane-boundary grid.
Signed surface observations distinguish material from empty cells. Nearby face
orientations resolve the ambiguity of extrapolating a normal at a concave edge.
Oblique cells are fitted as clipped polygonal prisms, retaining a triangular rib
as an extrusion rather than approximating it with many stair-step boxes.

For the full material cells, all admissible boxes are enumerated and an exact
branch-and-bound set-cover search minimizes **box count**. Boxes may overlap;
their union cannot include an empty or oblique cell. Replacing a contained box
with its admissible maximal box preserves the optimum. Thus two coplanar pieces
forming a rectangle merge into one box, an L shape needs two, and stepped shapes
retain the number required by their observed empty regions. The number means
geometric constituents, not connected solids: a fused L bracket is one CAD solid.
`minimumBoxCountProven` applies to this detected grid, not every possible CAD
representation; a search limit is reported explicitly rather than claimed optimal.

A boundary agreement check prevents incomplete local bars or encompassing boxes
from being accepted as a complete model. The curved-feature fallback uses the
existing box/through-hole recognizer and also checks complete surface agreement.
The commit adapter creates joined BoxPrimitive features and a sketch/extrusion
for each prism in one atomic patch, including correct IDs in nonempty documents.
`AxisAlignedBoxRecognizer` and `reconstructBoxWithThroughHoles()` remain available
as specialized lower-level APIs. None depends on Qt, VTK, PCL or client state.

The compound path supports closed, millimetre-scale surfaces with outward normals
and a shared orthogonal frame, optionally with a few planar prism faces. Complete
independent curved primitives may have any axis direction. Normal
orientation from the input is preserved during preprocessing. Missing-normal
PCA estimates alone do not establish a consistent outward orientation. A
deterministic multi-source shortest-path forest propagates signs from existing
signed observations through compatible tangent neighborhoods. Input signs are
preserved; directions without a signed root retain `normalOrientationKnown=false`.
`reconstructParametricSolid()` rejects unresolved orientation before material
occupancy is evaluated. Thin walls, sharp edges and gaps can deliberately leave
directions unresolved rather than fabricate an outward sign. Partial
visibility, independently oriented box components, curved multi-body parts and
fine features below the sampling/tolerance resolution remain outside this path.

Before detection, `preprocessPointCloud()` provides the scale-aware input path:

- known metre input is converted to the modeling core's millimetres;
- voxel downsampling retains an original representative `PointId` per voxel;
- statistical K-nearest-neighbor filtering removes isolated observations, with
  a median/MAD ceiling preventing distant outliers from masking closer ones;
- PCA neighborhoods estimate missing normals and local curvature; rank-one and
  coincident neighborhoods do not manufacture surface normals;
- orientation gates and robust filtering ratios are explicit, dimensionless
  preprocessing options; reports distinguish estimated, oriented and unresolved
  normals and record the median distance and applied outlier threshold.

The client preserves PLY `nx/ny/nz` attributes and reads the unit from the
adjacent reconstruction manifest. A cloud without metric metadata requires an
explicit user confirmation before its source units are interpreted as
millimetres.

The client derives voxel size from a 1%-trimmed coordinate range, so a small
number of distant points cannot erase the part before outlier filtering. This
range only chooses resolution; it does not remove source observations. Protected
manual observations continue to survive both filters.

Independent stress fixtures and explicitly sourced recorded-cloud replay are
available through `tools/run_cloud_reliability.py`. See
[`Cloud_Reliability_Validation.md`](../../docs/Cloud_Reliability_Validation.md)
for commands, accuracy/rejection contracts and the missing real mechanical dataset.
The next steps are partial-visibility handling, acquiring orientation when no
signed observations exist, independently oriented components and richer
curved-feature semantics.

Direct analytic-cloud regression (no capture or COLMAP):

```powershell
python tools/run_compound_fit_tests.py --noise 0
python tools/run_compound_fit_tests.py --noise 0.02 --output outputs/compound-fit-tests-noisy
python -m unittest discover -s tools/tests -p test_compound_fit.py -v
```

The fixture generator removes internal faces using the source union predicate;
ground truth volume/centroid and OCCT reference STEP are independent of fitted
features. Tests check primitive counts, proof of minimum box count, fused solid
count, dimensions, STEP readback, and symmetric difference. Independent solid
classification sampling checks boolean comparison; invalid/inconsistent boolean
volumes use the conservative recorded sampling upper confidence bound.

## Guided reconstruction

`reconstructGuidedSolid()` accepts original-point-ID template groups for planes,
boxes, spheres, cylinders, cones and tori. It fits only the requested family,
uses fixed plane equations/normals and primitive evidence to guide global
recognition, retains complete observed curved components, and reconstructs the
remaining points. Local patches complete only from observed global boundaries;
unsupported caps or thicknesses are never inferred from a label alone. A detached
OCCT model verifies that finite CAD faces explain at least 90% of every group
before the candidate is delivered. Cancellation and conflicting groups produce
no partially committed model.

Preprocessing accepts `protectedPointIds` so every manual observation survives
voxel/outlier filtering while the remaining scene continues to be downsampled
and denoised. All output IDs still refer to original observations.

`commitCompoundCandidate(..., replacePrevious=true)` atomically replaces features
marked as reconstructed, preserving independently authored history and rejecting
removal when authored dependents reference generated features.

## Build and test

From the repository root:

```powershell
cmake -S core/geometric_reconstruction `
  -B core/geometric_reconstruction/build-msvc-cgal `
  -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_TOOLCHAIN_FILE="D:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake" `
  -DOpenCASCADE_DIR="$PWD/deps/occt-8.0.1/cmake"
cmake --build core/geometric_reconstruction/build-msvc-cgal --config Release
ctest --test-dir core/geometric_reconstruction/build-msvc-cgal `
  -C Release --output-on-failure
```

Install `cgal:x64-windows` and `eigen3:x64-windows` in that vcpkg root before
configuration. The test writes `test-output/synthetic_box.step`.

CGAL Shape Detection is GPL-3.0-or-later or commercially licensed. Distribution
of a closed-source build requires the appropriate commercial-license decision.
