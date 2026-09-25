# Fountain-P11 reconstruction results

COLMAP 4.2.0 reconstructed the calibrated Fountain-P11 dataset from 11 real-world
images (3072 x 2048). The supplied intrinsics and world-to-camera poses came from
the dataset's optimized bundle-adjustment solution.

| Output | Points | Size | Description |
| --- | ---: | ---: | --- |
| `fountain_p11_sparse.ply` | 13,668 | 205,209 bytes | Feature-matched sparse triangulation |
| `fountain_p11_dense.ply` | 1,632,443 | 44,076,209 bytes | Geometrically consistent PatchMatch stereo fusion |

Supporting workspaces:

- `fountain_p11_sparse_workspace_v2/`: database, logs, and triangulated sparse model
- `fountain_p11_dense_workspace/`: undistorted model, depth maps, normal maps, and fusion manifest
- `fountain_p11_sparse_workspace/`: incomplete first attempt; it included a non-input support
  image and is retained only because local deletion was denied by the execution policy

The point clouds use the right-handed world coordinate frame and scale inherited
from the supplied calibration. The source calibration does not declare a physical
unit, so the unit remains `unknown`.

Reproduce the sparse result from the repository root:

```powershell
python .\core\reconstruction\reconstruct.py `
  --images .\data\fountain-p11\images `
  --camera-info .\data\fountain-p11\camera_info.json `
  --input-unit unknown `
  --gpu `
  --workspace .\outputs\fountain_p11_sparse_workspace_v2 `
  --output .\outputs\fountain_p11_sparse.ply
```

The dense output was produced by running COLMAP `image_undistorter`,
`patch_match_stereo --PatchMatchStereo.geom_consistency 1`, and then
`stereo_fusion --input_type geometric` from that sparse model.

```powershell
$colmap = '.\core\reconstruction\install\colmap\bin\colmap.exe'

& $colmap image_undistorter `
  --image_path .\data\fountain-p11\images `
  --input_path .\outputs\fountain_p11_sparse_workspace_v2\sparse\triangulated `
  --output_path .\outputs\fountain_p11_dense_workspace `
  --output_type COLMAP

& $colmap patch_match_stereo `
  --workspace_path .\outputs\fountain_p11_dense_workspace `
  --workspace_format COLMAP `
  --PatchMatchStereo.geom_consistency 1

& $colmap stereo_fusion `
  --workspace_path .\outputs\fountain_p11_dense_workspace `
  --workspace_format COLMAP `
  --input_type geometric `
  --output_path .\outputs\fountain_p11_dense.ply
```
