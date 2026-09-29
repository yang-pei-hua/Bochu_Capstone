# Fountain-P11 test dataset

This directory contains the Fountain-P11 real-world calibrated multi-view dataset used to verify
the project's COLMAP reconstruction layer.

- Scene: outdoor fountain
- Camera: Canon D60, 20 mm
- Images: 11 PNG files at 3072 x 2048
- Calibration: optimized camera intrinsics and world-to-camera poses from a bundle-adjustment solution
- Original authors: Christoph Strecha et al., EPFL multi-view stereo benchmark
- Mirror: <https://sourceforge.net/projects/graphview/files/example_data/fountain-p11.zip/download>
- Archive SHA-256: `9688e13c637b083b1d7a86c873b89dba8920e9145351f6069f37e0bbb87de566`

The mirror's `Readme.txt` identifies this as a Fountain-P11 bundle-adjustment solution and links to
the original EPFL dataset. The original EPFL dataset page describes the data as available for
research use. Review the upstream terms before redistributing the image files.

Directory layout:

```text
fountain-p11.zip       Verified source archive
images/                The 11 reconstruction input PNG images
raw/                   Extracted GraphView bundle-adjustment and support files
camera_info.json       CameraInfo converted for core/reconstruction
README.md              Provenance and reproduction notes
```

Regenerate `camera_info.json` from the source calibration:

```powershell
python .\core\reconstruction\tools\convert_fountain_p11.py `
  .\data\fountain-p11\raw `
  .\data\fountain-p11\camera_info.json `
  --images-dir .\data\fountain-p11\images
```

The converter reads the optimized world-to-camera translation and SO(3) rotation vector from
`solution.txt`, associates camera vertices with image names through `order.txt`, and converts the
rotation vectors to COLMAP Hamilton quaternions. On the source graph's 171,026 observations, this
interpretation has approximately 0.37 px mean and 0.23 px median reprojection error.
