# Calibration Report

On saving a calibration two files are saved. A `calibration.txt` and a `calibration.yaml`. The `calibration.txt` is considered the 'human readable' file format and basically mirrors what is shown in the calibration result screen. The `calibration.yaml` is still quite readable but easy to parse programmatically and can also be imported by the tool. 

An exemplary `calibration.yaml` looks like this:
```yaml
model: OPENCV
# fx, fy, cx, cy, k1, k2, p1, p2
parameters: [705.391983, 705.210233, 963.440447, 604.968632, -0.105541, 0.066014, 0.001093, -8.6e-05]
non_svp_model: FLATPORT
# Nx, Ny, Nz, int_dist, int_thick, na, ng, nw (Note that [Nx, Ny, Nz] must be unit vector)
non_svp_parameters: [0.01311427226, -0.004460945556, 0.9999040533, 0.02531637365, 0.01, 1, 1.49, 1.333]
# non_svp_est_std_dev: 0.001294740237, 0.001435687642, 1.84569193e-05, 0.0005963799373, 0, 0, 0, 0
width: 1920
height: 1200
# overall_rms: 0.8004862862
# target: chessboard, 7, 8, 0.040000
```
The parameters it contains are:
| Parameters | Description |
| --- | --- |
| model | The [camera model](camera_calibration.md#camera-models) used during calibration |
| parameters | The respective model [parameters](camera_calibration.md#camera-models) estimated for that camera model |
| non_svp_model | The **N**on-**S**ingle **V**iewPoint ([housing](housing_calibration.md#housing-models)) model used during calibration |
| non_svp_model_parameters | The respective model [parameters](housing_calibration.md#housing-models) estimated for that housing model |
| width & height | Image width and height |

For a stereo calibration this report also contains the [stereo pose](stereo_calibration.md#stereo-pose):
```yaml
cam_to_world_rotation_rowmajor: [1, 0, 0, 0, 1, 0, 0, 0, 1]
cam_to_world_translation: [0, 0, 0]
```