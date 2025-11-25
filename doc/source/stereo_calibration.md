# Stereo Calibration

Stereo calibration estimates the [relative position](#stereo-pose) between two cameras. The intrinsics of the involved cameras can either be calculated jointly in the same procedure or calibrated individually beforehand. Note that it is usually assumed that each camera and their relative pose, i.e. the camera rig is fixed. Changing either camera, lenses or camera position inside the rig requires recalibration.

## Workflow

This description will focus on the differences to the [intrinsics calibration](camera_calibration.md). Since stereo calibration also allows to jointly calibrate the intrinsics many of the same principles apply here.

* Choose a [calibration target](calibration_targets.md)
* Capture approximately 20 or more images of your target
  * Please refer to [workflow of camera calibration](camera_calibration.md#workflow), the same principles apply
  * The main difference is that two cameras participate in stereo:
    * For each pose of the calibration target both cameras need to capture a synchronized image. If your cameras are not triggered synchronously you can setup a static scene of camera rig and target, capture an image and then set up a new pose of the target (or equivalently the rig)
    * The target must be visible in each image. Some [targets](calibration_targets.md) require that they are fully visible in each frame (checkerboard) others can also be detected from partial views
* Select `File->Stereo Calibrate from Files...`
  * Enter the directory of your image files and the desired camera model for the first and second camera
  * Check the `Only estimate relative pose` box if you want to do so. In that case you must provide intrinsic parameters for each camera. You can import these from an existing calibration report using the `Import..` button
* Select and parameterize the [calibration target](calibration_targets.md) you took images of
* The main view shows the result of the calibration
  * Check the per view statistics. There might be outliers with very high error often caused by a miss-detection, motion blur or other problems. It may be sensible to remove these images and rerun the calibration
* Select `File->Save Calibration...` to export the calibration report
  * For stereo calibration two files will be generated: `calibration.yaml/.txt` and `calibration_stereo.yaml/.txt`. The first one refers to the first camera, the second (with the '_stereo' suffix) refers to the second camera. For the definition of the pose please refer to [stereo pose](#stereo-pose)

## Stereo Pose

The main output of the stereo calibration is the stereo pose. The stereo calibration report calls this pose 'camera to world'. This is because the first camera pose is defined to coincide with the world, so the second cameras 'camera to world' is effectively a 'camera_two to camera_one' transformation. This pose is represented by a 3x4 transformation matrix:
```{math}
\textbf{T} = \textbf{(R|t)}=\left ( \begin{array} {ccc|c}
                r_{x1} & r_{y1} & r_{z1} & t_x \\\
                r_{x2} & r_{y2} & r_{z2} & t_y \\\
                r_{x3} & r_{y3} & r_{z3} & t_z
  \end{array} \right )
```
with $\textbf{R}$ being a rotation matrix and $\textbf{t}$ the translation vector. In case of the 'camera to world' pose this transforms a 3D point in homogeneous coordinates from the camera coordinate system to the world coordinate system:
```{math}
\textbf{X}_{world} = \textbf{TX}_{camera}
```
Conveniently this 'camera to world' pose also has a geometrical interpretation: The vector $\textbf{t} = (t_x, t_y, t_z)^T$ describes the position of the camera in world coordinates. The column vectors of the rotation matrix represent the axes of the cameras coordinate system in world coordinates. The stereo `calibration.yaml` holds $\textbf{R},\textbf{t}$ as follows:
```yaml
cam_to_world_rotation_rowmajor: [1, 0, 0, 0, 1, 0, 0, 0, 1]
cam_to_world_translation: [0, 0, 0]
```

