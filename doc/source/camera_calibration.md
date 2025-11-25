# Camera Calibration

Camera calibration estimates the intrinsic properties of a camera lens system. These include focal length and principal point as well as lens distortion parameters. For the exact number of parameters and their application please refer to [camera models](#camera-models). Note that it is usually assumed that the camera lens system is fixed, i.e. changing zoom level, focus or a different lens altogether requires recalibration. When referring to a camera, this documentation always implies a camera lens system unless noted otherwise.

## Workflow

This description will focus on calibrating from image files. Further questions might also be answered in [FAQ](#faq).

* Choose a [calibration target](calibration_targets.md)
* Ensure the camera (or rather lens) is set up exactly as required for your application
* Capture approximately 20 or more images of your target. 
  * Try to vary angle and position of the target in every image. Cover the whole image space (i.e. make sure the calibration target is not only positioned in the center of the image but also positioned at the image borders and corners) and vary the distance to the camera from time to time.
  * Avoiding (motion) blur, shadows, specular highlights and irregular illumination will ensure proper detection of the target and precise localization of the keypoints.
* Select `File->Calibrate from Files...`
  * Enter the directory of your image files
  * Select the desired [camera model](#camera-models)
  * Leave 'None' for the housing
  * Select and parameterize the [calibration target](calibration_targets.md) you took images of.
  * Press 'Run'
* The main view shows the result of the calibration
  * Check the per view statistics. There might be outliers with very high error often caused by a miss-detection, motion blur or other problems. It may be sensible to remove these images and rerun the calibration.
* Select `File->Save Calibration...` to export the calibration report.
  * The exported `.yaml` can be imported in the calibration dialog to copy the parametrization and speed up doing a rerun. 

## Camera Models

For camera intrinsics this tool uses the standard pinhole and distortion model. For a camera without lens distortion projecting 3D point X (in the camera coordinate system) into image position x is given up to scale by:
```{math}
\textbf{x} \sim \textbf{KX}
```
with
```{math}
\textbf{K}=\left ( \begin{matrix} f_x & 0 & c_x \\\
                 0 & f_y & c_y \\\
                 0 & 0 & 1
  \end{matrix} \right )
```
where $ f_x, f_y $ is the focal length and  $(c_x, c_y)$ the principal point.

Lens distortion is modeled by [[1](#reference_1), [2](#reference_2), [3](#reference_3)]:
```{math}
\begin{pmatrix} \text{x}' \\ \text{y}' \end{pmatrix} = \begin{pmatrix} \text x \frac{1 + k_1 r^2 + k_2 r^4 + k_3 r^6}{1 + k_4 r^2 + k_5 r^4 + k_6 r^6} + 2 p_1 \text{xy} + p_2(r^2 + 2 \text{x}^2) \\ \text y \frac{1 + k_1 r^2 + k_2 r^4 + k_3 r^6}{1 + k_4 r^2 + k_5 r^4 + k_6 r^6} + p_1 (r^2 + 2 \text{y}^2) + 2 p_2 \text{xy} \\ \end{pmatrix}
``` 
with $r^2 = \text{x}^2+\text{y}^2$. Here $(\text{x}',\text{y}')$ and $(\text x, \text y) $ are the distorted and ideal normalized image coordinates respectively. 

Similarly, the fisheye lens distortion is modeled by [[4](#reference_4), [5](#reference_5)]:

```{math}
\begin{pmatrix}  \text{x}' \\ \text{y}' \end{pmatrix} = \begin{pmatrix}  (\theta (1 + k_1 \theta^2 + k_2 \theta^4 + k_3 \theta^6 + k_4 \theta^8) / r) \text x  \\   (\theta (1 + k_1 \theta^2 + k_2 \theta^4 + k_3 \theta^6 + k_4 \theta^8) / r) \text y\end{pmatrix}
```
with $\theta = atan(r)$.

This implementation of lens distortion matches the ones in [OpenCV](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html) and [COLMAP](https://colmap.github.io/). The names used for camera model selection denote varying degrees of complexity. We use the same names as COLMAP. The following table shows which parameters are used for a given model. A single focal length given means $f = f_x = f_y$.

| Name | Parameters |
| ---  |    ---      |
| Simple Pinhole | $f, c_x, c_y$  |
| Pinhole | $f_x, f_y, c_x, c_y$ |
| Simple Radial | $f, c_x, c_y, k_1$ |
| Radial | $f, c_x, c_y, k_1, k_2$ |
| OpenCV | $f_x, f_y, c_x, c_y, k_1, k_2, p_1, p_2$ |
| Full OpenCV | $f_x, f_y, c_x, c_y, k_1, k_2, p_1, p_2, k_3, k_4, k_5, k_6$ |
| OpenCV Fisheye* | $f_x, f_y, c_x, c_y, k_1, k_2, k_3, k_4$ |

\*Uses the fisheye distortion formulation.

## FAQ

**Which camera model should I use?**

The simplest model you can get away with. A complex model may overfit on the noise in your data, which is especially the case if there are only few images in your data, they are especially noise or not well distributed over the image space. The `Simple Pinhole` and `Pinhole` do not model any lens distortion and are therefore not well suited for most real world cameras.

**Which calibration target should I use?**

Please refer to [calibration targets](calibration_targets.md).

**How many images should I take for calibration?**

Recommendations usually vary from anything between 10 to 200 images. More important than the exact number is that they are of high quality: The target is well illuminated, not motion blurred, in focus, displays varied poses and covers the whole image space over all images (see [workflow](#workflow)).

**Does it matter whether I move the calibration target or the camera?**

No. From the point of view of the camera this is equivalent.
