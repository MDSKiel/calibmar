# Should I Calibrate My Housing?

When dealing with (geometric) refractive distortion in underwater images there are a few possible approaches: One is to interpret the refractive distortion as a type of lens distortion, where the housing is ignored and the idea is that all effects can be 'absorbed' into the [camera intrinsics](camera_calibration.md#camera-models). Another is to explicitly [model the housing](housing_calibration.md#housing-models) and calibrate it separately from the camera. Both are possible using this tool. The problem with treating refraction as a type of lens distortion is that it is generally less well behaved. For the vast majority of lenses the camera retains a (mostly) single projection center, the standard pinhole with distortion model works and images can be 'undistorted'. This is not true for refraction, where the rays no longer intersect at a single point but rather along an axis, making it an axial model [[6](#reference_6), [7](#reference_7), [8](#reference_8)]. This effectively means that the distortion experienced on the image plane depends on the distance of the scene to the camera. An obvious answer would be to always calibrate a housing, explicitly model refraction and avoid any systematic errors in downstream vision tasks.

## The problem with applying a housing calibration

Many computer vision algorithms rely on projective geometry in one way or another. Concepts like essential matrix, fundamental matrix or homographies explicitly rely on pinhole projection. Pinhole projection is computationally quite efficient and can be easily vectorized. Under multi layer refraction back projection means ray casting and projection requires solving a higher order polynomial [[6](#reference_6), [7](reference_7)]. For many algorithms this is computationally prohibitive. Another problem is that refractive vision remains a relatively niche application so many algorithms and software tools which exist for standard in-air do not exist this domain even if it were theoretically possible. One example of an existing solution is [refractive SfM](https://github.com/MDSKiel/colmap_underwater) which implements adaptations for refraction on several steps in the pipeline.

## When do I need to calibrate my housing?

**TLDR**: If the distance of the camera projection center to the flat port interface is small and the observed objects are all at the same distance, or a reasonably well centered dome port is used, absorbing refractive distortion into the intrinsics may work. Housing calibration can also be very useful to mechanically align a camera when building or assembling a camera housing system.

The following section will explore the cases where compensating refraction with lens distortion may not work as well and housing calibration (and adaption of downstream algorithms) could be sensible. The Experiments will ignore the thickness of the port and assume no tilt for the flat port. 

### Distance Dependant Distortion

Refraction will cause the rays of light to no longer intersect at a common center of projection, which will cause a distance dependant distortion. The following figure visualizes the distortion refraction causes (angles of refraction are exaggerated for visualization):

```{figure} ../assets/flatport_error_diagram.svg
:width: 40%
:align: center
Refractive distortion of pixel position $\mathrm{x_0}$ to $\mathrm{x_1}$ and $\mathrm{x_2}$ depending on scene distance (exaggerated refraction)
```

3D points $\mathrm{X_1}$, $\mathrm{X_2}$ at distance $d_1$, $d_2$ would be projected to $\mathrm{x_0}$ in air. With refraction they are projected to $\mathrm{x_1}$, $\mathrm{x_2}$. The distortion in image space for image position $x_0$ is $(\mathrm{x_0}-\mathrm{x_1})$ at $d_1$ and $(\mathrm{x_0}-\mathrm{x_2})$ at $d_2$. Absorbing the refractive distortion into the camera intrinsics by calibrating at distance $d_1$ would cause an error for points at $d_2$ and vice versa. To visualize this error the following figures show a numeric simulation: A large number of uniformly sampled pixel positions are refractively back-projected to a certain distance and then projected back into the image using a lens distortion model. The reprojection error is then minimized using a non-linear least squares optimization of the camera intrinsics (this method is used in [refractive SfM](https://doi.org/10.1109/IROS58592.2024.10802043) as a part of relative pose estimation). This is effectively simulating using the lens distortion model to absorb the refractive distortion by calibrating at the given distance. The quiver plots show the observed error on the plane in space $\vec{e} = \mathrm{X-X_{lens}}$ with $\mathrm{X}, \mathrm{X_{lens}}$ being the correct refractive and calibrated lens back projection at the given plane distance.

```{figure} ../assets/quiver_distance.svg
:width: 100%
:align: center
```

The above figure shows the metric error at one, three and five meters distance when using a fisheye model to absorb refractive distortion. At the calibrated distance the error is small, especially in the central parts of the image. For scenes at closer and further distances the error becomes quite significant. For a 3D reconstruction this type of error might introduce a warping of the recovered point cloud, measuring applications will overestimate sizes in the distance and underestimate close ones. In practice this means, if the working distance is known and does not vary much this method can work. Everything observed at other distances will include a systematic error.

### Camera To Flat Port Distance

Another important factor for a flat port system is the distance of the center of projection to the port interface. The center of projection for a real camera is usually at the entrance pupil somewhere close to the front of the lens. So the distance of that point to the port pane can normally be kept small. But as the following figure shows, a few millimeters can make a difference. The greater this distance is, further the system will deviate from a local center of projection. The camera system becomes less 'central' and the lens distortion model approximation will perform worse.

```{figure} ../assets/quiver_flatport_distance.svg
:width: 100%
:align: center
```

This figure shows the error at the calibrated distance, i.e. the best case scenario. Any deviation in scene distance will still increase the error (as shown in the corresponding figure). The closer the center of projection is to the port the closer the camera system is to a central pinhole camera and the approximation is better.

### Index of Refraction

An issue when using intrinsics to absorb refraction is that the properties of the system become part of the calibration. While most parameters can be reasonably considered constant, the index of refraction of water changes with salinity, temperature and pressure. When explicitly modelling a housing, the index of refraction remains an adaptable parameter, but when using the distortion model approximation all these parameters get entangled into the camera intrinsics. The following figure shows the error that occurs when a camera is calibrated at an index of refraction of 1.333 but then used under deviating conditions.

```{figure} ../assets/quiver_refractive_index.svg
:width: 100%
:align: center
```

This figure basically simulates calibrating the camera system in freshwater but then using it in seawater. The refractive indices of water can vary from roughly 1.3 up to 1.35 depending on pressure, salinity and temperature [[9](reference_9)]. This may seem like a small difference but can have a noticeable effect as shown above. As with the port camera distance, the more the index of refraction differs from air (~ 1), the stronger the angle of refraction and the less central the system becomes.

### Dome Port Decentering

A dome port is ideal to avoid geometric refraction all together. If the dome center coincides with the camera projection center all the rays intersect the port perpendicularly and do not get refracted at all. The alignment of a camera inside a dome in x-y (horizontal, vertical) direction is usually quite good, because the location of the sensor and therefore its optical axis is known and the housing can be designed accordingly. The z alignment of the projection center is much harder because it depends on the lens and the center of the dome sphere in space. This figure simulates decentering in z direction.

```{figure} ../assets/quiver_dome_decentering.svg
:width: 100%
:align: center
```

The error at a small decentering distance is minor, the system experiences minimal refraction. When the centers do not coincide a similar effect as to the flat port distance can be observed. The stronger the decentering the less central the resulting system becomes and the worse the lens distortion approximation becomes. Again this figure shows the error at the calibrated distance, i.e. the best case scenario.
