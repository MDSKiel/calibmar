# Housing Calibration

Housing calibration estimates the geometric configuration of camera and housing. This tool supports dome and flat port style housings. For details on the housing parameters please refer to [housing models](#housing-models). Housing calibration requires knowledge of the camera intrinsics (i.e. a previous [camera calibration](camera_calibration.md)). Further information can be found in the [FAQ](#faq) and in '[Should I Calibrate My Housing?](should_i_calibrate_my_housing.md)'.

## Workflow

* [Calibrate your camera](camera_calibration.md) in air and save the calibration.
* Choose a [calibration target](calibration_targets.md)
* Capture approximately 20 or more images of your target in water
  * Please refer to [workflow of camera calibration](camera_calibration.md#workflow), the same principles apply
* Select `File->Calibrate from Files...`
  * Enter the directory of your image files
  * Use the `Import..` button to import the previous in-air intrinsics calibration
    * Otherwise manually select the camera model and enter the intrinsics as 'initial parameters'
  * Select the appropriate Housing Interface
  * Enter the initial [housing parameters](#housing-models)
  * Select and parameterize the [calibration target](calibration_targets.md) you took images of
  * Press 'Run'
* The main view shows the result of the calibration
  * Check the per view statistics. There might be outliers with very high error often caused by a miss-detection, motion blur or other problems. It may be sensible to remove these images and rerun the calibration
* Select `File->Save Calibration...` to export the calibration report
  * The exported `.yaml` can be imported in the calibration dialog to copy the parametrization and speed up doing a rerun

## Housing Models

The parameters for the dome and flat port housing are $\{c_x, c_y, c_z,r_\mathrm{dome}, t_\mathrm{glass}, \mu_a, \mu_g, \mu_w\}$ and $\{n_x, n_y, n_z,r_\mathrm{flat}, t_\mathrm{glass}, \mu_a, \mu_g, \mu_w\}$ respectively. Their meaning is described in the follow table, where they appear as written in the dialogs. The offset vectors, distances and thicknesses are expected to be in meters (though they technically only need to be in the same unit as the scene, as defined by the calibration target). 

| Parameter | Description |
|  --- |    ---      |
| `Cx, Cy, Cz`  | The center of the dome $(c_x, c_y, c_z)^T$ in the camera coordinate system. Equivalently this is also the offset vector $\vec{c}$ from the camera to the dome center. Would usually be initialized with `(0,0,0)`. |
| `int_radius` | Interface radius $r_\mathrm{dome}$. The radius of the dome. | 
| `int_thick` | Interface thickness $t_\mathrm{glass}$. The thickness of the flat or dome port material. |
| `Nx, Ny, Nz` | The interface normal $(n_x, n_y, n_z)^T$. Would usually be initialized with `(0,0,1)`. Must be normalized to unit length. |
| `int_dist` | Interface distance $r_\mathrm{flat}$. The distance of the projection center of the camera to the flat port in direction of the normal.
| `na, ng, nw` | Refractive indices $\mu_a, \mu_g, \mu_w$ of air, port material (glass) and water.

All of these parameters are used to initialize the calibration. The optimized parameters are `Cx, Cy, Cz` in case of the dome port and `Nx, Ny, Nz, int_dist` for a flat port. All other parameters are considered constant as provided.

```{figure} ../assets/dome_flat_diagram.svg
:width: 100%
:align: center
Diagrams of flat and dome port with decentering and resulting ray refraction from [A Calibration Tool for Refractive Underwater Vision](https://openaccess.thecvf.com/content/ICCV2025W/CVAUI%20&%20AAMVEM/papers/Seegraber_A_Calibration_Tool_for_Refractive_Underwater_Vision_ICCVW_2025_paper.pdf)
```

## FAQ

**I've calibrated my housing, what do I do with these parameters?**

Please refer to '[Should I Calibrate My Housing?](should_i_calibrate_my_housing.md)'.

**Can the camera stay inside the housing for the intrinsics calibration in air?**

Strictly speaking the camera should be taking out of the housing. Practically this is often not possible or very cumbersome. The effect caused by the port in air is usually extremely small and will likely not make a big difference in the intrinsics calibration.

**What if the refractive index (temperature/salinity/pressure) of water changes between calibration and application?**

In general the output of the calibration is the geometric configuration of camera and housing. Downstream algorithms can use this geometric information (together with the refractive indices) to correct for refraction. As long as these algorithms are provided the correct refractive index during application this is still possible. For more information on the effect of the refractive index also refer to '[Should I Calibrate My Housing?](should_i_calibrate_my_housing.md)'.

**How do I determine the distance of camera center to the flat port for initialization?**

The optical center of a real camera is usually located somewhere close to the front of the lens at the entrance pupil. The distance of the lens to the port is typically sufficient as an initial guess for the calibration.