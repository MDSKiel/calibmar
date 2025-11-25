# Calibration Targets

The calibration target defines the world coordinate system during calibration. A few different calibration targets are currently supported. Any metric units such as checkerboard square size or marker size are expected in meters. All targets are considered rigid, the boards are considered planar. Any deviation of that will introduce error.

## Checkerboard

The checkerboard allows detection of keypoints at the intersections of the squares. It is a classic because the gradients at these intersections allow precise localization of the corners. For detection it needs a small border around it, a uniform illumination and it must be completely visible. It is parameterized by its number of columns, rows and square size.

## Marker (ArUco/April) Grid Board

The main advantage of the marker grid board is the ability to also use partials views of the board. The markers allow localizing the individual edges across the board. This makes it quite flexible, but also requires a few more parameters to completely define the coordinate system.

| Parameter | Description |
| --- | --- |
| Marker Type | The type of marker used. This can be ArUco (`4x4`,`5x5` etc.) or April tag (`APRILTAG_16h5`).
| Marker Border Bits | The size of the marker border in bits (a 'square' inside the maker is one bit). |
| Columns & Rows | Number of marker columns and rows |
| Board Origin | Where on the board is the first marker (e.g. `Top Left`, `Top Right` etc.).
| Marker IDs Ascending | Are the markers arranged ascending in `Horizontal` or `Vertical` order. 
| Marker Size | Size of the markers without the border |
| Marker Spacing | Size of the space between each marker | 

```{figure} ../assets/marker_board.svg
:width: 100%
:align: center
An ArUco (`4x4` marker type) Grid Board, with three rows, two columns, bottom left board origin and horizontally ascending IDs starting with zero.
```

## Auto Calibration Target (3D Target)

This target allows calibrating from SIFT features. It is extremely flexible, the only requirement being that images display a rigid scene (or target) with detectable SIFT features. If marker detection is enabled, the marker will be used in conjunction with the 'mask factor' to only use features close to the marker and mask out everything else. This enables the use of a target which can be moved in front of the camera without disturbing the calibration with features which do not belong to the target (i.e. a moving person or background). If a texture is used which produces a large amount of densely spaced SIFT features (e.g. [this one](https://doi.org/10.1109/IROS.2013.6696517)) a few images can generate a very large amount of constraints over the whole image space allowing a fast and precise calibration. The downside of SIFT features is that they are potentially less precisely localizable as e.g. checkerboard corners. If the `Enable Aruco Detection` checkbox is disabled this mode is essentially SfM auto calibration based on the [COLMAP](https://github.com/colmap/colmap)/[Refractive COLMAP](https://github.com/MDSKiel/colmap_underwater) implementation. 

| Parameter | Description |
| --- | --- |
| Enable Aruco Detection | Switches between masking and unmasked auto calibration (see above). |
| Aruco Marker Type | The type of marker to be used for masking. |
| Aruco Mask Factor | The bounds of the marker will be scaled by this factor to generate the mask. Should be set such that as much as possible of the target is used while the background is completely masked out. |

```{figure} ../assets/cube_diagram.png
:width: 100%
:align: center
Exemplary 'Auto Calibration Target' in cube shape, with masking markers using a dedicated [SIFT texture](https://doi.org/10.1109/IROS.2013.6696517).
```

## FAQ

**Which calibration target should I use and which parameters are ideal?**

Use one which is most practical for your application. A checkerboard is simple to generate and use, but requires to be fully in frame for detection. A marker board also works with partial views, which makes it easier to use for stereo calibration, but detections can sometimes be less robust. The more feature points a target displays the more constraints are generated per image, which favors smaller square sizes. At the same time smaller squares can lead to worse detections if the board is not perfectly in focus or too far away. Large boards cover more of the image but are harder to handle. There are many tradeoffs: Choose a target which allows you to take high quality images with feature points covering the image space over the whole data set.

**Do I have to use meters as my target unit?**

Technically the unit used does not matter, as it will simply set the scale of your target coordinate system. All derived measures, such as stereo pose or housing parameters will be in this unit scale. If there are no derived metric measures, i.e. the intrinsic calibration which is completely up to scale, the unit is mostly irrelevant and the only requirement is consistency (e.g. marker grid board size and spacing must be in the same scale).

