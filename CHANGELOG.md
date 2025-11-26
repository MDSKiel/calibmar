# Changelog

## 1.1.0

### New Features

- Add documentation 
- Stereo calibration improvements
  - Support two different camera models
  - Add support for marker grid boards
- Image undistortion dialog
  - Images in a directory can be undistorted given a camera model and parameters
- Interactive housing diagram, allows visualization/exploration of ray geometry for housings in 2D
- Small model explorer improvements

### Bugfixes

- Fix stereo calibrations giving wrong results when cameras had different image sizes
- Skip unreadable files for stereo calibration, which could lead to wrong image pair alignment

## 1.0.0

Initial release