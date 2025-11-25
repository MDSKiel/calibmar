# Introduction

The goal of calibration in context of cameras is to determine how 3D points are projected into the 2D image. In detail it can refer to a few closely related but separate topics for (underwater) camera systems.

## Camera Calibration

Camera calibration estimates the image formation properties of a camera and lens. The standard way to model the image formation is to use a pinhole camera together with a lens distortion model. The parameters that make up this combined model are called intrinsics. Note that the intrinsics calibration assumes the camera lens system is fixed, i.e. using a different lens or zoom requires recalibration. When referring to a camera, this documentation always implies a camera lens system unless noted otherwise. For more details please refer to [camera calibration](camera_calibration.md).

## Stereo Calibration

Stereo calibration estimates the relative position between two cameras. The intrinsics of the involved cameras can either be estimated jointly in the same procedure or calibrated individually beforehand. For more details please refer to [stereo calibration](stereo_calibration.md).

## Underwater Housing Calibration

Underwater housing calibration estimates the geometric configuration between a camera and its housing. This configuration is relevant because it determines the refractive effects underwater. This tool specifically handles flat port and dome port housings. The distortion caused by refraction can be quite a bit different from the distortion caused by lenses. Using the outputs of this housing calibration may require the use of special algorithms, depending on your downstream task. For more information please refer to [housing calibration](housing_calibration.md) and '[Should I calibrate my housing?](should_i_calibrate_my_housing.md)'.