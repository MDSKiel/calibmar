#include "undistort.h"

#include <FreeImage.h>
#include <colmap/image/warp.h>

namespace calibmar::undistort {
  colmap::Camera CreateUndistortedCamera(colmap::Camera camera) {
    colmap::Camera undistort_camera = colmap::Camera::CreateFromModelId(
        colmap::kInvalidCameraId, colmap::PinholeCameraModel::model_id, 1, camera.width, camera.height);

    undistort_camera.SetPrincipalPointX(camera.PrincipalPointX());
    undistort_camera.SetPrincipalPointY(camera.PrincipalPointY());

    if (camera.FocalLengthIdxs().size() == 1) {
      undistort_camera.SetFocalLength(camera.FocalLength());
    }
    else {
      undistort_camera.SetFocalLengthX(camera.FocalLengthX());
      undistort_camera.SetFocalLengthY(camera.FocalLengthY());
    }

    return undistort_camera;
  }

  void UndistortImage(std::unique_ptr<Pixmap> distorted_pixmap, const colmap::Camera& distorted_camera,
                      Pixmap& undistorted_pixmap, colmap::Camera& undistorted_camera) {
    cv::Mat image_data(distorted_pixmap->Data());
    FIBITMAP* bitmap = FreeImage_ConvertFromRawBitsEx(false, image_data.data, FREE_IMAGE_TYPE::FIT_BITMAP,
                                                      distorted_pixmap->Width(), distorted_pixmap->Height(), image_data.step,
                                                      image_data.channels() == 1 ? 8 : 24, 0, 0, 0, true);

    undistorted_camera = CreateUndistortedCamera(distorted_camera);

    colmap::Bitmap bmp(bitmap);
    colmap::Bitmap undistorted_bmp;
    colmap::WarpImageBetweenCameras(distorted_camera, undistorted_camera, bmp, &undistorted_bmp);
    bmp.CloneMetadata(&undistorted_bmp);

    // clone, because colmap::Bitmap will deallocate its own buffer
    cv::Mat undistorted_mat =
        cv::Mat(undistorted_bmp.Height(), undistorted_bmp.Width(), undistorted_bmp.Channels() == 1 ? CV_8UC1 : CV_8UC3,
                FreeImage_GetBits(undistorted_bmp.Data()), FreeImage_GetPitch(undistorted_bmp.Data()));
    undistorted_pixmap.Assign(undistorted_mat.clone());
    // also flip, because FreeImage stores data bottom up
    cv::flip(undistorted_mat, undistorted_pixmap.Data(), 0);
  }
}