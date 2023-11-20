#include "colmap/controllers/option_manager.h"
#include "colmap/scene/reconstruction.h"
#include "colmap/util/misc.h"

using namespace colmap;

int main(int argc, char** argv) {
  OptionManager options;
  std::string input_path;
  std::string output_path;
  size_t center_view_id = kInvalidImageId;
  double radius;
  std::string pose_prior_path = "";
  options.AddImageOptions();
  options.AddRequiredOption("input_path", &input_path);
  options.AddRequiredOption("output_path", &output_path);
  options.AddRequiredOption("center_view_id", &center_view_id);
  options.AddRequiredOption(
      "radius",
      &radius,
      "Radius to select images around the center view, unit meter");
  options.AddDefaultOption(
      "pose_prior_path",
      &pose_prior_path,
      "Whether to copy pose prior files together with image files");
  options.Parse(argc, argv);

  if (!ExistsDir(input_path)) {
    std::cerr << "ERROR: `input_path` is not a directory" << std::endl;
    return EXIT_FAILURE;
  }

  if (!ExistsDir(output_path)) {
    std::cerr << "ERROR: `output_path` is not a directory" << std::endl;
    return EXIT_FAILURE;
  }
  CHECK_GT(radius, 0.0) << "Radius must be greater than 0";
  CHECK_NE(center_view_id, kInvalidImageId);

  bool copy_pose_priors = pose_prior_path != "";

  PrintHeading1("Select images by position");

  CreateDirIfNotExists(JoinPaths(output_path, "images"));
  if (copy_pose_priors) {
    CreateDirIfNotExists(JoinPaths(output_path, "pose_priors"));
  }

  auto reconstruction = std::make_shared<Reconstruction>();
  reconstruction->Read(input_path);

  const std::vector<image_t>& reg_image_ids = reconstruction->RegImageIds();

  Eigen::Vector3d center_view_position =
      reconstruction->Image(static_cast<image_t>(center_view_id))
          .ProjectionCenter();

  for (const image_t image_id : reg_image_ids) {
    const Image& image = reconstruction->Image(image_id);

    Eigen::Vector3d position = image.ProjectionCenter();
    const double distance = (center_view_position - position).norm();
    if (distance > radius) {
      continue;
    }

    std::cout << "Copying " << image_id << " , distance " << distance
              << std::endl;
    const std::string& image_name = image.Name();
    std::string src_path = JoinPaths(*options.image_path.get(), image_name);
    std::string dst_path = JoinPaths(output_path, "images", image_name);

    // Create a parent directory in case the parent directory does not exist.
    CreateDirIfNotExists(GetParentDir(dst_path));
    FileCopy(src_path, dst_path);
    if (copy_pose_priors) {
      std::string file_root;
      std::string file_ext;
      SplitFileExtension(image_name, &file_root, &file_ext);
      const std::string csv_file = file_root + ".csv";
      src_path = JoinPaths(pose_prior_path, csv_file);
      dst_path = JoinPaths(output_path, "pose_priors", csv_file);
      if (ExistsFile(src_path)) {
        CreateDirIfNotExists(GetParentDir(dst_path));
        FileCopy(src_path, dst_path);
      } else {
        std::cerr << "WARN: csv file" << file_root << "doesn't exist "
                  << std::endl;
      }
    }
  }

  return true;
}
