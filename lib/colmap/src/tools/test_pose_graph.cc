#include "base/reconstruction.h"
#include "controllers/incremental_mapper.h"
#include "sfm/incremental_mapper.h"
#include "util/misc.h"
#include "util/option_manager.h"

using namespace colmap;

int main(int argc, char* argv[]) {
  colmap::PrintHeading1("Testing Pose Graph");

  std::string output_path;

  OptionManager options;
  options.AddDatabaseOptions();
  options.AddRequiredOption("output_path", &output_path);
  options.Parse(argc, argv);

  PrintHeading2("Loading database");

  Database database(*options.database_path);

  std::unordered_set<std::string> image_names;
  const size_t min_num_matches =
      static_cast<size_t>(options.mapper->min_num_matches);

  DatabaseCache database_cache;
  database_cache.Load(database, min_num_matches,
                      options.mapper->ignore_watermarks, image_names);

  IncrementalMapper mapper(&database_cache);

  // Start a new reconstruction.
  Reconstruction reconstruction;
  mapper.BeginReconstruction(&reconstruction);

  IncrementalMapper::Options mapper_options;
  mapper_options.init_min_tri_angle = 0.0;
  mapper_options.init_max_forward_motion = 1.0;

  // Set all images in reconstruction as registered
  PrintHeading2("Initializing pose graph");
  for (const auto& image_iter : reconstruction.Images()) {
    Image& image = reconstruction.Image(image_iter.first);
    image.SetRegistered(true);
  }

  PrintHeading2("Preparing optimization");
  PoseGraphOptimizationOptions pgo_options =
      options.mapper->GlobalPoseGraphOptimization();

  pgo_options.solver_options.function_tolerance /= 10;
  pgo_options.solver_options.gradient_tolerance /= 10;
  pgo_options.solver_options.parameter_tolerance /= 10;
  pgo_options.solver_options.max_num_iterations = 500;
  pgo_options.solver_options.max_linear_solver_iterations = 200;

  // Run pose graph
  mapper.EstimateInitialGlobalPoseGraph(mapper_options, pgo_options);

  // Output
  reconstruction.WriteText(output_path);

  return 0;
}