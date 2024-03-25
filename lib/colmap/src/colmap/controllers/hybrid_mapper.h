#pragma once

#include "colmap/controllers/incremental_mapper.h"
#include "colmap/scene/scene_clustering.h"
#include "colmap/sfm/hybrid_mapper.h"
#include "colmap/util/threading.h"

namespace colmap {

class HybridMapperController : public Thread {
 public:
  struct Options {
    // The path to the image folder which are used as input.
    std::string image_path;

    // The path to the database file which is used as input.
    std::string database_path;

    // The number of workers used to reconstruct clusters in parallel.
    int num_workers = -1;

    // The maxinum number of weak area revists.
    size_t max_num_weak_area_revisit = 1;

    // The maximum number of images for weak area revisit.
    size_t re_max_num_images = 30;

    // How large the radius is when selecting a weak area to revisit.
    double re_max_distance = 2.5;

    // The multiplier factor for the relative pose term in pose graph
    // optimization.
    double pgo_rel_pose_multi = 1.0;

    // The multiplier factor for the absolute pose prior term in pose graph
    // optimization.
    double pgo_abs_pose_multi = 0.001;

    // The multiplier factor for the motion smoothness term in pose graph
    // optimization.
    double pgo_smooth_multi = 2.0;

    // Whether to additionally export the optimized pose graph results together
    // with the reconstruction.
    bool show_pgo_result = false;

    // Options for clustering the scene graph.
    SceneClustering::Options clustering_options;

    // Options used to reconstruction each cluster individually.
    IncrementalMapperOptions incremental_options;

    HybridMapper::Options Mapper() const;

    bool Check() const;
  };

  HybridMapperController(
      const Options& options,
      std::shared_ptr<ReconstructionManager> reconstruction_manager);

 private:
  void Run() override;
  bool LoadDatabase();

  const Options options_;
  std::shared_ptr<ReconstructionManager> reconstruction_manager_;
  std::shared_ptr<DatabaseCache> database_cache_;
};

}  // namespace colmap