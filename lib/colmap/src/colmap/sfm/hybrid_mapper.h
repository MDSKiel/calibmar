#pragma once

#include "colmap/controllers/incremental_mapper.h"
#include "colmap/scene/database.h"
#include "colmap/scene/database_cache.h"
#include "colmap/scene/reconstruction.h"
#include "colmap/scene/scene_clustering.h"

namespace colmap {
class HybridMapper {
 public:
  struct Options {
    // The number of workers used to reconstruct clusters in parallel.
    int num_workers = -1;

    // The maximum number of images for weak area revisit.
    size_t re_max_num_images = 30;

    // How large the radius is when selecting a weak area to revisit.
    double re_max_distance = 2.5;

    // The multiplier factor for the relative pose term in pose graph
    // optimization.
    double pgo_rel_pose_multi = 1.0;

    // The multiplier factor for the absolute pose prior term in pose graph
    // optimization.
    double pgo_abs_pose_multi = 0.1;

    // The multiplier factor for the motion smoothness term in pose graph
    // optimization.
    double pgo_smooth_multi = 100.0;

    bool Check() const;
  };

  explicit HybridMapper(
      std::shared_ptr<const IncrementalMapperOptions> incremental_options,
      std::shared_ptr<const DatabaseCache> database_cache,
      const std::string& database_path,
      const std::string& image_path);

  void BeginReconstruction(
      const std::shared_ptr<Reconstruction>& reconstruction);

  // Cleanup the mapper after the current reconstruction is done.
  void EndReconstruction();

  std::shared_ptr<const Reconstruction> GetReconstruction() const;

  void PartitionScene(const SceneClustering::Options& clustering_options);

  void ExtractViewGraphStats(
      const std::vector<std::shared_ptr<const Reconstruction>>&
          reconstructions);

  void ReconstructClusters(const Options& options);

  void ReconstructWeakArea(const Options& options);

  void GlobalPoseGraphOptim(const Options& options);

  void ReconstructInlierTracks(const Options& options);

  void PrintViewGraphStats() const;

 protected:
  void ReconstructCluster(
      std::shared_ptr<const IncrementalMapperOptions> incremental_options,
      std::shared_ptr<ReconstructionManager> reconstruction_manager);

  std::unordered_map<image_t, std::vector<image_t>> FindLocalAreas(
      const std::unordered_set<image_t>& image_ids,
      const size_t max_num_images,
      const double max_distance) const;

  void UpdateSubReconstructions();

  void MergeClusters(const SceneClustering::Cluster& cluster);

  void MergeClusters();

  bool TriangulateTrack(const IncrementalTriangulator::Options& tri_options,
                        const Track& track);

  // Class that holds options for incremental mapping.
  const std::shared_ptr<const IncrementalMapperOptions> incremental_options_;
  // Class that holds all necessary data from database in memory.
  const std::shared_ptr<const DatabaseCache> database_cache_;

  const std::string database_path_;
  const std::string image_path_;

  Database database_;

  // Class that holds data of the reconstruction.
  std::shared_ptr<Reconstruction> reconstruction_;

  // Collect how many times an image is registered.
  std::unordered_map<image_t, size_t> num_registrations_;

  // Image pair stats from the database.
  std::unordered_map<image_pair_t, size_t> image_pair_stats_;
  // Upgraded image pair stats.
  std::unordered_map<image_pair_t, size_t> upgraded_image_pair_stats_;

  std::unique_ptr<SceneClustering> scene_clustering_;

  std::unordered_map<image_t, std::string> image_id_to_name_;

  std::unordered_map<const SceneClustering::Cluster*,
                     std::shared_ptr<ReconstructionManager>>
      reconstruction_managers_;

  std::vector<std::shared_ptr<ReconstructionManager>>
      weak_area_reconstructions_;
};
}  // namespace colmap