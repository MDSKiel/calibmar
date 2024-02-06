#include "colmap/sfm/hybrid_mapper.h"

#include "colmap/estimators/alignment.h"
#include "colmap/estimators/pose_graph_optimizer.h"
#include "colmap/estimators/triangulation.h"
#include "colmap/util/misc.h"
#include "colmap/util/threading.h"

namespace colmap {

bool HybridMapper::Options::Check() const {
  CHECK_OPTION_GT(re_max_num_images, 0);
  CHECK_OPTION_GT(re_max_distance, 0);
  CHECK_OPTION_GE(pgo_rel_pose_multi, 0);
  CHECK_OPTION_GE(pgo_abs_pose_multi, 0);
  CHECK_OPTION_GE(pgo_smooth_multi, 0);
  return true;
}

HybridMapper::HybridMapper(
    std::shared_ptr<const IncrementalMapperOptions> incremental_options,
    std::shared_ptr<const DatabaseCache> database_cache,
    const std::string& database_path,
    const std::string& image_path)
    : incremental_options_(std::move(incremental_options)),
      database_cache_(std::move(database_cache)),
      database_path_(database_path),
      image_path_(image_path),
      reconstruction_(nullptr),
      scene_clustering_(nullptr) {}

void HybridMapper::BeginReconstruction(
    const std::shared_ptr<Reconstruction>& reconstruction) {
  CHECK(reconstruction_ == nullptr);
  reconstruction_ = reconstruction;
  reconstruction_->Load(*database_cache_);
  reconstruction_->SetUp(database_cache_->CorrespondenceGraph());

  // Initialize all camera poses as their pose priors.
  const Rigid3d cam_from_prior = Inverse(reconstruction_->PriorFromCam());
  for (const auto& image_el : reconstruction_->Images()) {
    Image& image = reconstruction_->Image(image_el.first);
    const Rigid3d cam_from_world_prior =
        cam_from_prior * image.CamFromWorldPrior();
    image.CamFromWorld() = cam_from_world_prior;

    reconstruction_->RegisterImage(image_el.first);
    num_registrations_.emplace(image_el.first, 0);
  }

  // Read view graph stats from database.
  upgraded_image_pair_stats_.clear();
  image_pair_stats_.clear();

  for (const auto& image_pair : reconstruction_->ImagePairs()) {
    image_pair_stats_.emplace(image_pair.first,
                              image_pair.second.num_total_corrs);
  }
}

void HybridMapper::EndReconstruction() {
  CHECK_NOTNULL(reconstruction_);

  reconstruction_->TearDown();
  reconstruction_ = nullptr;

  num_registrations_.clear();
  image_pair_stats_.clear();
  upgraded_image_pair_stats_.clear();
  image_id_to_name_.clear();
}

std::shared_ptr<const Reconstruction> HybridMapper::GetReconstruction() const {
  CHECK_NOTNULL(reconstruction_);
  return reconstruction_;
}

void HybridMapper::PartitionScene(
    const SceneClustering::Options& clustering_options) {
  database_.Open(database_path_);
  LOG(INFO) << "Reading images...";
  const auto images = database_.ReadAllImages();
  image_id_to_name_.reserve(images.size());
  for (const auto& image : images) {
    image_id_to_name_.emplace(image.ImageId(), image.Name());
  }

  scene_clustering_ = std::make_unique<SceneClustering>(
      SceneClustering::Create(clustering_options, database_));
  database_.Close();
}

void HybridMapper::ExtractViewGraphStats(
    const std::vector<std::shared_ptr<const Reconstruction>>& reconstructions) {
  // After reconstructing clusters, collect for new image pair stats.
  std::shared_ptr<const CorrespondenceGraph> correspondence_graph =
      database_cache_->CorrespondenceGraph();

  for (const auto& recon : reconstructions) {
    const std::vector<image_t>& reg_image_ids = recon->RegImageIds();

    for (const image_t image_id : reg_image_ids) {
      const Image& image = recon->Image(image_id);

      num_registrations_[image_id]++;

      for (point2D_t point2D_idx = 0; point2D_idx < image.NumPoints2D();
           ++point2D_idx) {
        if (image.Point2D(point2D_idx).HasPoint3D()) {
          // Here is the same as `SetObservationAsTriangulated` in
          // reconstruction.cc
          const Point2D& point2D = image.Point2D(point2D_idx);

          const auto corr_range =
              correspondence_graph->FindCorrespondences(image_id, point2D_idx);
          for (const auto* corr = corr_range.beg; corr < corr_range.end;
               ++corr) {
            if (!recon->ExistsImage(corr->image_id) ||
                !recon->IsImageRegistered(corr->image_id)) {
              continue;
            }
            const Image& corr_image = recon->Image(corr->image_id);
            const Point2D& corr_point2D = corr_image.Point2D(corr->point2D_idx);
            // Update number of shared 3D points between image pairs and make
            // sure to
            // only count the correspondences once (not twice forward and
            // backward).
            if (point2D.point3D_id == corr_point2D.point3D_id &&
                image_id < corr->image_id) {
              const image_pair_t pair_id =
                  Database::ImagePairToPairId(image_id, corr->image_id);
              if (upgraded_image_pair_stats_.count(pair_id) > 0) {
                upgraded_image_pair_stats_.at(pair_id) += 1;
              } else {
                upgraded_image_pair_stats_.emplace(pair_id, 1);
              }
            }
          }
        }
      }
    }  // End for all images.

  }  // End for all reconstructions.
}

void HybridMapper::ReconstructClusters(const Options& options) {
  auto leaf_clusters = scene_clustering_->GetLeafClusters();

  size_t total_num_images = 0;
  for (size_t i = 0; i < leaf_clusters.size(); ++i) {
    total_num_images += leaf_clusters[i]->image_ids.size();
    LOG(INFO) << StringPrintf("  Cluster %d with %d images",
                              i + 1,
                              leaf_clusters[i]->image_ids.size());
  }

  LOG(INFO) << StringPrintf("Clusters have %d images", total_num_images);

  // Determine the number of workers and threads per worker.
  const int kMaxNumThreads = -1;
  const int num_eff_threads = GetEffectiveNumThreads(kMaxNumThreads);
  const int kDefaultNumWorkers = 8;
  const int num_eff_workers =
      options.num_workers < 1
          ? std::min(static_cast<int>(leaf_clusters.size()),
                     std::min(kDefaultNumWorkers, num_eff_threads))
          : options.num_workers;
  const int num_threads_per_worker =
      std::max(1, num_eff_threads / num_eff_workers);

  // Start reconstructing the bigger clusters first for better resource usage.
  std::sort(leaf_clusters.begin(),
            leaf_clusters.end(),
            [](const SceneClustering::Cluster* cluster1,
               const SceneClustering::Cluster* cluster2) {
              return cluster1->image_ids.size() > cluster2->image_ids.size();
            });

  // Start the reconstruction workers. Use a separate reconstruction manager per
  // thread to avoid race conditions.
  reconstruction_managers_.reserve(leaf_clusters.size());
  {
    // I honestly don't know why, but having database opened during this
    // multi-threaded reconstruction is helpful to reduce the chance of getting
    // `database is locked error`. But this does not help to completely avoid
    // it.
    database_.Open(database_path_);
    ThreadPool thread_pool(num_eff_workers);
    for (const auto& cluster : leaf_clusters) {
      reconstruction_managers_[cluster] =
          std::make_shared<ReconstructionManager>();
      auto incremental_options = std::make_shared<IncrementalMapperOptions>(
          *incremental_options_.get());
      incremental_options->multiple_models = true;
      if (incremental_options->num_threads < 0) {
        incremental_options->num_threads = num_threads_per_worker;
      }

      for (const auto image_id : cluster->image_ids) {
        incremental_options->image_names.insert(image_id_to_name_.at(image_id));
      }

      thread_pool.AddTask(&HybridMapper::ReconstructCluster,
                          this,
                          incremental_options,
                          reconstruction_managers_[cluster]);
    }
    thread_pool.Wait();
    database_.Close();
  }
  std::vector<std::shared_ptr<const Reconstruction>> sub_recons;
  for (const auto& cluster_el : reconstruction_managers_) {
    for (size_t i = 0; i < cluster_el.second->Size(); i++) {
      sub_recons.push_back(cluster_el.second->Get(i));
    }
  }
  ExtractViewGraphStats(sub_recons);
}

void HybridMapper::ReconstructWeakArea(const Options& options) {
  // Collect for weakly reconstructed iamges.
  std::unordered_set<image_t> weak_image_ids;
  for (const auto& image : num_registrations_) {
    if (image.second == 0) {
      weak_image_ids.insert(image.first);
    }
  }

  // Find image pairs who have sufficient amount of inlier correspondences,
  // however not enough shared observations.
  const size_t kMinNumMatchesAsWeak =
      static_cast<size_t>(4.0 * incremental_options_->min_num_matches);
  const double re_min_ratio =
      incremental_options_->Triangulation().re_min_ratio;
  for (const auto& image_pair : image_pair_stats_) {
    size_t num_shared_observations = 0;
    if (upgraded_image_pair_stats_.count(image_pair.first)) {
      num_shared_observations = upgraded_image_pair_stats_.at(image_pair.first);
    }
    const double tri_ratio = static_cast<double>(num_shared_observations) /
                             static_cast<double>(image_pair.second);
    if (tri_ratio > re_min_ratio || image_pair.second < kMinNumMatchesAsWeak) {
      continue;
    }
    image_t image_id1, image_id2;
    Database::PairIdToImagePair(image_pair.first, &image_id1, &image_id2);
    weak_image_ids.insert(image_id1);
    weak_image_ids.insert(image_id2);
  }

  std::stringstream ss;
  if (weak_image_ids.size() > 0) {
    ss << "  => Identified ";
    for (const image_t image_id : weak_image_ids) {
      ss << " " << image_id;
    }
    ss << " as weak areas, and try to revisit these areas";
    LOG(INFO) << ss.str();
  } else {
    ss << "  => No weak area identified, continue reconstruction";
    LOG(INFO) << ss.str();
    return;
  }

  std::unordered_map<image_t, std::vector<image_t>> weak_area_clusters =
      FindLocalAreas(
          weak_image_ids, options.re_max_num_images, options.re_max_distance);

  LOG(INFO) << "  => Searched " << weak_area_clusters.size()
            << " clusters of images to revisit";

  ss.clear();
  size_t wr_cnt = 0;
  for (const auto& weak_area : weak_area_clusters) {
    ss << "Weak area " << wr_cnt << ", center image " << weak_area.first
       << ", contains images:";
    for (size_t j = 0; j < weak_area.second.size(); j++) {
      ss << " " << weak_area.second[j];
    }
    LOG(INFO) << ss.str();
    wr_cnt++;
  }

  // Determine the number of workers and threads per worker.
  const int kMaxNumThreads = -1;
  const int num_eff_threads = GetEffectiveNumThreads(kMaxNumThreads);
  const int kDefaultNumWorkers = 8;
  const int num_eff_workers =
      options.num_workers < 1
          ? std::min(static_cast<int>(weak_area_clusters.size()),
                     std::min(kDefaultNumWorkers, num_eff_threads))
          : options.num_workers;
  const int num_threads_per_worker =
      std::max(1, num_eff_threads / num_eff_workers);

  // Start the reconstruction workers. Use a separate reconstruction manager per
  // thread to avoid race conditions.
  std::unordered_map<image_t, std::shared_ptr<ReconstructionManager>>
      weak_area_reconstructions;
  weak_area_reconstructions.reserve(weak_area_clusters.size());

  {
    database_.Open(database_path_);
    ThreadPool thread_pool(num_eff_workers);
    for (const auto& cluster : weak_area_clusters) {
      // Use the weak image id as one of the initial image pair.
      weak_area_reconstructions[cluster.first] =
          std::make_shared<ReconstructionManager>();
      auto incremental_options = std::make_shared<IncrementalMapperOptions>(
          *incremental_options_.get());
      incremental_options->multiple_models = true;
      incremental_options->min_model_size = 3;
      incremental_options->init_image_id1 = cluster.first;
      if (incremental_options->num_threads < 0) {
        incremental_options->num_threads = num_threads_per_worker;
      }

      for (const auto image_id : cluster.second) {
        incremental_options->image_names.insert(image_id_to_name_.at(image_id));
      }

      thread_pool.AddTask(&HybridMapper::ReconstructCluster,
                          this,
                          incremental_options,
                          weak_area_reconstructions[cluster.first]);
    }
    thread_pool.Wait();
    database_.Close();
  }
  std::vector<std::shared_ptr<const Reconstruction>> sub_recons;
  for (const auto& cluster_el : weak_area_reconstructions) {
    for (size_t i = 0; i < cluster_el.second->Size(); i++) {
      sub_recons.push_back(cluster_el.second->Get(i));
    }
  }
  ExtractViewGraphStats(sub_recons);

  // Save the reconstruction results.
  for (auto& cluster : weak_area_reconstructions) {
    weak_area_reconstructions_.emplace_back(std::move(cluster.second));
  }
  for (const auto& recons : weak_area_reconstructions_) {
    for (size_t i = 0; i < recons->Size(); i++) {
      std::shared_ptr<const Reconstruction> recon = recons->Get(i);
      std::stringstream ss;
      ss << "WR recon: " << i << " contains images:";
      const std::vector<image_t>& reg_image_ids = recon->RegImageIds();
      for (const auto& image_id : reg_image_ids) {
        ss << " " << image_id;
      }
      LOG(INFO) << ss.str();
    }
  }
}

void HybridMapper::GlobalPoseGraphOptim(const Options& options) {
  PoseGraphOptimizer pgo_optim(reconstruction_);

  // Reuse some of the options from BundleAdjustment.
  BundleAdjustmentOptions ba_options =
      incremental_options_->GlobalBundleAdjustment();

  Eigen::Matrix6d information = Eigen::Matrix6d::Identity();
  size_t num_rel = 0, num_abs = 0, num_smooth = 0;

  // Collect for all sub-reconstructions.
  std::vector<std::shared_ptr<const Reconstruction>> sub_recons;
  for (const auto& cluster_el : reconstruction_managers_) {
    for (size_t i = 0; i < cluster_el.second->Size(); i++) {
      sub_recons.push_back(cluster_el.second->Get(i));
    }
  }

  // Collect for new sub-recons from weak area reconstructions.
  for (size_t i = 0; i < weak_area_reconstructions_.size(); i++) {
    const auto& recons = weak_area_reconstructions_[i];
    for (size_t j = 0; j < recons->Size(); j++) {
      sub_recons.push_back(recons->Get(j));
    }
  }

  // DEBUG mode: write out the stats of pose graph relative constraints.
  std::unordered_map<image_pair_t, size_t> pgo_rel_stats;
  // for(const auto& image_pair: image_pair_stats_){
  //   pgo_rel_stats.emplace(image_pair.first, 0);
  // }

  for (const auto recon : sub_recons) {
    for (const auto& image_pair : upgraded_image_pair_stats_) {
      image_t image_id1;
      image_t image_id2;
      Database::PairIdToImagePair(image_pair.first, &image_id1, &image_id2);

      if (!recon->ExistsImage(image_id1) || !recon->ExistsImage(image_id2) ||
          !recon->IsImageRegistered(image_id1) ||
          !recon->IsImageRegistered(image_id2)) {
        continue;
      }

      const Image& image_a = recon->Image(image_id1);
      const Image& image_b = recon->Image(image_id2);

      const Rigid3d cam2_from_cam1 =
          image_b.CamFromWorld() * Inverse(image_a.CamFromWorld());
      pgo_optim.AddRelativePose(image_id1,
                                image_id2,
                                cam2_from_cam1,
                                information * options.pgo_rel_pose_multi,
                                nullptr);
      num_rel++;
      if (pgo_rel_stats.count(image_pair.first) == 0) {
        pgo_rel_stats.emplace(std::make_pair(image_pair.first, 1));
      } else {
        pgo_rel_stats.at(image_pair.first) += 1;
      }
    }
  }

  LOG(INFO) << "Saving pgo rel stats to files";
  std::ofstream file("pgo_rel_stats.txt", std::ios_base::out);

  file << reconstruction_->Images().size() << std::endl;
  for (const auto& image_it : reconstruction_->Images()) {
    const Eigen::Vector3d& position = image_it.second.ProjectionCenter();
    file << image_it.first << " " << position[0] << " " << -position[1]
         << std::endl;
  }
  file << pgo_rel_stats.size() << std::endl;
  for (const auto& image_pair : pgo_rel_stats) {
    image_t image_id1;
    image_t image_id2;
    Database::PairIdToImagePair(image_pair.first, &image_id1, &image_id2);
    file << image_id1 << " " << image_id2 << " " << image_pair.second
         << std::endl;
  }
  file.close();

  const std::vector<image_t>& reg_image_ids = reconstruction_->RegImageIds();
  const Rigid3d cam_from_prior = Inverse(reconstruction_->PriorFromCam());

  for (const image_t image_id : reg_image_ids) {
    Image& image = reconstruction_->Image(image_id);
    const Rigid3d cam_from_world_prior =
        cam_from_prior * image.CamFromWorldPrior();
    pgo_optim.AddAbsolutePose(image_id,
                              cam_from_world_prior,
                              information * options.pgo_abs_pose_multi,
                              nullptr);
    num_abs++;
  }

  // For images that have no constraint at all, i.e. not registered in any of
  // the clsuters. Also for image pairs if they are still considered as a weak
  // image pair, add a smoothness contraint and a relative motion from the
  // navigation data.
  std::unordered_set<image_t> weak_image_ids;
  for (const auto& image : num_registrations_) {
    if (image.second == 0) {
      weak_image_ids.insert(image.first);
    }
  }

  // Find image pairs who have sufficient amount of inlier correspondences,
  // however no shared observations.
  const size_t kMinNumMatchesAsWeak =
      static_cast<size_t>(4.0 * incremental_options_->min_num_matches);
  const double re_min_ratio =
      incremental_options_->Triangulation().re_min_ratio;
  for (const auto& image_pair : image_pair_stats_) {
    size_t num_shared_observations = 0;
    if (upgraded_image_pair_stats_.count(image_pair.first)) {
      num_shared_observations = upgraded_image_pair_stats_.at(image_pair.first);
    }
    const double tri_ratio = static_cast<double>(num_shared_observations) /
                             static_cast<double>(image_pair.second);
    if (tri_ratio > re_min_ratio || image_pair.second < kMinNumMatchesAsWeak) {
      continue;
    }
    image_t image_id1, image_id2;
    Database::PairIdToImagePair(image_pair.first, &image_id1, &image_id2);
    weak_image_ids.insert(image_id1);
    weak_image_ids.insert(image_id2);
  }

  std::stringstream ss;
  if (weak_image_ids.size() > 0) {
    ss << "  => Identified ";
    for (const image_t image_id : weak_image_ids) {
      ss << " " << image_id;
    }
    ss << " as weak images, add smooth motion constraints to these images";
  }
  LOG(INFO) << ss.str();

  // Sort input images by their names.
  std::vector<image_t> ordered_image_ids = reg_image_ids;
  std::sort(ordered_image_ids.begin(),
            ordered_image_ids.end(),
            [&](const image_t image_id1, const image_t image_id2) {
              const Image& image1 = reconstruction_->Image(image_id1);
              const Image& image2 = reconstruction_->Image(image_id2);
              return image1.Name() < image2.Name();
            });

  for (const image_t image_id : weak_image_ids) {
    // Look for its previous image and its next image.
    size_t pos = 0;
    for (size_t i = 0; i < ordered_image_ids.size(); i++) {
      if (ordered_image_ids[i] == image_id) {
        pos = i;
        break;
      }
    }
    if (pos == 0) {
      // The first image in the list.
      image_t image_id_next = ordered_image_ids[pos + 1];

      const Image& image = reconstruction_->Image(image_id);
      const Image& image_next = reconstruction_->Image(image_id_next);

      const Rigid3d cam_from_world_prior_curr =
          cam_from_prior * image.CamFromWorldPrior();
      const Rigid3d cam_from_world_prior_next =
          cam_from_prior * image_next.CamFromWorldPrior();
      const Rigid3d next_from_curr =
          cam_from_world_prior_next * Inverse(cam_from_world_prior_curr);

      pgo_optim.AddRelativePose(image_id,
                                image_id_next,
                                next_from_curr,
                                information * options.pgo_smooth_multi,
                                nullptr);
    } else if (pos == ordered_image_ids.size() - 1) {
      // The last image in the list.
      image_t image_id_prev = ordered_image_ids[pos - 1];

      const Image& image_prev = reconstruction_->Image(image_id_prev);
      const Image& image = reconstruction_->Image(image_id);

      const Rigid3d cam_from_world_prior_prev =
          cam_from_prior * image_prev.CamFromWorldPrior();
      const Rigid3d cam_from_world_prior_curr =
          cam_from_prior * image.CamFromWorldPrior();
      const Rigid3d curr_from_prev =
          cam_from_world_prior_curr * Inverse(cam_from_world_prior_prev);

      pgo_optim.AddRelativePose(image_id_prev,
                                image_id,
                                curr_from_prev,
                                information * options.pgo_smooth_multi,
                                nullptr);
    } else {
      image_t image_id_prev = ordered_image_ids[pos - 1];
      image_t image_id_next = ordered_image_ids[pos + 1];

      // And additionally add smooth motion term.
      pgo_optim.AddSmoothMotion(image_id_prev,
                                image_id,
                                image_id_next,
                                information * options.pgo_smooth_multi,
                                nullptr);
      num_smooth++;
    }
  }

  LOG(INFO) << "  => Pose graph constraints:\n"
            << "Relative pose: " << num_rel << "\n"
            << "Absolute pose: " << num_abs << "\n"
            << "Smooth motion: " << num_smooth;
  pgo_optim.Solve();
}

void HybridMapper::ReconstructInlierTracks(const Options& options) {
  // First merge all sub-reconstructions to collect for all tracks.
  MergeClusters();
  // Collect for all inlier tracks and re-estimate them using the optimized
  // poses.

  std::shared_ptr<const Reconstruction> merged_recon =
      reconstruction_managers_.begin()->second->Get(0);

  // Re-use some options from IncrementalTriangulator.
  IncrementalTriangulator::Options tri_options =
      incremental_options_->Triangulation();

  const auto& points3Ds = merged_recon->Points3D();
  LOG(INFO) << "  => Re-triangulating " << points3Ds.size() << " 3D points";

  for (const auto& point3D_el : points3Ds) {
    if (point3D_el.second.track.Length() < 2 ||
        (point3D_el.second.track.Length() == 2 &&
         tri_options.ignore_two_view_tracks)) {
      continue;
    }
    const Track& track = point3D_el.second.track;
    TriangulateTrack(tri_options, track);
  }

  // Bundle adjust the triangulated 3D points.
  auto ba_options = incremental_options_->GlobalBundleAdjustment();
  ba_options.refine_focal_length = false;
  ba_options.refine_principal_point = false;
  ba_options.refine_extra_params = false;
  ba_options.refine_extrinsics = false;

  // Configure bundle adjustment to adjust only 3D points.
  BundleAdjustmentConfig ba_config;
  for (const image_t image_id : reconstruction_->RegImageIds()) {
    ba_config.AddImage(image_id);
  }

  reconstruction_->FilterObservationsWithNegativeDepth();
  BundleAdjuster bundle_adjuster(ba_options, ba_config);
  CHECK(bundle_adjuster.Solve(reconstruction_.get()));
}

void HybridMapper::PrintViewGraphStats() const {
  std::unordered_set<image_t> un_registered_image_ids;

  for (const auto& image : num_registrations_) {
    if (image.second == 0) {
      un_registered_image_ids.insert(image.first);
      LOG(INFO) << "Image " << image.first
                << " is not registered in any clusters";
    }
  }

  // Find image pairs who have sufficient amount of inlier correspondences,
  // however no shared observations.
  const size_t kMinNumMatchesAsWeak =
      static_cast<size_t>(4.0 * incremental_options_->min_num_matches);
  const double re_min_ratio =
      incremental_options_->Triangulation().re_min_ratio;
  for (const auto& image_pair : image_pair_stats_) {
    size_t num_shared_observations = 0;
    if (upgraded_image_pair_stats_.count(image_pair.first)) {
      num_shared_observations = upgraded_image_pair_stats_.at(image_pair.first);
    }
    const double tri_ratio = static_cast<double>(num_shared_observations) /
                             static_cast<double>(image_pair.second);
    if (tri_ratio > re_min_ratio || image_pair.second < kMinNumMatchesAsWeak) {
      continue;
    }
    image_t image_id1, image_id2;
    Database::PairIdToImagePair(image_pair.first, &image_id1, &image_id2);
    LOG(INFO) << "Image pair: " << image_id1 << " -- " << image_id2
              << " is weak, " << num_shared_observations << " / "
              << image_pair.second;
  }

  size_t num_valid_pairs = 0;
  size_t num_tri_pairs = 0;

  for (const auto& image_pair : image_pair_stats_) {
    size_t num_shared_observations = 0;
    if (upgraded_image_pair_stats_.count(image_pair.first)) {
      num_shared_observations = upgraded_image_pair_stats_.at(image_pair.first);
    }
    const double tri_ratio = static_cast<double>(num_shared_observations) /
                             static_cast<double>(image_pair.second);
    if (image_pair.second >
        static_cast<size_t>(incremental_options_->min_num_matches)) {
      num_valid_pairs++;
      if (tri_ratio > re_min_ratio) {
        num_tri_pairs++;
      }
    }
  }

  LOG(INFO) << "Number of un-registered images: "
            << un_registered_image_ids.size();

  LOG(INFO) << "Number of total image pairs: " << image_pair_stats_.size();
  LOG(INFO) << "Number of total upgraded image pairs: "
            << upgraded_image_pair_stats_.size();
  LOG(INFO) << "Number of total valid image pairs: " << num_valid_pairs;
  LOG(INFO) << "Number of total valid upgraded image pairs: " << num_tri_pairs;
  LOG(INFO) << "View-graph completeness ratio: "
            << static_cast<double>(upgraded_image_pair_stats_.size()) /
                   static_cast<double>(image_pair_stats_.size());

  // Temporary thing:
  // Save view graph stats in a file.
  LOG(INFO) << "Saving view graph stats to files";
  std::ofstream file("image_pair_stats.txt", std::ios_base::out);

  file << reconstruction_->Images().size() << std::endl;
  for (const auto& image_it : reconstruction_->Images()) {
    const Eigen::Vector3d& position = image_it.second.ProjectionCenter();
    file << image_it.first << " " << position[0] << " " << -position[1]
         << std::endl;
  }
  file << image_pair_stats_.size() << std::endl;
  for (const auto& image_pair : image_pair_stats_) {
    image_t image_id1;
    image_t image_id2;
    Database::PairIdToImagePair(image_pair.first, &image_id1, &image_id2);
    file << image_id1 << " " << image_id2 << " " << image_pair.second
         << std::endl;
  }
  file.close();

  file.open("upgraded_image_pair_stats.txt", std::ios_base::out);

  file << reconstruction_->Images().size() << std::endl;
  for (const auto& image_it : reconstruction_->Images()) {
    const Eigen::Vector3d& position = image_it.second.ProjectionCenter();
    file << image_it.first << " " << position[0] << " " << -position[1]
         << std::endl;
  }
  file << upgraded_image_pair_stats_.size() << std::endl;
  for (const auto& image_pair : upgraded_image_pair_stats_) {
    image_t image_id1;
    image_t image_id2;
    Database::PairIdToImagePair(image_pair.first, &image_id1, &image_id2);
    file << image_id1 << " " << image_id2 << " " << image_pair.second
         << std::endl;
  }
  file.close();
}

void HybridMapper::ReconstructCluster(
    std::shared_ptr<const IncrementalMapperOptions> incremental_options,
    std::shared_ptr<ReconstructionManager> reconstruction_manager) {
  IncrementalMapperController mapper(std::move(incremental_options),
                                     image_path_,
                                     database_path_,
                                     std::move(reconstruction_manager));
  mapper.Start();
  mapper.Wait();
}

std::unordered_map<image_t, std::vector<image_t>> HybridMapper::FindLocalAreas(
    const std::unordered_set<image_t>& image_ids,
    const size_t max_num_images,
    const double max_distance) const {
  const std::vector<image_t>& reg_image_ids = reconstruction_->RegImageIds();

  struct ImageInfo {
    image_t image_id;
    double distance;
  };

  std::unordered_map<image_t, std::vector<ImageInfo>> local_image_infos;

  for (const image_t image_id : image_ids) {
    Eigen::Vector3d center_image_position =
        reconstruction_->Image(image_id).ProjectionCenter();

    std::vector<ImageInfo> local_images;

    for (const image_t reg_image_id : reg_image_ids) {
      if (reg_image_id == image_id) {
        continue;
      }

      Eigen::Vector3d position =
          reconstruction_->Image(reg_image_id).ProjectionCenter();
      const double distance = (center_image_position - position).norm();
      if (distance > max_distance) {
        continue;
      }

      ImageInfo image_info;
      image_info.image_id = reg_image_id;
      image_info.distance = distance;
      local_images.push_back(image_info);
    }

    local_image_infos.emplace(image_id, local_images);
  }

  // Creating clusters of images for each image id.
  std::unordered_map<image_t, std::vector<image_t>> clusters;
  std::unordered_set<image_t> total_images_to_be_reconstructed;

  for (const image_t image_id : image_ids) {
    if (total_images_to_be_reconstructed.count(image_id)) {
      // This image will be revisited, no need to look for its local images.
      continue;
    }

    std::vector<image_t> cluster;

    total_images_to_be_reconstructed.insert(image_id);
    cluster.push_back(image_id);

    std::vector<ImageInfo>& local_images = local_image_infos.at(image_id);

    std::sort(local_images.begin(),
              local_images.end(),
              [](const ImageInfo& image1, const ImageInfo& image2) {
                return image1.distance < image2.distance;
              });

    for (size_t i = 0; i < local_images.size(); i++) {
      if (i >= max_num_images) {
        break;
      }
      total_images_to_be_reconstructed.insert(local_images[i].image_id);
      cluster.push_back(local_images[i].image_id);
    }

    clusters.emplace(std::make_pair(image_id, cluster));
  }

  return clusters;
}

void HybridMapper::UpdateSubReconstructions() {
  // Collect for all sub-reconstructions.
  std::vector<std::shared_ptr<Reconstruction>> sub_recons;
  for (auto& cluster_el : reconstruction_managers_) {
    for (size_t i = 0; i < cluster_el.second->Size(); i++) {
      sub_recons.push_back(cluster_el.second->Get(i));
    }
  }

  // Collect for new sub-recons from weak area reconstructions.
  for (size_t i = 0; i < weak_area_reconstructions_.size(); i++) {
    auto& recons = weak_area_reconstructions_[i];
    for (size_t j = 0; j < recons->Size(); j++) {
      sub_recons.push_back(recons->Get(j));
    }
  }

  for (auto& recon : sub_recons) {
    const std::vector<image_t>& reg_image_ids = recon->RegImageIds();

    std::unordered_set<point3D_t> updated_point3D_ids;

    for (const image_t image_id : reg_image_ids) {
      if (!reconstruction_->IsImageRegistered(image_id)) {
        continue;
      }

      Image& image = recon->Image(image_id);

      const Rigid3d cam_from_world_old = image.CamFromWorld();
      const Rigid3d world_from_cam_new =
          Inverse(reconstruction_->Image(image_id).CamFromWorld());

      for (const Point2D& point2D : image.Points2D()) {
        if (point2D.HasPoint3D() &&
            updated_point3D_ids.count(point2D.point3D_id) == 0) {
          Point3D& point3D = recon->Point3D(point2D.point3D_id);
          Eigen::Vector3d xyz_local = cam_from_world_old * point3D.xyz;
          Eigen::Vector3d xyz_new = world_from_cam_new * xyz_local;
          point3D.xyz = xyz_new;
          updated_point3D_ids.insert(point2D.point3D_id);
        }
      }
      // Update the image pose.
      image.CamFromWorld() = reconstruction_->Image(image_id).CamFromWorld();
    }

    // Refine all 3D points of this sub-reconstruction.
    auto ba_options = incremental_options_->GlobalBundleAdjustment();
    ba_options.refine_focal_length = false;
    ba_options.refine_principal_point = false;
    ba_options.refine_extra_params = false;
    ba_options.refine_extrinsics = false;
    ba_options.solver_options.minimizer_progress_to_stdout = false;
    ba_options.print_summary = false;

    // Configure bundle adjustment to adjust only 3D points.
    BundleAdjustmentConfig ba_config;
    for (const image_t image_id : reg_image_ids) {
      ba_config.AddImage(image_id);
    }

    recon->FilterObservationsWithNegativeDepth();
    BundleAdjuster bundle_adjuster(ba_options, ba_config);
    CHECK(bundle_adjuster.Solve(recon.get()));
  }
}

void HybridMapper::MergeClusters(const SceneClustering::Cluster& cluster) {
  // Extract all reconstructions from all child clusters.
  std::vector<std::shared_ptr<Reconstruction>> reconstructions;
  for (const auto& child_cluster : cluster.child_clusters) {
    if (!child_cluster.child_clusters.empty()) {
      MergeClusters(child_cluster);
    }

    auto& reconstruction_manager = reconstruction_managers_.at(&child_cluster);
    for (size_t i = 0; i < reconstruction_manager->Size(); ++i) {
      reconstructions.push_back(reconstruction_manager->Get(i));
    }
  }

  // Try to merge all child cluster reconstruction.
  while (reconstructions.size() > 1) {
    bool merge_success = false;
    for (size_t i = 0; i < reconstructions.size(); ++i) {
      const int num_reg_images_i = reconstructions[i]->NumRegImages();
      for (size_t j = 0; j < i; ++j) {
        const double kMaxReprojError = 32.0;
        const int num_reg_images_j = reconstructions[j]->NumRegImages();
        if (MergeReconstructions(kMaxReprojError,
                                 *reconstructions[j],
                                 reconstructions[i].get(),
                                 false)) {
          LOG(INFO) << StringPrintf(
              " => Merged clusters with %d and %d images into %d images",
              num_reg_images_i,
              num_reg_images_j,
              reconstructions[i]->NumRegImages());
          reconstructions.erase(reconstructions.begin() + j);
          merge_success = true;
          break;
        }
      }

      if (merge_success) {
        break;
      }
    }

    if (!merge_success) {
      break;
    }
  }

  // Insert a new reconstruction manager for merged cluster.
  auto& reconstruction_manager = reconstruction_managers_[&cluster];
  reconstruction_manager = std::make_shared<ReconstructionManager>();
  for (const auto& reconstruction : reconstructions) {
    reconstruction_manager->Get(reconstruction_manager->Add()) = reconstruction;
  }

  // Delete all merged child cluster reconstruction managers.
  for (const auto& child_cluster : cluster.child_clusters) {
    reconstruction_managers_.erase(&child_cluster);
  }
}

void HybridMapper::MergeClusters() {
  // Update sub-reconstructions and then try to merge them. Since we have
  // updated camera poses and 3D points. There is no need to perform
  // similarity transformation to align them when merging.
  UpdateSubReconstructions();

  LOG(INFO) << " => Merging clusters";

  MergeClusters(*scene_clustering_->GetRootCluster());

  LOG(INFO) << " => Merging weak areas";

  // Also merge weak area reconstructions if there are any.
  std::shared_ptr<Reconstruction> merged_recon =
      reconstruction_managers_.begin()->second->Get(0);
  const double kMaxReprojError = 32.0;
  for (size_t i = 0; i < weak_area_reconstructions_.size(); i++) {
    for (size_t j = 0; j < weak_area_reconstructions_[i]->Size(); j++) {
      const int num_reg_images_before = merged_recon->NumRegImages();
      const int num_reg_images_wr =
          weak_area_reconstructions_[i]->Get(j)->NumRegImages();
      MergeReconstructions(kMaxReprojError,
                           *weak_area_reconstructions_[i]->Get(j),
                           merged_recon.get(),
                           false);
      LOG(INFO) << StringPrintf(
          " => Merged weak area with %d and %d images into %d images",
          num_reg_images_wr,
          num_reg_images_before,
          merged_recon->NumRegImages());
    }
  }
}

bool HybridMapper::TriangulateTrack(
    const IncrementalTriangulator::Options& tri_options, const Track& track) {
  // Setup data for triangulation estimation.
  std::vector<TriangulationEstimator::PointData> point_data;
  point_data.resize(track.Length());
  std::vector<TriangulationEstimator::PoseData> pose_data;
  pose_data.resize(track.Length());
  for (size_t i = 0; i < track.Length(); i++) {
    const TrackElement& track_el = track.Element(i);
    const Image& image = reconstruction_->Image(track_el.image_id);
    const Point2D& point2D = image.Point2D(track_el.point2D_idx);
    const Camera& camera = reconstruction_->Camera(image.CameraId());
    if (!tri_options.enable_refraction) {
      // Non-refractive case.
      point_data[i].point = point2D.xy;
      point_data[i].point_normalized = camera.CamFromImg(point_data[i].point);
      pose_data[i].proj_matrix = image.CamFromWorld().ToMatrix();
      pose_data[i].proj_center = image.ProjectionCenter();
      pose_data[i].camera = &camera;
    } else {
      // Refractive case.
      Rigid3d virtual_from_real;
      Camera virtual_camera;
      camera.ComputeVirtual(point2D.xy, virtual_camera, virtual_from_real);
      point_data[i].point = point2D.xy;
      point_data[i].point_normalized =
          virtual_camera.CamFromImg(point_data[i].point);
      const Rigid3d virtual_from_world =
          virtual_from_real * image.CamFromWorld();
      pose_data[i].proj_matrix = virtual_from_world.ToMatrix();
      pose_data[i].proj_center = virtual_from_world.rotation.inverse() *
                                 -virtual_from_world.translation;
      pose_data[i].camera = &virtual_camera;
    }
  }

  // Setup estimation options.
  EstimateTriangulationOptions tri_est_options;
  tri_est_options.min_tri_angle = DegToRad(tri_options.min_angle);
  tri_est_options.residual_type =
      TriangulationEstimator::ResidualType::ANGULAR_ERROR;
  tri_est_options.ransac_options.max_error =
      DegToRad(tri_options.create_max_angle_error);
  tri_est_options.ransac_options.confidence = 0.9999;
  tri_est_options.ransac_options.min_inlier_ratio = 0.02;
  tri_est_options.ransac_options.max_num_trials = 10000;

  // Enforce exhaustive sampling for small track lengths.
  const size_t kExhaustiveSamplingThreshold = 15;
  if (point_data.size() <= kExhaustiveSamplingThreshold) {
    tri_est_options.ransac_options.min_num_trials =
        NChooseK(point_data.size(), 2);
  }

  // Estimate triangulation.
  Eigen::Vector3d xyz;
  std::vector<char> inlier_mask;
  if (!EstimateTriangulation(
          tri_est_options, point_data, pose_data, &inlier_mask, &xyz)) {
    return 0;
  }

  Track new_track;
  new_track.Reserve(track.Length());
  for (size_t i = 0; i < inlier_mask.size(); ++i) {
    if (inlier_mask[i]) {
      new_track.AddElement(track.Element(i));
    }
  }

  // Add estimated point to reconstruction.
  reconstruction_->AddPoint3D(xyz, std::move(new_track));
  return true;
}

}  // namespace colmap