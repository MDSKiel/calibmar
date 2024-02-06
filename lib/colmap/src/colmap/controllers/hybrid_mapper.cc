#include "colmap/controllers/hybrid_mapper.h"

#include "colmap/util/misc.h"

namespace colmap {
namespace {
void AdjustGlobalBundle(const IncrementalMapperOptions& options,
                        IncrementalMapper* mapper) {
  BundleAdjustmentOptions custom_ba_options = options.GlobalBundleAdjustment();

  const size_t num_reg_images = mapper->GetReconstruction().NumRegImages();

  // Use stricter convergence criteria for first registered images.
  const size_t kMinNumRegImagesForFastBA = 10;
  if (num_reg_images < kMinNumRegImagesForFastBA) {
    custom_ba_options.solver_options.function_tolerance /= 10;
    custom_ba_options.solver_options.gradient_tolerance /= 10;
    custom_ba_options.solver_options.parameter_tolerance /= 10;
    custom_ba_options.solver_options.max_num_iterations *= 2;
    custom_ba_options.solver_options.max_linear_solver_iterations = 200;
  }

  if (options.ba_fix_intrin_until_num_images > 0 &&
      num_reg_images >
          static_cast<size_t>(options.ba_fix_intrin_until_num_images)) {
    custom_ba_options.refine_focal_length = false;
    custom_ba_options.refine_principal_point = false;
    custom_ba_options.refine_extra_params = false;
  }

  if (options.enable_refraction &&
      options.ba_fix_refrac_params_until_num_images > 0 &&
      num_reg_images <
          static_cast<size_t>(options.ba_fix_refrac_params_until_num_images)) {
    custom_ba_options.refine_refrac_params = false;
  }

  if (num_reg_images <
      static_cast<size_t>(options.ba_refine_prior_from_cam_after_num_images)) {
    custom_ba_options.refine_prior_from_cam = false;
  }

  PrintHeading1("Global bundle adjustment");
  mapper->AdjustGlobalBundle(options.Mapper(), custom_ba_options);
}

void IterativeGlobalRefinement(const IncrementalMapperOptions& options,
                               IncrementalMapper* mapper) {
  PrintHeading1("Retriangulation");
  CompleteAndMergeTracks(options, mapper);
  LOG(INFO) << "  => Retriangulated observations: "
            << mapper->Retriangulate(options.Triangulation());

  for (int i = 0; i < options.ba_global_max_refinements; ++i) {
    const size_t num_observations =
        mapper->GetReconstruction().ComputeNumObservations();
    size_t num_changed_observations = 0;
    AdjustGlobalBundle(options, mapper);
    num_changed_observations += CompleteAndMergeTracks(options, mapper);
    num_changed_observations += FilterPoints(options, mapper);
    const double changed =
        num_observations == 0
            ? 0
            : static_cast<double>(num_changed_observations) / num_observations;
    LOG(INFO) << StringPrintf("  => Changed observations: %.6f", changed);
    if (changed < options.ba_global_max_refinement_change) {
      break;
    }
  }

  FilterImages(options, mapper);
}

}  // namespace

HybridMapper::Options HybridMapperController::Options::Mapper() const {
  HybridMapper::Options options;
  options.num_workers = num_workers;
  options.re_max_num_images = re_max_num_images;
  options.re_max_distance = re_max_distance;
  options.pgo_rel_pose_multi = pgo_rel_pose_multi;
  options.pgo_abs_pose_multi = pgo_abs_pose_multi;
  options.pgo_smooth_multi = pgo_smooth_multi;
  return options;
}

bool HybridMapperController::Options::Check() const {
  CHECK_OPTION_GE(num_workers, -1);
  CHECK_OPTION_GE(max_num_weak_area_revisit, 0);
  CHECK_OPTION_GT(re_max_num_images, 0);
  CHECK_OPTION_GT(re_max_distance, 0);
  CHECK_OPTION_GE(pgo_rel_pose_multi, 0);
  CHECK_OPTION_GE(pgo_abs_pose_multi, 0);
  CHECK_OPTION_GE(pgo_smooth_multi, 0);
  clustering_options.Check();
  CHECK_EQ(clustering_options.branching, 2);
  incremental_options.Check();
  return true;
}

HybridMapperController::HybridMapperController(
    const Options& options,
    std::shared_ptr<ReconstructionManager> reconstruction_manager)
    : options_(options),
      reconstruction_manager_(std::move(reconstruction_manager)) {
  CHECK(options_.Check());
}

void HybridMapperController::Run() {
  if (!LoadDatabase()) {
    return;
  }

  // Initialize a global reconstruction by the pose priors.
  PrintHeading1("Initialize global reconstruction");

  // Create a temporary global reconstruction and register all images as their
  // pose priors.
  std::shared_ptr<Reconstruction> global_recon =
      std::make_shared<Reconstruction>();
  // Set prior_from_cam if pose prior is used in reconstruction.
  if (options_.incremental_options.use_pose_prior &&
      !options_.incremental_options.prior_from_cam.empty()) {
    const std::vector<double> params =
        CSVToVector<double>(options_.incremental_options.prior_from_cam);
    global_recon->PriorFromCam() =
        Rigid3d(Eigen::Quaterniond(params[0], params[1], params[2], params[3])
                    .normalized(),
                Eigen::Vector3d(params[4], params[5], params[6]));
  }
  HybridMapper hybrid_mapper(std::make_shared<const IncrementalMapperOptions>(
                                 options_.incremental_options),
                             database_cache_,
                             options_.database_path,
                             options_.image_path);
  hybrid_mapper.BeginReconstruction(global_recon);

  //////////////////////////////////////////////////////////////////////////////
  // Cluster scene
  //////////////////////////////////////////////////////////////////////////////

  hybrid_mapper.PartitionScene(options_.clustering_options);

  //////////////////////////////////////////////////////////////////////////////
  // Reconstruct clusters
  //////////////////////////////////////////////////////////////////////////////

  PrintHeading1("Reconstructing clusters");
  hybrid_mapper.ReconstructClusters(options_.Mapper());

  for (size_t i = 0; i < options_.max_num_weak_area_revisit; i++) {
    PrintHeading1("Reconstructing weak areas");
    hybrid_mapper.ReconstructWeakArea(options_.Mapper());
  }

  PrintHeading1("Global pose graph optimization");
  hybrid_mapper.GlobalPoseGraphOptim(options_.Mapper());

  PrintHeading1("Reconstruct inlier tracks");
  hybrid_mapper.ReconstructInlierTracks(options_.Mapper());

  std::shared_ptr<Reconstruction> pgo_result;
  if (options_.show_pgo_result) {
    // Make a copy of the current stage of reconstruction and write out.
    pgo_result = std::make_shared<Reconstruction>(*global_recon.get());
  }

  PrintHeading1("Print view graph stats");
  hybrid_mapper.PrintViewGraphStats();

  //////////////////////////////////////////////////////////////////////////////
  // Bundle adjustment
  //////////////////////////////////////////////////////////////////////////////

  PrintHeading1("Final global bundle adjustment");

  // This TearDown is necessary as it cleans up the image pair stats and so on
  // for later running incremental mapper on the reconstruction.
  global_recon->TearDown();

  IncrementalMapper incremental_mapper(database_cache_);
  incremental_mapper.BeginReconstruction(global_recon);

  IterativeGlobalRefinement(options_.incremental_options, &incremental_mapper);

  if (options_.incremental_options.extract_colors) {
    PrintHeading1("Extracting color");
    global_recon->ExtractColorsForAllImages(options_.image_path);
  }

  hybrid_mapper.EndReconstruction();

  reconstruction_manager_->Get(reconstruction_manager_->Add()) = global_recon;

  if (options_.show_pgo_result) {
    reconstruction_manager_->Get(reconstruction_manager_->Add()) = pgo_result;
  }

  // Debug, export the merged reconstruction.
  // auto& reconstruction_managers = hybrid_mapper.GetReconstructionManagers();
  // CHECK_EQ(reconstruction_managers.size(), 1);
  // *reconstruction_manager_ =
  // std::move(reconstruction_managers.begin()->second);

  // Debug, output all clustered reconstructions
  // auto& reconstruction_managers = hybrid_mapper.GetReconstructionManagers();
  // for (const auto& cluster_el : reconstruction_managers) {
  //   for (size_t i = 0; i < cluster_el.second.Size(); i++) {
  //     const Reconstruction& recon = cluster_el.second.Get(i);
  //     const size_t idx = reconstruction_manager_->Add();
  //     reconstruction_manager_->Get(idx) = recon;
  //   }
  // }

  GetTimer().PrintMinutes();
}

bool HybridMapperController::LoadDatabase() {
  PrintHeading1("Loading database");

  std::unordered_set<std::string> image_names;
  Database database(options_.database_path);
  Timer timer;
  timer.Start();
  const size_t min_num_matches =
      static_cast<size_t>(options_.incremental_options.min_num_matches);
  database_cache_ =
      DatabaseCache::Create(database,
                            min_num_matches,
                            options_.incremental_options.ignore_watermarks,
                            image_names);
  timer.PrintMinutes();

  if (database_cache_->NumImages() == 0) {
    LOG(WARNING) << "No images with matches found in the database.";
    return false;
  }

  return true;
}

}  // namespace colmap