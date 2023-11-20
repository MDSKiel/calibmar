import os
import sys
import subprocess
import argparse
import shutil
import imageio.v3 as imageio
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional, List, Tuple

from read_write_model import detect_model_format

CAMERA_MODELS = [
    "SIMPLE_PINHOLE",
    "PINHOLE",
    "SIMPLE_RADIAL",
    "RADIAL",
    "OPENCV",
    "OPENCV_FISHEYE",
    "FULL_OPENCV",
    "FOV",
    "SIMPLE_RADIAL_FISHEYE",
    "RADIAL_FISHEYE",
    "THIN_PRISM_FISHEYE",
    "METASHAPE_FISHEYE",
]

CAMERA_REFRAC_MODELS = ["FLATPORT", "DOMEPORT"]


def print_heading1(heading):
    print()
    print("=" * 78)
    print(heading)
    print("=" * 78)
    print()


def print_heading2(heading):
    print()
    print(heading)
    print("-" * min(len(heading), 78))


def whereis(file):
    if sys.platform.startswith("win"):
        cmd = "where"
    else:
        cmd = "which"
    try:
        ret = subprocess.run(
            [cmd, file], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=True
        )
        return os.path.split(ret.stdout.decode())[0]
    except subprocess.CalledProcessError:
        return None


def exec_cmd(cmd):
    print_heading2(" ".join(cmd))
    try:
        proc = subprocess.Popen(cmd)
        proc.wait()
        return proc.returncode
    except KeyboardInterrupt:
        sys.exit("\r\nProcess canceled by user, all files remains")


def get_recursive_file_list(path: Path):
    lst = []
    for p in path.rglob("*"):
        if p.is_file():
            lst.append(p)
    return lst


def create_omv_camera_xml(w: int, h: int, output_path: str):
    proj = ET.Element("Projection")
    proj.set("Version", f"{0.1}")

    proj_params = ET.SubElement(proj, "ProjectionParametersPerspective")
    proj_params.set("Version", f"{0.1}")
    proj_params.set("Focallength", f"{0.5 * float(w)}")
    proj_params.set("Skew", f"{0.0}")

    proj_params_base = ET.SubElement(proj_params, "ProjectionParametersBase")
    proj_params_base.set("Version", f"{0.2}")

    distortion = ET.SubElement(proj_params, "Distortion")
    distortion.set("k1", f"{0.0}")
    distortion.set("k2", f"{0.0}")
    distortion.set("p1", f"{0.0}")
    distortion.set("p2", f"{0.0}")
    distortion.set("Type", "DISTYPE_DEF")

    child = ET.SubElement(proj_params_base, "Identifier")
    child.set("val", "")
    child = ET.SubElement(proj_params_base, "VideoSourceType")
    child.set("val", "")
    child = ET.SubElement(proj_params_base, "ImageSize")
    child.set("width", f"{w}")
    child.set("height", f"{h}")
    child = ET.SubElement(proj_params_base, "PrincipalPoint")
    child.set("x", f"{0.5 * float(w)}")
    child.set("y", f"{0.5 * float(h)}")
    child = ET.SubElement(proj_params_base, "AspectRatio")
    child.set("val", f"{1.0}")
    child = ET.SubElement(proj_params_base, "Rotation")
    child.text = "0 0 0 1 "
    child = ET.SubElement(proj_params_base, "RMatrix")
    child.text = "1 0 0 0 1 0 0 0 1 "
    child = ET.SubElement(proj_params_base, "Center")
    child.set("x", f"{0.0}")
    child.set("y", f"{0.0}")
    child.set("z", f"{0.0}")

    ET.ElementTree(proj).write(output_path)
    return


def parse_args():
    # General arguments
    parser = argparse.ArgumentParser()
    group = parser.add_argument_group("general")
    group.add_argument(
        "-i", "--image_path", help="path to images", type=str, required=True
    )
    group.add_argument(
        "-o", "--output_path", help="path to output directory", type=str, required=True
    )
    group.add_argument(
        "-e", "--exp_name", help="experiment name", type=str, default="exp0"
    )
    group.add_argument(
        "--gpu_index",
        help="GPU index (-1 for automatic, > 0 to select a specific GPU device)",
        type=int,
        default=-1,
    )
    group.add_argument(
        "--skip_color_normalization",
        help="skip OMV color normalization",
        action="store_true",
        default=False,
    )
    group.add_argument(
        "--skip_features",
        help="skip feature detection and feature matching",
        action="store_true",
        default=False,
    )
    group.add_argument(
        "--skip_sparse",
        help="skip COLMAP sparse reconstruction",
        action="store_true",
        default=False,
    )
    group.add_argument(
        "--skip_dense",
        help="skip OpenMVS dense reconstruction",
        action="store_true",
        default=False,
    )

    # Run OMV's cudaStandaloneStream color normalization tool
    group = parser.add_argument_group("OMV CUDA Color Normalization")
    group.add_argument(
        "--normalize_color",
        help="normalize color of the given images using OMV cudaStandaloneStream",
        action="store_true",
        default=False,
    )
    group.add_argument(
        "--color_norm_red",
        help="color normalization factor for R value",
        type=int,
        default=128,
    )
    group.add_argument(
        "--color_norm_green",
        help="color normalization factor for G value",
        type=int,
        default=128,
    )
    group.add_argument(
        "--color_norm_blue",
        help="color normalization factor for B value",
        type=int,
        default=128,
    )
    group.add_argument(
        "--target_exposure",
        help="target exposure value in double",
        type=float,
        default=0.001,
    )

    # Run COLMAP Sparse reconstruction
    group = parser.add_argument_group("COLMAP Sparse Reconstruction")
    group.add_argument(
        "--camera_model",
        help="camera model for sparse reconstruction",
        choices=CAMERA_MODELS,
        default="METASHAPE_FISHEYE",
    )
    group.add_argument(
        "--camera_params",
        help="camera intrinsic parameters specified as a list of comma-separated parameters",
        type=str,
        default="",
    ),
    group.add_argument(
        "--enable_refraction",
        help="whether to use refractive camera model in reconstruction",
        action="store_true",
        default=False,
    )
    group.add_argument(
        "--camera_refrac_model",
        help="camera refractive model for sparse reconstruction",
        choices=CAMERA_REFRAC_MODELS,
        default="FLATPORT",
    )
    group.add_argument(
        "--camera_refrac_params",
        help="camera refractive parameters specified as a list of comma-separated parameters",
        type=str,
        default="",
    )
    group.add_argument(
        "--max_image_size",
        help="maximum image size for feature extraction, otherwise image will be down-scaled.",
        type=int,
        default=3200,
    )
    group.add_argument(
        "--domain_size_pooling",
        help="whether to use domain size pooling in feature extraction. (This helps to increase number of features, however it is slow as it only runs on CPU)",
        action="store_true",
        default=False,
    )
    group.add_argument(
        "--camera_mask_path",
        help="path to an image file specifying a mask for all images. No features will be extracted in regions where the mask is black (pixel intensity value 0 in grayscale).",
        type=str,
        default="",
    )
    group.add_argument(
        "--use_color_norm_in_features",
        help="use color normalized images for feature extraction. Otherwise use original color images.",
        action="store_true",
        default=False,
    )
    group.add_argument(
        "--fix_intrin",
        help="fix the input intrinsic parameters in the reconstruction",
        action="store_true",
        default=False,
    )
    group.add_argument(
        "--hybrid_mapper",
        help="whether to use hybrid mapper for sparse reconstruction. (Hybrid mapper is a combination of global mapper and incremental mapper)",
        action="store_true",
        default=False,
    )
    group.add_argument(
        "--leaf_max_num_images",
        help="the maximum number of images in a leaf note cluster. This option is only for hybrid mapper",
        type=int,
        default=200,
    )
    group.add_argument(
        "--image_overlap",
        help="the number of overlapping images between child clusters. This option is only for hybrid mapper",
        type=int,
        default=40,
    )
    group.add_argument(
        "--max_num_weak_area_revisit",
        help="the maxinum number of weak area revists",
        type=int,
        default=2,
    )
    group.add_argument(
        "--re_max_num_images",
        help="the maximum number of images for weak area revisit",
        type=int,
        default=30,
    )
    group.add_argument(
        "--re_max_distance",
        help="how large the radius is when selecting a weak area to revisit",
        type=float,
        default=2.5,
    )
    group.add_argument(
        "--pgo_rel_pose_multi",
        help="the multiplier factor for the relative pose term in pose graph optimization",
        type=float,
        default=1.0,
    )
    group.add_argument(
        "--pgo_abs_pose_multi",
        help="the multiplier factor for the absolute pose term in pose graph optimization",
        type=float,
        default=0.001,
    )
    group.add_argument(
        "--pgo_smooth_multi",
        help="the multiplier factor for the motion smoothness term in pose graph optimization",
        type=float,
        default=2.0,
    )
    group.add_argument(
        "--min_num_matches",
        help="the minimum number of matches for inlier matches to be considered.",
        type=int,
        default=15,
    )
    group.add_argument(
        "--ba_local_num_images",
        help="the number of images to optimize in local bundle adjustment",
        type=int,
        default=6,
    )
    group.add_argument(
        "--ba_local_max_num_iterations",
        help="the maximum number of local bundle adjustment iterations",
        type=int,
        default=25,
    )
    group.add_argument(
        "--ba_local_max_refinements",
        help="maximum refinements for local bundle adjustment",
        type=int,
        default=2,
    )
    group.add_argument(
        "--ba_global_max_num_iterations",
        help="the maximum number of global bundle adjustment iterations",
        type=int,
        default=50,
    )
    group.add_argument(
        "--use_pose_prior",
        help="whether to use prior poses in the reconstruction process",
        action="store_true",
        default=False,
    )
    group.add_argument(
        "--pose_prior_path", help="path to pose priors", type=str, default=""
    )
    group.add_argument(
        "--prior_from_cam",
        help="transform from camera to prior if using pose prior in reconstruction",
        type=str,
        default="1, 0, 0, 0, 0, 0, 0",
    )
    group.add_argument(
        "--not-use_global_pose_prior_std",
        help="wheter to use global pose prior weight in bundle adjustment",
        action="store_true",
        default=False,
    )
    group.add_argument(
        "--ba_pose_prior_std",
        help="pose prior standard deviation when using pose prior constraint in bundle adjustment. First 3 components: standard deviation of 3D rotation, Last 3 components: standard deviation of translation in [meter]",
        type=str,
        default="0.1, 0.1, 0.1, 0.1, 0.1, 0.1",
    )
    group.add_argument(
        "--ba_refine_prior_from_cam",
        help="whether to optimize prior_from_cam during bundle adjustment",
        action="store_true",
        default=False,
    )
    group.add_argument(
        "--ba_refine_prior_from_cam_after_num_images",
        help="refine prior_from_cam after registering a certain number of images",
        type=int,
        default=800,
    )
    group.add_argument(
        "--ba_refine_intrin_after_num_images",
        help="refine camera intrinsics after registering a certain number of images",
        type=int,
        default=-1,
    )
    group.add_argument(
        "--write_snapshot",
        help="for every [snapshot_images_freq] it will write out a snapshot of the current reconstruction, useful feature for debugging",
        action="store_true",
        default=False,
    )
    group.add_argument("--snapshot_images_freq", type=int, default=500)

    # Run OpenMVS Dense reconstruction
    group = parser.add_argument_group("OpenMVS Dense Reconstruction")
    group.add_argument(
        "--split_model",
        help="whether to split the model into smaller chunks and reconstruct them one by one",
        action="store_true",
        default=False,
    )
    group.add_argument(
        "--split_type",
        help="how to split the model?",
        type=str,
        choices=["tiles, extent, parts"],
        default="parts",
    )
    group.add_argument(
        "--split_params", help="params to split the model", type=str, default=""
    )
    group.add_argument(
        "--dense_resolution_level",
        help="how many times to scale down the images for mesh texturing",
        type=int,
        default=2,
    )
    group.add_argument(
        "--dense_max_resolution", help="maximum image resolution", type=int, default=800
    )
    group.add_argument(
        "--dense_min_resolution", help="mininum image resolution", type=int, default=640
    )
    group.add_argument(
        "--decimate",
        help="decimate factor in range [0..1] to be applied to the reconstructed surface. (1 - disabled)",
        type=float,
        default=0.7,
    )
    group.add_argument(
        "--max_face_area",
        help="in mesh refinement, maximum face area projected in any pair of images that is not subdivided (0 - disabled)",
        type=int,
        default=64,
    )

    args = parser.parse_args()
    return args


class COLMAPOpenMVSPipeline:
    """Run COLMAP-OpenMVS pipeline"""

    def __init__(self, args):
        # Find executables
        self.colmap_bin = os.path.join(whereis("colmap"), "colmap")
        self.openmvs_bin = whereis("ReconstructMesh")
        self.omv_bin = whereis("cudaStandaloneStream")

        assert self.colmap_bin is not None, "COLMAP not found!"
        assert self.openmvs_bin is not None, "OpenMVS not found!"
        assert self.omv_bin is not None, "OMV not found!"

        self.cmds_logger: List[Tuple[str, List[str]]] = []

        # General options
        self.image_path: Path = Path(args.image_path)
        self.exp_name: str = args.exp_name
        self.output_path: Path = Path(args.output_path)
        self.exp_path: Path = self.output_path / args.exp_name
        self.gpu_index: int = args.gpu_index

        # Skip some of the steps in the pipeline
        self.skip_color_normalization: bool = args.skip_color_normalization
        self.skip_features: bool = args.skip_features
        self.skip_sparse: bool = args.skip_sparse
        self.skip_dense: bool = args.skip_dense

        self.cmds_logger_path: Path = self.exp_path / "cmds.log"

        # Color normalization options
        self.normalize_color: bool = args.normalize_color
        self.color_norm_red: int = args.color_norm_red
        self.color_norm_green: int = args.color_norm_green
        self.color_norm_blue: int = args.color_norm_blue
        self.target_exposure: float = args.target_exposure
        if args.normalize_color:
            self.image_path_color_norm: Path = self.image_path.parent / str(
                self.image_path.stem + "_color_norm/"
            )

        # COLMAP sparse reconstruction options
        self.camera_model: str = args.camera_model
        self.known_intrin: bool = False if args.camera_params == "" else True
        self.camera_params: str = args.camera_params
        self.enable_refraction: bool = args.enable_refraction
        self.camera_refrac_model: str = args.camera_refrac_model
        self.camera_refrac_params: str = args.camera_refrac_params
        self.max_image_size: int = args.max_image_size
        self.domain_size_pooling: bool = args.domain_size_pooling
        self.camera_mask_path: str = args.camera_mask_path
        self.use_color_norm_in_features: bool = args.use_color_norm_in_features

        self.fix_intrin: bool = args.fix_intrin
        self.hybrid_mapper: bool = args.hybrid_mapper
        self.leaf_max_num_images: int = args.leaf_max_num_images
        self.image_overlap: int = args.image_overlap
        self.max_num_weak_area_revisit: int = args.max_num_weak_area_revisit
        self.re_max_num_images: int = args.re_max_num_images
        self.re_max_distance: float = args.re_max_distance
        self.pgo_rel_pose_multi: float = args.pgo_rel_pose_multi
        self.pgo_abs_pose_multi: float = args.pgo_abs_pose_multi
        self.pgo_smooth_multi: float = args.pgo_smooth_multi

        self.min_num_matches: int = args.min_num_matches
        self.ba_local_num_images: int = args.ba_local_num_images
        self.ba_local_max_num_iterations: int = args.ba_local_max_num_iterations
        self.ba_local_max_refinements: int = args.ba_local_max_refinements
        self.ba_global_max_num_iterations: int = args.ba_global_max_num_iterations

        self.use_pose_prior: bool = args.use_pose_prior
        self.pose_prior_path: str = args.pose_prior_path
        self.have_pose_prior: bool = not args.pose_prior_path == ""
        self.prior_from_cam: str = args.prior_from_cam
        self.ba_use_global_pose_prior_std: bool = not args.not_use_global_pose_prior_std
        self.ba_pose_prior_std: str = args.ba_pose_prior_std
        self.refine_prior_from_cam: bool = args.ba_refine_prior_from_cam
        self.refine_prior_from_cam_after: int = (
            args.ba_refine_prior_from_cam_after_num_images
        )
        self.refine_intrin_after: int = args.ba_refine_intrin_after_num_images
        self.write_snapshot: bool = args.write_snapshot
        self.snapshot_images_freq: int = args.snapshot_images_freq

        self.database_path: Path = self.output_path / "database.db"
        self.sparse_path: Path = self.exp_path / "sparse"
        if self.write_snapshot:
            self.snapshot_path: Path = self.exp_path / "snapshots"

        # OpenMVS dense reconstruction options
        self.split_model: bool = args.split_model
        self.split_type: str = args.split_type
        self.split_params: str = args.split_params
        self.resolution_level: int = args.dense_resolution_level
        self.max_resolution: int = args.dense_max_resolution
        self.min_resolution: int = args.dense_min_resolution
        self.decimate: float = args.decimate
        self.max_face_area: int = args.max_face_area

        self.dense_path: Path = self.exp_path / "dense"

    def launch(self):
        self.exp_path.mkdir(parents=True, exist_ok=True)

        if self.normalize_color and not self.skip_color_normalization:
            self._run_color_normalize()

        # Run COLMAP sparse reconstruction
        if self.skip_sparse:
            if not self.skip_dense:
                # Check if there is a sparse reconstruction ready
                have_sparse = detect_model_format(
                    self.sparse_path / "0", ".bin"
                ) or detect_model_format(self.sparse_path / "0", ".txt")
                assert (
                    have_sparse
                ), "ERROR: No sparse reconstruction found! Cannot skip sparse reconstruction"
        else:
            self._run_colmap()

        # Run OpenMVS dense reconstruction
        if not self.skip_dense:
            self._run_openmvs()

        self.write_log()

    def write_log(self):
        with open(self.cmds_logger_path, "w") as file:
            for name, cmd in self.cmds_logger:
                file.write(name + "\n")
                file.write(" ".join(cmd) + "\n")
                file.write("\n")

    def _run_color_normalize(self):
        self.image_path_color_norm.mkdir(parents=True, exist_ok=True)
        # Create a list of images
        image_paths = get_recursive_file_list(self.image_path)
        image_paths = sorted(image_paths)
        _tmp_file = self.image_path / "_tmp_image.lst"

        with open(_tmp_file, "w") as file:
            for p in image_paths:
                file.write(p.as_posix() + "\n")

        # Read the first image to obtain the image size
        img = imageio.imread(image_paths[0])
        H, W, _ = img.shape

        _tmp_cam_file = self.image_path / "_tmp_camera.xml"
        create_omv_camera_xml(W, H, _tmp_cam_file)

        cmds = [
            os.path.join(self.omv_bin, "cudaStandaloneStream"),
            "--SourceImageList",
            _tmp_file.as_posix(),
            "--OutputDir",
            self.image_path_color_norm.as_posix() + "/",
            "--Red",
            f"{self.color_norm_red}",
            "--Green",
            f"{self.color_norm_green}",
            "--Blue",
            f"{self.color_norm_blue}",
            "--SourceCam",
            _tmp_cam_file.as_posix(),
            "--WarpedCam",
            _tmp_cam_file.as_posix(),
            "--TargetExposure",
            f"{self.target_exposure}",
            "--Scatter",
            (self.image_path.parent / "backscatter.png").as_posix(),
        ]

        print_heading1("Running OMV CUDA color normalization")
        exec_cmd(cmds)
        self.cmds_logger.append(("cudaStandaloneStream", cmds))

        _tmp_file.unlink()
        _tmp_cam_file.unlink()

        # Rename and re-arrange resulting images such that they have the same file structure as the original images.
        image_paths_color_norm = get_recursive_file_list(self.image_path_color_norm)
        image_paths_color_norm = sorted(image_paths_color_norm)

        for p in image_paths_color_norm:
            p.replace(p.with_name(p.name.replace("_result", "")))

        image_paths_color_norm = get_recursive_file_list(self.image_path_color_norm)
        image_paths_color_norm = sorted(image_paths_color_norm)

        print(
            f"There are {len(image_paths)} images and {len(image_paths_color_norm)} left after color normalization"
        )

        image_names_color_norm = [item.stem for item in image_paths_color_norm]
        diff = [item for item in image_paths if item.stem not in image_names_color_norm]

        # Remove the head and tail of the original image sequence such that the original images and the color normalized images are consistent
        for p in diff:
            p.unlink()

        # Update the image paths
        image_paths = get_recursive_file_list(self.image_path)
        image_paths = sorted(image_paths)

        for i, path in enumerate(image_paths):
            output_path = Path(self.image_path_color_norm) / Path(
                path.as_posix()[len(self.image_path.as_posix() + "/") :]
            )
            output_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(image_paths_color_norm[i], output_path)

    def _run_colmap(self):
        self.exp_path.mkdir(parents=True, exist_ok=True)

        if self.skip_features:
            assert self.database_path.exists(), "Database doesn't exist!"
        else:
            # Feature extractor
            extractor_cmds = [
                self.colmap_bin,
                "feature_extractor",
                "--database_path",
                self.database_path.as_posix(),
                "--image_path",
                self.image_path_color_norm.as_posix()
                if self.use_color_norm_in_features
                else self.image_path.as_posix(),
                "--camera_mode",
                "1",
                "--ImageReader.camera_model",
                self.camera_model,
                "--SiftExtraction.gpu_index",
                f"{self.gpu_index}",
                "--SiftExtraction.max_image_size",
                f"{self.max_image_size}",
                "--SiftExtraction.domain_size_pooling",
                f"{self.domain_size_pooling}",
                "--ImageReader.camera_mask_path",
                self.camera_mask_path,
            ]

            if self.known_intrin:
                extractor_cmds += ["--ImageReader.camera_params", self.camera_params]

            if self.enable_refraction:
                extractor_cmds += [
                    "--ImageReader.camera_refrac_model",
                    self.camera_refrac_model,
                    "--ImageReader.camera_refrac_params",
                    self.camera_refrac_params,
                ]

            if self.have_pose_prior:
                extractor_cmds += [
                    "--ImageReader.pose_prior_path",
                    self.pose_prior_path,
                ]

            print_heading1("Running feature extraction")
            exec_cmd(extractor_cmds)
            self.cmds_logger.append(("feature_extractor", extractor_cmds))

            # Feature matcher
            matcher_cmds = [
                self.colmap_bin,
                "spatial_matcher" if self.have_pose_prior else "exhaustive_matcher",
                "--database_path",
                self.database_path.as_posix(),
                "--SiftMatching.gpu_index",
                f"{self.gpu_index}",
            ]
            if self.have_pose_prior:
                matcher_cmds += [
                    "--SpatialMatching.is_gps",
                    "0",
                    "--SpatialMatching.ignore_z",
                    "0",
                    "--SpatialMatching.max_num_neighbors",
                    "40",
                    "--SpatialMatching.max_distance",
                    "10",
                ]
            if self.enable_refraction:
                matcher_cmds += ["--TwoViewGeometry.enable_refraction", "1"]

            print_heading1("Running feature matcher")
            exec_cmd(matcher_cmds)
            self.cmds_logger.append(("feature_matcher", matcher_cmds))

        # Run mapper
        self.sparse_path.mkdir(parents=True, exist_ok=True)
        if self.write_snapshot and not self.hybrid_mapper:
            self.snapshot_path.mkdir(parents=True, exist_ok=True)
        mapper_cmds = [
            self.colmap_bin,
            "hybrid_mapper" if self.hybrid_mapper else "mapper",
            "--database_path",
            self.database_path.as_posix(),
            "--image_path",
            self.image_path_color_norm.as_posix()
            if self.use_color_norm_in_features
            else self.image_path.as_posix(),
            "--output_path",
            self.sparse_path.as_posix(),
            "--Mapper.multiple_models",
            "0",
            "--Mapper.min_num_matches",
            f"{self.min_num_matches}",
            "--Mapper.ignore_watermarks",
            "true",
            "--Mapper.abs_pose_min_num_inliers",
            "6",
            "--Mapper.tri_re_max_trials",
            "5",
            "--Mapper.ba_refine_intrin_after_num_images",
            f"{self.refine_intrin_after}",
            "--Mapper.ba_local_num_images",
            f"{self.ba_local_num_images}",
            "--Mapper.ba_local_max_num_iterations",
            f"{self.ba_local_max_num_iterations}",
            "--Mapper.ba_local_max_refinements",
            f"{self.ba_local_max_refinements}",
            "--Mapper.ba_global_max_num_iterations",
            f"{self.ba_global_max_num_iterations}",
        ]
        if self.hybrid_mapper:
            mapper_cmds += [
                "--leaf_max_num_images",
                f"{self.leaf_max_num_images}",
                "--image_overlap",
                f"{self.image_overlap}",
                "--max_num_weak_area_revisit",
                f"{self.max_num_weak_area_revisit}",
                "--re_max_num_images",
                f"{self.re_max_num_images}",
                "--re_max_distance",
                f"{self.re_max_distance}",
                "--pgo_rel_pose_multi",
                f"{self.pgo_rel_pose_multi}",
                "--pgo_abs_pose_multi",
                f"{self.pgo_abs_pose_multi}",
                "--pgo_smooth_multi",
                f"{self.pgo_smooth_multi}",
            ]
        if self.fix_intrin:
            mapper_cmds += [
                "--Mapper.ba_refine_focal_length",
                "0",
                "--Mapper.ba_refine_extra_params",
                "0",
            ]
        else:
            mapper_cmds += ["--Mapper.ba_refine_principal_point", "1"]

        if self.use_pose_prior:
            mapper_cmds += [
                "--Mapper.use_pose_prior",
                "1",
                "--Mapper.prior_from_cam",
                self.prior_from_cam,
                "--Mapper.ba_use_global_pose_prior_std",
                f"{self.ba_use_global_pose_prior_std}",
                "--Mapper.ba_pose_prior_std",
                self.ba_pose_prior_std,
            ]
            if self.refine_prior_from_cam:
                mapper_cmds += [
                    "--Mapper.ba_refine_prior_from_cam",
                    "1",
                    "--Mapper.ba_refine_prior_from_cam_after_num_images",
                    f"{self.refine_prior_from_cam_after}",
                ]

        if self.enable_refraction:
            mapper_cmds += ["--Mapper.enable_refraction", "1"]

        if self.write_snapshot and not self.hybrid_mapper:
            mapper_cmds += [
                "--Mapper.snapshot_path",
                self.snapshot_path.as_posix(),
                "--Mapper.snapshot_images_freq",
                f"{self.snapshot_images_freq}",
            ]

        print_heading1("Running mapper")
        exec_cmd(mapper_cmds)
        self.cmds_logger.append(("mapper", mapper_cmds))

    def _run_openmvs(self):
        self.dense_path.mkdir(parents=True, exist_ok=True)

        if self.split_model:
            split_cmds = [
                self.colmap_bin,
                "model_splitter",
                "--input_path",
                (self.sparse_path / "0").as_posix(),
                "--output_path",
                self.dense_path.as_posix(),
                "--split_type",
                self.split_type,
                "--split_params",
                self.split_params,
                "--overlap_ratio",
                "0.2",
            ]
            print_heading1("Running model splitter")
            exec_cmd(split_cmds)
            self.cmds_logger.append(("model_splitter", split_cmds))

            for chunk in self.dense_path.glob("*/"):
                (chunk / "mvs").mkdir(parents=True, exist_ok=True)
                undistort_cmds = [
                    self.colmap_bin,
                    "image_undistorter",
                    "--image_path",
                    self.image_path_color_norm.as_posix()
                    if self.normalize_color
                    else self.image_path.as_posix(),
                    "--input_path",
                    chunk.as_posix(),
                    "--output_path",
                    (chunk / "mvs" / "undistortion").as_posix(),
                    "--min_scale",
                    "0.8",
                    "--max_scale",
                    "1.2",
                ]
                print_heading1(f"Run image undistortion for chunk {chunk.name}")
                exec_cmd(undistort_cmds)
                self.cmds_logger.append(
                    (f"image_undistorter_{chunk.name}", undistort_cmds)
                )

        else:
            # Do not split model.
            (self.dense_path / "0" / "mvs").mkdir(parents=True, exist_ok=True)
            undistort_cmds = [
                self.colmap_bin,
                "image_undistorter",
                "--image_path",
                self.image_path_color_norm.as_posix()
                if self.normalize_color
                else self.image_path.as_posix(),
                "--input_path",
                (self.sparse_path / "0").as_posix(),
                "--output_path",
                (self.dense_path / "0" / "mvs" / "undistortion").as_posix(),
                "--min_scale",
                "0.8",
                "--max_scale",
                "1.2",
            ]
            print_heading1("Running image undistortion for chunk 0")
            exec_cmd(undistort_cmds)
            self.cmds_logger.append(("image_undistorter_0", undistort_cmds))

        chunks = list(self.dense_path.glob("*/"))
        chunks = sorted(chunks)
        chunk_ids = [int(chunk.name) for chunk in chunks]
        mvs_paths = [chunk / "mvs" for chunk in chunks]

        # Run InterfaceCOLMAP
        for i, chunk in enumerate(chunks):
            cmds = [
                os.path.join(self.openmvs_bin, "InterfaceCOLMAP"),
                "--input-file",
                (mvs_paths[i] / "undistortion").as_posix(),
                "--output-file",
                (mvs_paths[i] / "scene.mvs").as_posix(),
                "--working-folder",
                (mvs_paths[i] / "undistortion").as_posix(),
            ]
            print_heading1(f"Running InterfaceCOLMAP for chunk {chunk_ids[i]}")
            exec_cmd(cmds)
            self.cmds_logger.append((f"interface_colmap_{chunk_ids[i]}", cmds))

        # Run DensifyPointCloud
        for i, chunk in enumerate(chunks):
            cmds = [
                os.path.join(self.openmvs_bin, "DensifyPointCloud"),
                "--input-file",
                "scene.mvs",
                "--output-file",
                "dense.mvs",
                "--working-folder",
                mvs_paths[i].as_posix(),
                "--cuda-device",
                f"{self.gpu_index}",
                "--resolution-level",
                f"{self.resolution_level}",
                "--max-resolution",
                f"{self.max_resolution}",
                "--min-resolution",
                f"{self.min_resolution}",
            ]
            print_heading1(f"Running DensifyPointcloud for chunk {chunk_ids[i]}")
            exec_cmd(cmds)
            self.cmds_logger.append((f"densifypointcloud_{chunk_ids[i]}", cmds))

        # Run ReconstructMesh
        for i, chunk in enumerate(chunks):
            cmds = [
                os.path.join(self.openmvs_bin, "ReconstructMesh"),
                "--input-file",
                "dense.mvs",
                "--output-file",
                "mesh.mvs",
                "--working-folder",
                mvs_paths[i].as_posix(),
                "--cuda-device",
                f"{self.gpu_index}",
                # options to clean the mesh
                "--decimate",
                f"{self.decimate}",
                "--remove-spurious",
                "30",
                "--close-holes",
                "100",
            ]
            print_heading1(f"Running ReconstructMesh for chunk {chunk_ids[i]}")
            exec_cmd(cmds)
            self.cmds_logger.append((f"reconstruct_mesh_{chunk_ids[i]}", cmds))

        # Run RefineMesh
        for i, chunk in enumerate(chunks):
            cmds = [
                os.path.join(self.openmvs_bin, "RefineMesh"),
                "--input-file",
                "mesh.mvs",
                "--output-file",
                "mesh.mvs",
                "--working-folder",
                mvs_paths[i].as_posix(),
                "--cuda-device",
                f"{self.gpu_index}",
                "--resolution-level",
                f"{self.resolution_level}",
                "--min-resolution",
                f"{self.min_resolution}",
                "--max-face-area",
                f"{self.max_face_area}",
                "--scales",
                "5",
            ]
            print_heading1(f"Running RefineMesh for chunk {chunk_ids[i]}")
            exec_cmd(cmds)
            self.cmds_logger.append((f"refine_mesh_{chunk_ids[i]}", cmds))

        # Run TextureMesh
        for i, chunk in enumerate(chunks):
            cmds = [
                os.path.join(self.openmvs_bin, "TextureMesh"),
                "--input-file",
                "mesh.mvs",
                "--output-file",
                f"mesh_texture_chunk_{chunk_ids[i]}.mvs",
                "--working-folder",
                mvs_paths[i].as_posix(),
                "--cuda-device",
                f"{self.gpu_index}",
                "--resolution-level",
                f"{self.resolution_level}",
                "--min-resolution",
                f"{self.min_resolution}",
                "--export-type",
                "obj",
            ]
            print_heading1(f"Running TextureMesh for chunk {chunk_ids[i]}")
            exec_cmd(cmds)
            self.cmds_logger.append((f"texture_mesh_{chunk_ids[i]}", cmds))

        # Move all mesh to `dense_path`
        for i, chunk in enumerate(chunks):
            prefix = "mesh"
            postfix = f"_chunk_{chunk_ids[i]}"
            mesh_path = prefix + "_texture" + postfix + ".obj"
            texture_path = prefix + "_texture" + postfix + "_material_0_map_Kd" + ".jpg"
            material_path = prefix + "_texture" + postfix + ".mtl"
            if (
                not (mvs_paths[i] / mesh_path).exists()
                or not (mvs_paths[i] / texture_path).exists()
                or not (mvs_paths[i] / material_path).exists()
            ):
                continue
            shutil.move(mvs_paths[i] / mesh_path, self.dense_path / mesh_path)
            shutil.move(mvs_paths[i] / texture_path, self.dense_path / texture_path)
            shutil.move(mvs_paths[i] / material_path, self.dense_path / material_path)


def main():
    args = parse_args()

    pipeline = COLMAPOpenMVSPipeline(args)
    pipeline.launch()


if __name__ == "__main__":
    main()
