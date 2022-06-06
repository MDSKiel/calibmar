import argparse
import subprocess
import os
from pathlib import Path

def exec_cmd(cmd):
    print('Running command: ' + " ".join(cmd))
    subprocess.run(cmd)

def parse_args():
    parser = argparse.ArgumentParser(description = "Texture reconstruction using OpenMVS")
    parser.add_argument("--output_name", required=True, help = "output model name")
    args = parser.parse_args()
    return args

def openmvs_texture_reconstruction(args):
    output_dir = "openmvs/"
    mvs_format_path = os.path.join(output_dir, args.output_name + "_mvs.mvs")
    Path(output_dir).mkdir(exist_ok = True, parents=True)
    
    # Convert COLMAP dense model into OpenMVS format
    cmd = [
        "InterfaceCOLMAP",
        "-i",
        ".",
        "-o",
        mvs_format_path,
        "-v",
        "2"
    ]
    exec_cmd(cmd)

    # Reconstruct mesh
    mesh_path = os.path.join(output_dir, args.output_name + "_mesh.mvs")
    cmd = [
        "ReconstructMesh",
        "-i",
        mvs_format_path,
        "-o",
        mesh_path,
        "-v",
        "2"
    ]
    exec_cmd(cmd)

    # Refine mesh
    cmd = [
        "RefineMesh",
        "-i",
        mesh_path,
        "-o",
        mesh_path,
        "--max-face-area",
        "16",
        "-v",
        "2"
    ]
    exec_cmd(cmd)

    # Texture mesh

    cmd = [
        "TextureMesh",
        "-i",
        mesh_path,
        "-o",
        mesh_path,
        "--export-type",
        "obj"
    ]
    exec_cmd(cmd)

    # Clean logs
    os.system("rm -r ./*.log")

def main():
    args = parse_args()

    # OpenMVS texture reconstruction.
    openmvs_texture_reconstruction(args)
    return

    
    

if __name__ == "__main__":
    main()