import argparse
import os
import numpy as np
from pathlib import Path
import csv
import re
from datetime import datetime
from tqdm import tqdm

def get_recursive_file_list(path: Path):
    lst = []
    for p in path.rglob("*"):
        if p.is_file():
            lst.append(p)
    return lst

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-i",
        "--image_path",
        help="path to the image directory",
        type=str,
        required=True,
    )
    parser.add_argument(
        "-n",
        "--navigtation_path",
        help="path to the raw navigation data",
        type=str,
        required=True,
    )
    parser.add_argument(
        "-o",
        "--output_path",
        help="path to the output directory",
        type=str,
        required=True,
    )
    args = parser.parse_args()
    return args


def extract_timestamp(filename):
    timestr = re.search('\d{8}_\d{6}.\d{3}', filename).group()
    time = datetime.strptime(timestr, "%Y%m%d_%H%M%S.%f")
    return time


def closest_time(target, times):
    time_diff = np.abs([target - time for time in times])
    min_time_idx = time_diff.argmin(0)
    return min_time_idx, times[min_time_idx]


def main():
    args = parse_args()
    csv_data = []
    headers = [
        "DateTime [%Y-%m-%d %H:%M:%S.%f]",
        "Latitude [deg]",
        "Longitude [deg]",
        "Altitude [m]",
        "Depth [m]",
        "North SD [m]",
        "East SD [m]",
        "Depth SD [m]",
        "Yaw [rad]",
        "Pitch [rad]",
        "Roll [rad]",
        "Yaw SD [rad]",
        "Pitch SD [rad]",
        "Roll SD [rad]",
    ]
    print("Creating one single csv file for each image")
    os.makedirs(args.output_path, exist_ok=True)

    with open(args.navigtation_path, "r") as nav_file:
        csv_reader = csv.DictReader(nav_file)
        _ = next(csv_reader)
        for row in csv_reader:
            extracted_fields = [row[key] for key in headers]
            extracted_fields[1:] = [
                float(extracted_fields[i]) for i in range(1, len(extracted_fields))
            ]
            csv_data.append(extracted_fields)
    
    # Create a recursive file list 
    image_path = Path(args.image_path)
    image_paths = get_recursive_file_list(image_path)
    image_paths = sorted(image_paths)
    pos = 0
    step = 100

    for path in tqdm(image_paths):
        csv_path = path.with_suffix(".csv")
        csv_path = csv_path.as_posix()[len(image_path.as_posix() + "/"):]
        csv_path = Path(args.output_path) / Path(csv_path)
        os.makedirs(csv_path.parent, exist_ok=True)
        time = extract_timestamp(path.stem)
        search_time_lst = [
            datetime.strptime(csv_data[i][0], "%Y-%m-%d %H:%M:%S.%f")
            for i in range(pos, min(pos + step, len(csv_data)))
        ]
        min_time_idx, min_time = closest_time(time, search_time_lst)
        pos += min_time_idx
        with open(csv_path, "w") as file:
            csv_writer = csv.writer(file)
            csv_writer.writerow(headers)
            csv_writer.writerow(csv_data[pos])


if __name__ == "__main__":
    main()
