import argparse
import os
import numpy as np
from colorama import Fore, Style
from tabulate import tabulate
from evo.core import trajectory, sync, metrics, geometry
from evo.tools import file_interface
from evo.core.trajectory import PoseTrajectory3D

def check_tum_format(file_path):
    with open(file_path, 'r') as f:
        for line in f:
            entries = line.strip().split()
            if len(entries) != 8:
                print(f"Invalid line in {file_path}: {line.strip()}")
                return False
    return True

def read_reference_transform(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
        # print("File content:")
        # for i, line in enumerate(lines):
        #     print(f"Line {i}: {line.strip()}")
        rotation_matrix = np.array([
            [float(value) for value in lines[3].strip().split()],
            [float(value) for value in lines[4].strip().split()],
            [float(value) for value in lines[5].strip().split()]
        ])
        translation_vector = np.array([
            float(lines[7].strip()),
            float(lines[8].strip()),
            float(lines[9].strip())
        ])
        ref_total_time = float(lines[-1].strip().split(":")[1].replace("s", "").strip())
    return rotation_matrix, translation_vector, ref_total_time

def calculate_total_time(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
        if not lines:
            return 0
        first_timestamp = float(lines[0].strip().split()[0])
        last_timestamp = float(lines[-1].strip().split()[0])
        return last_timestamp - first_timestamp

def main():
    parser = argparse.ArgumentParser(description="Evaluate APE of trajectories.")
    parser.add_argument("reference_file", help="Path to the reference trajectory file.")
    parser.add_argument("evaluation_dir", help="Path to the directory or file containing evaluation trajectories.")
    parser.add_argument("-t", action="store_true", help="Sort by ATE Translation(m)")
    parser.add_argument("-r", action="store_true", help="Sort by ATE Rotation")
    parser.add_argument("-a", action="store_true", help="Sort by ATE RMSE")
    args = parser.parse_args()

    reference_file = args.reference_file
    evaluation_dir = args.evaluation_dir

    if not os.path.exists(reference_file):
        print(f"Reference file not found: {reference_file}")
        return

    if not os.path.exists(evaluation_dir):
        print(f"Evaluation directory or file not found: {evaluation_dir}")
        return

    print("loading reference transform...")
    print()
    try:
        ref_rotation, ref_translation, ref_total_time = read_reference_transform(reference_file)
        reference_file_name = os.path.basename(reference_file)
        # print("GT Pose transformation:")
        print(f"{Fore.GREEN}●  M3DGR ●{Style.RESET_ALL}  {reference_file_name.split('.')[0]} Pose transformation:")
        print("• Rotation:\n", ref_rotation)
        print("• Translation:\n", ref_translation)
        print(f"• Total_time: {ref_total_time:.2f} s")
        print()
    except Exception as e:
        print(f"Error reading reference transform: {e}")
        return

    traj_est_files = []
    if os.path.isdir(evaluation_dir):
        for root, _, files in os.walk(evaluation_dir):
            for file in files:
                if file.endswith(".txt") or file.endswith(".tum"):
                    traj_est_files.append(os.path.join(root, file))
    elif os.path.isfile(evaluation_dir):
        traj_est_files = [evaluation_dir]

    # for file in traj_est_files:
    #     print(file)

    results = []

    print("Generate calculation table...")

    for traj_est_file in traj_est_files:
        if not os.path.exists(traj_est_file):
            print(f"File not found: {traj_est_file}, skipping.")
            continue

        if not check_tum_format(traj_est_file):
            print(f"Skipping invalid TUM file: {traj_est_file}")
            continue

        try:
            traj_est = file_interface.read_tum_trajectory_file(traj_est_file)
        except Exception as e:
            print(f"Error reading trajectory file {traj_est_file}: {e}")
            continue

        last_pose_index = len(traj_est.poses_se3) - 1
        for i in range(len(traj_est.poses_se3) - 2, -1, -1):
            if not np.allclose(traj_est.poses_se3[i], traj_est.poses_se3[last_pose_index]):
                break
            last_pose_index = i

        try:
            est_first_pose = traj_est.poses_se3[0]
            est_last_pose = traj_est.poses_se3[last_pose_index]
            est_relative_transform = np.linalg.inv(est_first_pose) @ est_last_pose
            est_rotation = est_relative_transform[:3, :3]
            est_translation = est_relative_transform[:3, 3]

            rotation_error = np.linalg.norm(ref_rotation - est_rotation)
            translation_error = np.linalg.norm(ref_translation - est_translation)
            rmse_error = np.sqrt((rotation_error**2 + translation_error**2) / 2)

            # eval_total_time = calculate_total_time(traj_est_file)

            start_time = traj_est.timestamps[0]
            end_time = traj_est.timestamps[last_pose_index]
            eval_total_time = end_time - start_time

            tracking_rate = (eval_total_time / ref_total_time) * 100

            if tracking_rate > 100:
                 tracking_rate = 100

            results.append([
                traj_est_file, 
                f"{translation_error:.2f}", 
                f"{rotation_error:.2f}", 
                f"{rmse_error:.2f}", 
                f"{tracking_rate:.2f}%"
            ])
        except Exception as e:
            print(f"Error calculating relative transform for {traj_est_file}: {e}")
    
    if args.t:
        results.sort(key=lambda x: float(x[1]))
    elif args.r:
        results.sort(key=lambda x: float(x[2]))
    elif args.a:
        results.sort(key=lambda x: float(x[3]))
    else:
        results.sort(key=lambda x: float(x[1]))
    
    results_with_index = [[str(i + 1)] + row for i, row in enumerate(results)]
    
    # results_with_index = [[f"{Fore.RED}{str(i + 1)}{Style.RESET_ALL}"] + row for i, row in enumerate(results)]
    
    headers = [f"{Fore.RED}Index{Style.RESET_ALL}", 
           f"{Fore.RED}File{Style.RESET_ALL}", 
           f"{Fore.RED}ATE Translation(m){Style.RESET_ALL}", 
           f"{Fore.RED}ATE Rotation{Style.RESET_ALL}", 
           f"{Fore.RED}ATE RMSE{Style.RESET_ALL}", 
           f"{Fore.RED}Tracking Rate (%){Style.RESET_ALL}"]

    print(tabulate(results_with_index, headers=headers, tablefmt="grid"))
    
    print(f"{Fore.GREEN}-------------Thanks for using the {Style.BRIGHT}M3DGR{Style.NORMAL} 🦄 dataset.-----------{Style.RESET_ALL}")
if __name__ == "__main__":
    main()