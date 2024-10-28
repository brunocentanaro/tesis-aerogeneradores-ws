import os
import json
import numpy as np
import sys


def process_json_files(folder_path):
    medians = []
    standard_deviations = []
    all_position_errors = []
    all_durations = []
    all_sim_durations = []

    script_dir = os.path.dirname(os.path.abspath(__file__))
    folder_path = os.path.join(script_dir, folder_path)

    for filename in os.listdir(folder_path):
        if filename.endswith('.json'):
            file_path = os.path.join(folder_path, filename)
            try:
                with open(file_path, 'r') as file:
                    data = json.load(file)
                    position_errors = data.get('positionErrors', [])
                    duration = data.get('duration', 0)
                    sim_duration = data.get('simDuration', 0)
                    if duration:
                        all_durations.append(duration)
                    if sim_duration:
                        all_sim_durations.append(sim_duration)
                    if position_errors:
                        median = np.median(position_errors)
                        std_dev = np.std(position_errors)
                        medians.append(median)
                        standard_deviations.append(std_dev)
                        all_position_errors.extend(position_errors)
                    else:
                        print(
                            f"The file {filename} does not contain 'positionErrors' or it is empty.\n")
            except json.JSONDecodeError as e:
                print(f"Error decoding JSON in file {filename}: {e}")

    if all_position_errors:
        total_median = np.median(all_position_errors)
        total_std_dev = np.std(all_position_errors)
        print("Combined Results from All Files:")
        print(f"Total Median: {total_median}")
        print(f"Total Standard Deviation: {total_std_dev}\n")
        print(f"Total Duration: {sum(all_durations)}")
        print(f"Total Duration Median: {np.median(all_durations)}")
        print(f"Total standard deviation of durations: {np.std(all_durations)}\n")
        print(f"Total Sim Duration: {sum(all_sim_durations)}")
        print(f"Total Sim Duration Median: {np.median(all_sim_durations)}")
        print(f"Total standard deviation of Sim Durations: {np.std(all_sim_durations)}\n")
    else:
        print("No 'positionErrors' data found in any file.")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        folder_path = sys.argv[1]
    else:
        raise ValueError("Please provide the folder path as an argument.")
    process_json_files(folder_path)