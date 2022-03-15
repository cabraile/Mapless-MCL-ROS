#!/usr/bin/python3
import os
import argparse
import sys

def parse_args() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset_dir", required=True)
    parser.add_argument("--start_at", required=True)
    parser.add_argument("--pause", action="store_true")
    return parser.parse_known_args()[0]

def main( ) -> int:
    args = parse_args()
    dataset_dir = os.path.abspath(args.dataset_dir)
    parameters = {
        "clock" : True,
        "start" : args.start_at,
        "pause" : args.pause
    }

    # Parameters
    cmd_params_str_list = []
    for key, value in parameters.items():
        if type(value) == bool:
            if value == False:
                continue
            param_str = "--" + key
        else:
            param_str = "--" + key + " " + str(value) 
        cmd_params_str_list.append(param_str)

    cmd_params_str = ""
    for param_str in cmd_params_str_list:
        cmd_params_str += " " + param_str

    # Bag files
    files_names = os.listdir(dataset_dir)
    cmd_files_str = ""
    for file_name in files_names:
        ext = os.path.splitext( os.path.basename(file_name) )[-1]
        if ext != ".bag":
            continue
        cmd_files_str += " " + file_name

    # Full cmd
    cwd = os.getcwd()
    os.chdir(dataset_dir)
    cmd_str = "rosbag play"
    cmd_full = cmd_str + " " + cmd_params_str + " " + cmd_files_str
    try:
        os.system(cmd_full)
    finally:
        os.chdir(cwd)

if __name__ == "__main__":
    sys.exit(main())