#!/usr/bin/env python3

# NOTE: please use only standard libraries
import argparse
import subprocess
from pathlib import Path

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-c", type=str, help="gripper config file"
    )
    parser.add_argument(
        "-h", type=str, default="localhost", help="host name or ip-address"
    )
    args = parser.parse_args()

    assert args.c is not None
    path = Path(args.c)
    config_file = path.parts[-1]
    config_path = path.parent.absolute()

    docker_run_command = """
        docker run \
            --rm --net=host -it \
            -v {config_path}:/mnt
            gpg_ros:latest \
            /bin/bash -i -c \
            "source ~/.bashrc; \
            roscd gpg_ros; \
            rossetip $ROS_IP; rossetmaster {host}; \
            roslaunch gpg_ros grasp_server.launch config:=/mnt/{config_file}
            """.format(
        host=args.h,
        config_path=config_path,
        config_file=config_file,
    )
    print(docker_run_command)
    subprocess.call(docker_run_command, shell=True)
