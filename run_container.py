#!/usr/bin/env python3

# NOTE: please use only standard libraries
import os
import argparse
import subprocess
from pathlib import Path

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-c", type=str, help="gripper config file"
    )
    parser.add_argument(
        "-host", type=str, default="localhost", help="host name or ip-address"
    )
    parser.add_argument(
        "launch_args",
        nargs=argparse.REMAINDER,
        help="launch args in ros style e.g. foo:=var",
    )
    args = parser.parse_args()

    assert args.c is not None
    path = Path(args.c)
    config_file = path.parts[-1]
    config_path = path.parent.absolute()

    docker_run_command = """
        docker run \
            --rm --net=host -it \
            -v {config_path}:/mnt \
            gpg_ros:latest \
            /bin/bash -i -c \
            "source ~/.bashrc; \
            roscd gpg_ros; \
            export ROS_IP={ip}; export ROS_MASTER={host}; export ROS_MASTER_URI=http://{host}:11311; \
            roslaunch gpg_ros grasp_server.launch config:=/mnt/{config_file}"
            """.format(
        ip=os.environ['ROS_IP'] if 'ROS_IP' in os.environ else '127.0.0.1',
        host=args.host,
        config_path=config_path,
        config_file=config_file,
    )
    print(docker_run_command)
    subprocess.call(docker_run_command, shell=True)
