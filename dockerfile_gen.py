import argparse
import sys
import subprocess
import pathlib
import os

ENTRYPOINT_FILE_TEXT="""#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/noetic/setup.bash"
source "${CATKIN_WS}/devel/setup.bash
exec "$@"
"""

BASE_TEXT = """
FROM ros:noetic-ros-base-focal

ARG DEBIAN_FRONTEND=noninteractive
ENV CATKIN_WS /workspace

# APT dependencies
RUN apt update && apt install -y --no-install-recommends\
    build-essential \
    git \
    python3 \
    python3-pip \
    python-is-python3 \
    ros-noetic-catkin \
    ros-noetic-rtabmap-ros \
    ros-noetic-stereo-image-proc \
    ros-noetic-robot-localization \
    ros-noetic-camera-info-manager \
    ros-noetic-ros-numpy \
    wget \
    unzip && rm -rf /var/lib/apt/lists/*

RUN mkdir -p ${CATKIN_WS}/src ${CATKIN_WS}/src/dependencies /data /bags

# Install the mapless_mcl_py project
COPY . ${CATKIN_WS}/src/mapless_mcl_ros
WORKDIR ${CATKIN_WS}/src/mapless_mcl_ros/mapless_mcl_py
RUN pip3 install -e . && \
    pip3 install -r ${CATKIN_WS}/src/mapless_mcl_ros/mapless_mcl_ros_demos/requirements.txt

# Download the other dependencies
WORKDIR ${CATKIN_WS}/src/dependencies

RUN pip3 install -U catkin-tools

# - Darknet ROS
RUN git clone --recursive https://github.com/leggedrobotics/darknet_ros
"""

DEMO_TEXT = """
# - Split rectify (for running the Carina demo)
# Intermediate tools
# TODO: 'gdown' only for demo
RUN pip3 install -U gdown && \
    gdown https://drive.google.com/uc?id=1cig26bATuz5g-EIiUT-YCdjLovWB4RuB && \
    unzip split_rectify_stereo.zip && \
    rm split_rectify_stereo.zip
"""

CBUILD_TEXT ="""
# Build all ROS packages
WORKDIR ${CATKIN_WS}

RUN catkin config --extend /opt/ros/noetic && catkin init && catkin build -DCMAKE_BUILD_TYPE=Release
RUN echo 'source "${CATKIN_WS}/devel/setup.bash"' >> ~/.bashrc
"""

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image_name", default="mapless_mcl_ros:latest")
    parser.add_argument("--include_demo", action="store_true", help="If provided, include the demo-related dependencies.")
    parser.add_argument("--build", action="store_true", help="If provided, this script will build the generated dockerfile.")
    return parser.parse_args()

def main() -> int :
    args = parse_args()

    # Write the temporary dockerfile
    project_dir = pathlib.Path(__file__).parent.resolve()
    if args.include_demo:
        dockerfile_dir = os.path.join(project_dir, "docker", "demo")
    else:
        dockerfile_dir = os.path.join(project_dir, "docker", "base")

    dockerfile_path = os.path.join(dockerfile_dir, "dockerfile")
    with open(dockerfile_path, "w") as dockerfile:
        dockerfile.write(BASE_TEXT)
        if args.include_demo:
            dockerfile.write(DEMO_TEXT)
        dockerfile.write(CBUILD_TEXT)
    
    # Build the docker file
    if args.build:

        ret = subprocess.run( ["docker","build","-t", args.image_name, "-f", dockerfile_path, "."] )
        if ret.returncode != 0:
            print("The build process ended with an error")

    return 0

if __name__=="__main__":
    sys.exit(main())

