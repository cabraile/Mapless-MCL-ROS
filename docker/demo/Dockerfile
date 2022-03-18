FROM ros:noetic-ros-base-focal

ARG DEBIAN_FRONTEND=noninteractive
ENV CATKIN_WS /workspace

# APT dependencies
RUN apt update && apt install -y --no-install-recommends\
    git \
    python3 \
    python3-pip \
    python-is-python3 \
    ros-noetic-catkin \
    ros-noetic-rtabmap-ros \
    ros-noetic-stereo-image-proc \
    ros-noetic-robot-localization \
    unzip && rm -rf /var/lib/apt/lists/*

# Intermediate tools
# TODO: 'gdown' only for demo
RUN pip3 install -U gdown

RUN mkdir -p ${CATKIN_WS}/src && mkdir -p ${CATKIN_WS}/src/dependencies

# Install the mapless_mcl_py project
COPY . ${CATKIN_WS}/src/mapless_mcl_ros
WORKDIR /${CATKIN_WS}/src/mapless_mcl_ros/mapless_mcl_py
RUN pip3 install -e .

# Download the other dependencies
WORKDIR ${CATKIN_WS}/src/dependencies

# - Darknet ROS
RUN git clone --recursive https://github.com/leggedrobotics/darknet_ros

# - Split rectify (for running the Carina demo)
# TODO: INSPECT. Somehow the file is not being downloaded.
RUN gdown https://drive.google.com/uc?id=1cig26bATuz5g-EIiUT-YCdjLovWB4RuB && \
    unzip split_rectify_stereo.zip && \
    rm split_rectify_stereo.zip

# Build all ROS packages
WORKDIR ${CATKIN_WS}
RUN catkin_make_isolated -DCMAKE_BUILD_TYPE=Release
RUN echo 'source "${CATKIN_WS}/install/setup.bash"' >> ~/.bashrc