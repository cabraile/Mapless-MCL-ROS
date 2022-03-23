**This README is still under progress.**

![An illustration of the method in a localization pipeline.](resources/hero.png)

Mapless Monte-Carlo Localization
=============
Tired of having to remap your city streets in 3D for running your autonomous-driving experiments? HD maps are too expensive for your team? This might be a project for you!

This repository contains the implementation of the Mapless Monte-Carlo Localization (or MMCL for short), in which no prior mapping step using an expensive sensor suite is required for localization. In fact, the maps used in this project can be downloaded using the [OpenStreetMap](https://www.openstreetmap.org/) database.

For more details on the method, check out our [paper](resources/paper.pdf), which was submitted to the IV2022 conference and still under review process.

Setup
=============

Ubuntu
-------

If you intend on running this project in a docker container, skip to the docker section. On the other hand, you can still peek the dockerfiles (under `./docker`) for guidance on the setup.


**Make sure you have ROS1 installed**
    - Install ROS1 in your machine - we recommend installing the [Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) version.
**Prepare your catkin workspace**
    - `cd` to the directory from which you will use as a catkin workspace.
- **Python 3**.
- `darknet_ros`. Clone their 

**Optional (for running the demonstrations)**:
    - Download the ROS bag files [here](https://drive.google.com/drive/folders/19K-1EjE-EJwqM4iHRPnX-oLn--NlU0lt?usp=sharing);
    - Download the road map file [here](https://drive.google.com/file/d/1BPNlTLTExGXqM3NVAV280eHUFdWGb9p0/view?usp=sharing);
    - Download the trajectory file [here](https://drive.google.com/file/d/12sEUPd4Ntv2hiyNpk6SLxLV2rPuJuXH6/view?usp=sharing);
    - Download the `split_rectify_stereo` package [here](https://drive.google.com/file/d/1cig26bATuz5g-EIiUT-YCdjLovWB4RuB/view?usp=sharing).


Docker
-------
We prepared a docker machine for running this project's  nodes in case you do not want to set your whole machine up.

One liner image build command:
```bash
docker build -t maples_mcl_ros:latest -f docker/demo/Dockerfile .
```
Or, instead, if you want to include the demo files:
```bash
docker build -t maples_mcl_ros:latest -f docker/demo/Dockerfile .  --build-arg INCLUDE_DEMO=true
```

How to use
=============

Downloading the map
-------------

Preparing the trajectories and road elements
-------------

Setting up the launch files
-------------

Run
-------------

Demos
=============
