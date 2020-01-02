# Scripts for building Raspberry Pi image

This directory contains scripts for automated image building via travis-ci.org.

You can place the folders with the `clever` settings files (`launch`,`map` and `camera_info`) to the folder `clever-config` located in this directory. Then you can build your image with custom drone settings locally.

* All files from the `launch` folder will be copied to the `/home/pi/catkin_ws/src/clever/clever/launch` directory in the assembled image.
* All files from the `map` folder will be copied to the `/home/pi/catkin_ws/src/clever/aruco_pose/map` directory in the assembled image.
* All files from the `camera_info` folder will be copied to the `/home/pi/catkin_ws/src/clever/clever/camera_info` directory in the assembled image.

Install docker if needed:

```bash
sudo apt install docker.io
```

Build your custom image with docker:

```bash
cd source-dir
sudo docker run --privileged -it --rm -v /dev:/dev -v $(pwd):/mnt goldarte/img-tool:v0.5
```

The image will be located in `images` directory in the root of clever-show source code directory.

Full documentation about building custom image is located here:
* English
* [Russian](../docs/ru/image-building.md)
