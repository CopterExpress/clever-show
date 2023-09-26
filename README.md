# clever-show

[Русская версия](README_RU.md)

Software for making the drone show with drones controlled by [Raspberry Pi](https://www.raspberrypi.org/) with COEX [Clover](https://github.com/CopterExpress/clover) package and flight controller with [PX4](https://github.com/PX4/Firmware) firmware.

Create animation in [Blender](https://www.blender.org/), convert it to drone paths, set up the drones and run your own show!

## Demo video

[![Autonomous drone show in a theater](http://img.youtube.com/vi/HdHbZFz7nR0/0.jpg)](http://www.youtube.com/watch?v=HdHbZFz7nR0)

12 drones perform in a show in Electrotheatre Stanislavsky, Moscow.

## This software includes

* [Drone side](drone/) for remote synchronized control of drones with emergency drone protection module
* [Server side](server/) for making the drone show with ability of tuning drones, animation and music
* [Blender 2.8 addon](blender-addon/) for exporting animation to drone paths
* [Raspberry Pi image](https://github.com/CopterExpress/clever-show/releases/latest) for quick launch software on the drones
* [Documentation](docs/en/SUMMARY.md)

## Positioning systems supported

* [Optical flow](https://clover.coex.tech/en/optical_flow.html) (indoor)
* [ArUco map-based](https://clover.coex.tech/en/aruco_map.html) (indoor)
* [GPS](https://clover.coex.tech/en/gps.html) (outdoor)

## Quick start

Start making your own show using [this tutorial](docs/en/start-tutorial.md)!
