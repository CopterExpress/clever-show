# clever-show

[Русская версия](README_RU.md)

Software for making the drone show controlled by Raspberry Pi, PX4 and COEX [Clever](https://github.com/CopterExpress/clever) package.

Create animation in Blender, convert it to drone paths, set up the drones and run your own show!

[![Build Status](https://travis-ci.org/CopterExpress/clever-show.svg?branch=master)](https://travis-ci.org/CopterExpress/clever-show)

## Demo video

[![Autonomous drone show in a theater](http://img.youtube.com/vi/HdHbZFz7nR0/0.jpg)](http://www.youtube.com/watch?v=HdHbZFz7nR0)

12 drones perform in a show in Electrotheatre Stanislavsky, Moscow.

## This software includes

* [Drone side](https://github.com/CopterExpress/clever-show/tree/master/Drone) with autonomous flight module, animation player module and client application for remote synchronized control of drones
* [Server side](https://github.com/CopterExpress/clever-show/tree/master/Server) for making the drone show with ability of tuning drones, animation and music
* [Blender 2.8 addon](https://github.com/CopterExpress/clever-show/tree/master/blender-addon) for animation export to drone paths
* [Raspberry Pi image](https://github.com/CopterExpress/clever-show/releases/latest) for quick launch software on the drones

## Documentation

> Documentation is available only in Russian for now.

Start tutorial is located [here](docs/ru/start-tutorial.md).

Detailed documentation is located in the [docs](https://github.com/CopterExpress/clever-show/tree/master/docs) folder.
