# Blender animation export add-on

The add-on for Blender is designed to convert the flight animation of copters drawn in Blender into flight paths for each copter, taking into account the color of objects at each given time.

Export result is a folder with .csv files where each line  in file represents a sequence with comma delimiter:

* `x, y, z` coordinates of an object in meters
* `yaw` of an object in radians
* `red, green, blue` values of the color of an object, each is integer from 0 to 255

Documentation is located here:

* [English](../docs/en/blender-addon.md)
* [Russian](../docs/ru/blender-addon.md)
