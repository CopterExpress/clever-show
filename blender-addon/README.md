# blender-csv-animation
A Blender extension that export paths of objects in blender animation to a csv files

## CSV file format
First row is the animation filename.
Every next row of the file contains following information about an object:
- frame number,
- x coordinate,
- y coordinate,
- z coordinate,
- rotaion around z-axis angle (yaw for copter),
- rgb.

## How to use it
Clone or download this repository
```bash
git clone https://github.com/artem30801/blender-csv-animation
```
Open Blender and install the addon:
1) Open User Prerences windows using main menu or shortcut (Ctrl + Alt + U): Files - User Preferences
2) Under Add-ons tab click Install Add-on from File...
3) Choose addon.py file from the directory of this repository
4) Enable the Add-on

Use [official docs](https://docs.blender.org/manual/en/latest/preferences/addons.html) for getting additional information
