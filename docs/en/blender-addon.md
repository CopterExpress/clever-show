# Animation export Blender addon

The Addon for Blender is designed to convert Blender's copters flight animations into flight paths for each copter of the animation, including the color of objects at any given time.

## Making the animation

In order to export drone animation you need to animate movements of any object(s) representing a drone. Coordinates of the drone objects will be used as coordinates for drone animation playback for each frame.
Material color of the object will be exported as LED strip color for each frame. The material name should start with `led`. If material uses nodes, type of material node should be `Emmision` or `Diffuse`.

If you don't know how to work in Blender you can check out those links:
* [Officail documentation - Interface](https://docs.blender.org/manual/en/latest/interface/index.html)
* [Officail documentation - Animation](https://docs.blender.org/manual/en/latest/animation/index.html)
* [Hotkeys cheat sheet](https://docs.google.com/document/d/1zPBgZAdftWa6WVa7UIFUqW_7EcqOYE0X743RqFuJL3o/edit?usp=sharing)

## Addon installation and configuration

* Download and install the latest version of Blender 2.90 from [the official website](https://www.blender.org/download/).
* Open Blender, select `Edit > Preferences` from the top menu. In the opened settings window, select `Add-ons` in the side panel. Click the button `Install...` in the upper right corner of the window. In the dialog box, open the path to the addon folder [clever-show/blender-addon](.../../blender-addon/) and select the file `addon.py`. Click `Install Add-on from file...`. Addon is now installed.
* After installing the addon, tick the `Import-Export: clever-show animation (.csv)` checkbox to activate the addon.

Addon is now active and ready to go. You will not need to perform these operations at further Blender startups on the same PC.

You may want to change FPS of in-Blender animation playback to match it with drone animation execution speed (10 frames per second). In order to change it go to :

>Properties editor (by default, on the left) > Output properties tab > Dimensions panel > Frame Rate

Change value of the property to `Custom`, then change value of appeared below `FPS` property to `10`.

## Exporting with the addon

* To open the export dialog box, click on the top menu `File > Export > clever-show animation (.csv)`. In the export window that opens, you should select the destination export path and the name of the folder that the addon will create during the export process. The export options panel is available in the side menu:
* `Use name filter for objects` - checkbox determines if the object filter will be used while saving the paths. If this option is disabled, the paths of all visible objects will be exported. `Name identifier` - object name filter. If checkbox `Use name filter for objects` is active, only paths of objects containing this value in the name will be saved.
* `Show detailed animation warnings` - checkbox determines whether the animation's speeds and distances limits warnings will be displayed.
* `Speed limit` - warnings will be displayed if the specified speed limit is violated.
* `Distance limit` - warnings will be displayed if the specified minimum distance between drones is violated.

After configuring the required parameters, press the `Export clever-show animation` button. Animations of the specified objects from the Blender project will be exported to the specified folder as files in the `.csv` format where each line  in file represents a sequence with comma delimiter:

* `x, y, z` coordinates of an object in meters
* `yaw` of an object in radians
* `red, green, blue` values of the color of an object, each is integer from 0 to 255

## Deactivation and removal

To deactivate an addon, uncheck the checkbox next to the addon name as described [above] (#installation and configuration).

For more information, click the arrow icon to the left of the activation field. There are also buttons in the unfolded block:

* `Documentation` - leads to the addon's documentation page.
* `Report a bug` - leads to the issues page of the clever-show repository.
* `Remove` - removes the addon (before installing a new version it is recommended to remove the old one).
