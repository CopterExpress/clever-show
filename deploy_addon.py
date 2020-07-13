import os
import shutil


#shutil.copytree("blender-addon", 'C:/Users/artem/AppData/Roaming/Blender Foundation/Blender/2.83/scripts/addons/')
shutil.rmtree('C:/Users/artem/AppData/Roaming/Blender Foundation/Blender/2.83/scripts/addons/blender-addon/')
shutil.copytree("blender-addon/clever-show-addon-src", 'C:/Users/artem/AppData/Roaming/Blender Foundation/Blender/2.83/scripts/addons/blender-addon/')
#shutil.make_archive("addon", 'zip', "blender-addon")
#shutil.move("addon.zip", 'C:/Users/artem/AppData/Roaming/Blender Foundation/Blender/2.83/scripts/addons/addon.zip')
