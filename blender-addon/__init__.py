import bpy
from bpy.utils import register_class, unregister_class

from . operators.exporter import *

bl_info = {
    "name": "Export > CSV Drone Swarm Animation Exporter (.csv)",
    "author": "Artem Vasiunik",
    "version": (0, 4, 4),
    "blender": (2, 80, 0),
    "location": "File > Export > CSV Drone Swarm Animation Exporter (.csv)",
    "description": "Export > CSV Drone Swarm Animation Exporter (.csv)",
    "warning": "",
    "wiki_url": "https://github.com/artem30801/blender-csv-animation/blob/master/README.md",
    "tracker_url": "https://github.com/artem30801/blender-csv-animation/issues",
    "category": "Import-Export"
}

classes = (ExportCsv, )


def menu_func(self, context):
    self.layout.operator(
        ExportCsv.bl_idname,
        text="CSV Drone Swarm Animation Exporter (.csv)"
    )


def register():
    for cls in classes:
        register_class(cls)

    bpy.types.TOPBAR_MT_file_export.append(menu_func)


def unregister():
    for cls in reversed(classes):
        unregister_class(cls)

    bpy.types.TOPBAR_MT_file_export.remove(menu_func)


if __name__ == "__main__":
    register()
