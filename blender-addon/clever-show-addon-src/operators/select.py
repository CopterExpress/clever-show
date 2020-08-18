import bpy
from bpy.types import Operator

from .export import ExportSwarmAnimation

class SelectSwarmDrones(Operator):
    bl_idname = "clever_show.select"
    bl_label = "Select clever-show drones"

    def execute(self, context):
        bpy.ops.object.select_all(action='DESELECT')
        drones = ExportSwarmAnimation._get_drone_objects(context)
        first = True
        for drone_obj in drones:
            drone_obj.select_set(True)
            if first:
                bpy.context.view_layer.objects.active = drone_obj
                first = False

        return {'FINISHED'}
