import re

from bpy.types import Operator
from bpy.props import EnumProperty

class DroneCustomPropsActions(Operator):
    """Move items up and down, add and remove"""
    bl_idname = "clever_show.list_action"
    bl_label = "List Actions"
    bl_description = "Move items up and down, add and remove"
    bl_options = {'REGISTER', 'INTERNAL'}

    action: EnumProperty(
        items=(
            ('UP', "Up", ""),
            ('DOWN', "Down", ""),
            ('REMOVE', "Remove", ""),
            ('ADD', "Add", "")))

    def _add(self, context):
        drone = context.object.drone

        item = drone.custom_props.add()
        item.name = f"empty_action {len(drone.custom_props)}"

        drone.active_index = len(drone.custom_props) - 1
        self.report({'INFO'}, f"Item '{item.name}' added to {context.object.name}")

    def _move(self, context, item):
        drone = context.object.drone
        index = drone.active_index

        if self.action == 'DOWN':
            to_index = index + 1 if index < len(drone.custom_props) - 1 else 0
        else:  # elif self.action == 'UP':
            to_index = index - 1 if index > 0 else len(drone.custom_props) - 1

        drone.custom_props.move(index, to_index)
        drone.active_index = to_index
        self.report({'INFO'}, f"Item '{item.name}' "
                              f"moved to position {to_index}")

    def _remove(self, context, item):
        drone = context.object.drone
        index = drone.active_index

        info = f"Item '{item.name}' removed from list"
        drone.active_index = max(index - 1, 0) if index > 0 else index + 1
        drone.active_index = drone.active_index if len(drone.custom_props) > 1 else 0
        drone.custom_props.remove(index)
        self.report({'INFO'}, info)

    def invoke(self, context, event):
        if self.action == 'ADD':
            self._add(context)
            return {"FINISHED"}

        try:
            drone = context.object.drone
            index = drone.active_index
            item = drone.custom_props[index]
        except IndexError:
            return {'CANCELLED'}

        if self.action == 'REMOVE':
            self._remove(context, item)
        elif self.action in ('DOWN', 'UP'):
            self._move(context, item)

        return {"FINISHED"}

