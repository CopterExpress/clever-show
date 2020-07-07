import os
import sys

import json

import bpy
from bpy_extras.io_utils import ExportHelper
from bpy.types import Operator
from bpy.props import StringProperty, BoolProperty, FloatProperty, IntProperty


def create_dir(folder_path):
    if os.path.isdir(folder_path):
        return
    os.mkdir(folder_path)


def get_rgb(drone_obj):
    try:
        slot = next(filter(lambda x: "led_color" in x.name.lower(), drone_obj.material_slots))
    except StopIteration:
        raise RuntimeError("No matching slots")

    try:
        material = slot.material
        if material.use_nodes:
            value = get_node_color(material)
        else:
            value = material.diffuse_color

        alpha = value[3]
        return [int(value[component] * alpha * 255) for component in range(3)]
    except AttributeError:
        raise RuntimeError("Missing attributes")


def get_node_color(material):
    try:
        node = next(filter(lambda x: x.type in ('EMISSION', 'BSDF_DIFFUSE', "Principled BSDF"),
                           material.node_tree.nodes))
    except StopIteration:
        raise RuntimeError("Missing attributes")
    else:
        return node.inputs[0].default_value


class ExportSwarmAnimation(Operator, ExportHelper):
    bl_idname = "clever_show.export"
    bl_label = "clever-show animation (.anim)"
    filename_ext = ''
    use_filter_folder = True

    filepath: StringProperty(
        name="File Path",
        description="File path used for exporting CSV files",
        maxlen=1024,
        subtype='DIR_PATH',
        default=""
    )

    def _get_drone_objects(self, context):
        clever_show = context.scene.clever_show
        return context.visible_objects

    def execute(self, context):
        create_dir()

        drone_objects = self._get_drone_objects(context)

        for drone_obj in drone_objects:

            self._export_drone(drone_obj, context)

    def _export_drone(self, drone_obj, context):
        animation = []

        scene = context.scene

        frame_start = context.scene.frame_start
        frame_end = context.scene.frame_end

        for frame_num in range(frame_start, frame_end + 1):
            scene.frame_set(frame_start)
            position = drone_obj.matrix_world.to_translation()
            yaw = drone_obj.matrix_world.to_euler('XYZ')[2]
            frame = {"fly": list(position), "yaw": yaw, }

            try:
                led_color = get_rgb(drone_obj)
                frame.update({"led_color": led_color})
            except RuntimeError:
                pass

            animation.append(frame)



