import os
import sys

import json
import itertools

import bpy
from bpy_extras.io_utils import ExportHelper
from bpy.types import Operator
from bpy.props import StringProperty, BoolProperty, FloatProperty, IntProperty


def create_dir(folder_path):
    if os.path.isdir(folder_path):
        return
    os.mkdir(folder_path)


def iter_pairs(obj):
    return zip(obj[::2], obj[1::2])


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
            animation = self._generate_animation(drone_obj, context)
            animation = self._process_animation(animation, context)
            with open("12", "w") as f:
                pass

    def _generate_animation(self, drone_obj, context):
        animation = list()

        scene = context.scene
        clever_show = scene.clever_show

        frame_start = context.scene.frame_start
        frame_end = context.scene.frame_end
        # Add frame with animation parameters
        scene.frame_set(frame_start)
        # animation.append({"set_fps": context.scene.render.fps,
        #                   "drone_name": drone_obj.name,
        #                   })

        if clever_show.add_takeoff:
            animation.append({"takeoff": {}})

        # Add flight and
        for frame_num in range(frame_start, frame_end + 1):
            scene.frame_set(frame_start)
            position = [round(x, 3) for x in drone_obj.matrix_world.to_translation()]
            yaw = round(drone_obj.matrix_world.to_euler('XYZ')[2], 3)
            frame = dict()

            # check to not update position or yaw if they are same as previous frame
            if animation[-1]["fly"] != position:
                frame.update({"fly": position})
            if animation[-1]["yaw"] != yaw:
                frame.update({"yaw": yaw})

            try:
                led_color = get_rgb(drone_obj) # TODO!!!!!!
                frame.update({"led_color": led_color})
            except RuntimeError:
                pass

            animation.append(frame)

        if clever_show.add_land:
            animation.append({"land": {}})

        if clever_show.use_armed:
            self._detect_armed_states(drone_obj, animation, context)

        return animation

    def _detect_armed_states(self, drone_obj, animation, context):
        # clever_show = context.scene.clever_show

        tracks = drone_obj.animation_data
        if not tracks:
            return animation

        fcurve = next((fcurve for fcurve in tracks if fcurve.data_path == "drone.armed"), None)
        if fcurve is None:
            return animation

        keyframes = fcurve.keyframe_points
        if len(keyframes) % 2 != 0:
            raise RuntimeError("Incorrect armed state keyframes!")

        for frame1, frame2 in iter_pairs(keyframes):
            if frame1 == frame2:  # equal: wrong order
                raise RuntimeError("Incorrect armed state keyframes!")

            start_frame = frame1.co[0]
            duration = frame2.co[0] - start_frame
            # todo height?

            if frame1 < frame2:  # not armed -> armed: takeoff
                func = "takeoff"
            else:  # armed -> not armed: takeoff
                func = "land"

            animation[start_frame].update({func: {"duration": duration}})

        return animation

    def _detect_animation_takeoff_land(self, drone_obj, animation, context):
        takeoff = False
        land = False

        # previous_z = 0
        def find(animation):
            i = 0
            previous_z = animation[i]["fly"][2]  # height of the first frame
            while i < len(animation):
                z = animation[i]["fly"][2]




            for frame in animation:
                z = frame["fly"][2]
                if previous_z == 0 and z > previous_z:
                    takeoff = True
                #if p




    def _process_animation(self, animation, context):  #delete unnececary flight functions while copter landed
        # clever_show = context.scene.clever_show
        flight_functions = ("fly", "fly_gps", "yaw", "flip, ")

        def get_frames(func):
            return [i for i, frame in enumerate(animation) if func in frame]

        takeoffs = get_frames("takeoff")
        lands = get_frames("land")

        # Add 'fake' landing (only for checks) at the beginning if animation starts with takeoff
        if takeoffs[0] < lands[0]:
            lands.insert(0, 0)  #             lands.insert(0, 1)

        # Add 'fake' takeoff (only for checks) at the end if animation doesn't ends with takeoff
        last = len(animation)
        if last not in takeoffs:
            takeoffs.append(last - 1)

        # Now we 'strip' flight functions between every land and takeoff (when drone is on the ground)
        # Note that animation CAN be started mid-air (without takeoff in animation itself) or end without landing
        to_delete = itertools.chain.from_iterable(
            range(land, takeoff + 1) for takeoff, land in zip(takeoffs, lands))
        for frame_num in to_delete:
            frame = animation[frame_num]
            for func in flight_functions:
                frame.pop(func, None)

        # TODO DELETE FRAMES ADTER|DURING TAKEOFF

        return animation

