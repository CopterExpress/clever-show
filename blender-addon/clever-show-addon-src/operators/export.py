import os
import sys

import json
import itertools
from collections.abc import Mapping

import bpy
from bpy_extras.io_utils import ExportHelper
from bpy.types import Operator
from bpy.props import StringProperty, BoolProperty, FloatProperty, IntProperty


def create_dir(folder_path):
    if not os.path.isdir(folder_path):
        os.mkdir(folder_path)


# def iter_pairs(obj):
#     return zip(obj[::2], obj[1::2])

def neighbour_pairs(sequence):
    iterable = iter(sequence)
    try:
        prev = next(iterable)
    except StopIteration:
        return ()
    for item in iterable:
        yield prev, item
        prev = item


def update_nested(d, u):
    for key, val in u.items():
        if isinstance(val, Mapping):
            d[key] = update_nested(d.get(key, {}), val)
        else:
            d[key] = val
    return d


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

    def draw(self, context):
        clever_show = context.scene.clever_show

        layout = self.layout
        col = layout.column()
        col.label(text="Filtering")
        col.prop(clever_show, "filter_obj")
        if clever_show.filter_obj == "name":
            col.prop(clever_show, "drones_name")
        col.separator()

    @staticmethod
    def _get_drone_objects(context):
        clever_show = context.scene.clever_show
        filtering = clever_show.filter_obj

        if filtering == "selected":
            return context.selected_objects

        objects = context.visible_objects
        if filtering == "all":
            return objects
        if filtering == "name":
            return (d_obj for d_obj in objects if clever_show.drones_name in d_obj.name)
        if filtering == "prop":
            return (d_obj for d_obj in objects if d_obj.drone.is_drone)

        raise ValueError

    @staticmethod
    def _get_led_materials(context, drone_obj):
        clever_show = context.scene.clever_show
        filtering = clever_show.filter_mats
        if filtering == "none":
            return []

        materials = [slot.material for slot in drone_obj.material_slots if slot.material]
        if filtering == "all":
            return materials
        if filtering == "name":
            return (mat for mat in materials if mat.name == clever_show.leds_name)
        if filtering == "prop":
            return (mat for mat in materials if mat.led.is_led)

        raise ValueError

    @staticmethod
    def _get_led_color(material):
        if material.use_nodes:
            value = ExportSwarmAnimation._get_node_color(material)
        else:
            value = material.diffuse_color

        alpha = value[3]
        return [int(value[component] * alpha * 255) for component in range(3)]

    @staticmethod
    def _get_node_color(material):
        supported_nodes = ('EMISSION', 'BSDF_DIFFUSE', "BSDF_PRINCIPLED")
        nodes = material.node_tree.nodes
        links = material.node_tree.links
        out = nodes.get("Material Output")
        try:
            link = [l for l in links if l.from_node.type in supported_nodes and l.to_node == out][0]
        except IndexError:
            raise RuntimeError("Missing proper nodes")
        else:
            return link.from_node.inputs['Base Color'].default_value  # [0]

    def execute(self, context):
        create_dir(self.filepath)

        drone_objects = self._get_drone_objects(context)

        for drone_obj in drone_objects:
            animation = self._generate_animation(drone_obj, context)
            animation = self._remove_idle_frames(animation)
            animation = self._remove_repeats(animation)
            animation = self._compress_empty(animation)
            with open(os.path.join(self.filepath, '{}.anim'.format(drone_obj.name)), 'w') as f:
                f.writelines(json.dumps(frame)+"\n" for frame in animation)

        return {'FINISHED'}

    def _generate_animation(self, drone_obj, context):
        # todo yield?
        animation = list()

        scene = context.scene
        clever_show = scene.clever_show

        frame_start = context.scene.frame_start
        frame_end = context.scene.frame_end
        # Add frame with animation parameters
        scene.frame_set(frame_start)

        animation.append({"set_fps": context.scene.render.fps,
                          "drone_name": drone_obj.name,
                          })

        # Add flight
        for frame_num in range(frame_start, frame_end + 1):
            scene.frame_set(frame_num)
            position = [round(x, 3) for x in drone_obj.matrix_world.to_translation()]
            yaw = round(drone_obj.matrix_world.to_euler('XYZ')[2], 3)
            frame = dict()

            # check to not update position or yaw if they are same as previous frame
            frame.update({"fly": position})
            frame.update({"yaw": yaw})
            if clever_show.use_armed:
                frame.update({"armed": drone_obj.drone.armed})

            led_materials = self._get_led_materials(context, drone_obj)
            for material in led_materials:

                try:
                    color = self._get_led_color(material)
                except (RuntimeError, AttributeError) as e:
                    continue

                if material.led.is_led:  # and clever_show.filter_mats == "prop"
                    group_name = material.led.group
                    effect = material.led.effect
                    update_nested(frame, {"led_effect": {group_name: effect}})
                else:
                    group_name = "ALL"

                update_nested(frame, {"led_color": {group_name: color}})

            animation.append(frame)

        if clever_show.add_takeoff:
            animation.insert(1, {"takeoff": {}})

        if clever_show.add_land:
            animation.append({"land": {}})

        if clever_show.use_armed:
            pass
            #self._detect_armed_states(drone_obj, animation, context)

        return animation

    @staticmethod
    def find_intervals(values, ensure_first=True):
        j = None
        if ensure_first:
            values = itertools.chain((-1, ), values)  # to ensure detection from first element
        for i, items in enumerate(neighbour_pairs(values)):
            item1, item2 = items
            if item2 > item1:
                j = i
            elif j is not None and item2 < item1:
                yield (j, i)  # j+1, i+1
                j = None

        if j is not None:
            yield (j, j)  # j+1, j+1

    @staticmethod
    def pop_func(animation, func):
        state = False
        for frame in animation:
            current = frame.pop(func, None)
            state = current if current is not None else state
            yield state

    @staticmethod
    def _populate_with_defaults(animation, func):
        pass

    @classmethod
    def _detect_states(cls, drone_obj, animation, context):
        # clever_show = context.scene.clever_show
        intervals = cls.find_intervals(cls.pop_func(animation, "armed"), ensure_first=False)

        for start, end in intervals:
            pass
            # duration = max(end-start, 1)

            # if start < end:  # not armed -> armed: takeoff
            #     func = "takeoff"
            # else:  # armed -> not armed: takeoff
            #     func = "land"

            # animation[start_frame].update({func: {"duration": duration}})

    @classmethod
    def _detect_armed_states(cls, animation):
        for i, items in enumerate(neighbour_pairs(animation)):
            item1, item2 = items
            if item2 > item1:
                j = i
            elif j is not None and item2 < item1:
                yield (j, i)  # j+1, i+1
                j = None


        return animation

    def _detect_animation_takeoff_land(self, drone_obj, animation, context):
        floor_level = 0
        start_frame = 0
        for i, frames in enumerate(neighbour_pairs(animation)):
            frame1, frame2 = frames
            if frame1 == floor_level and frame2 > frame1:  # takeoff start
                pass
            elif frame2 == floor_level and frame1 > frame2:  # land start
                pass

        def find(animation):

            def get_z(index, default=float('nan')):
                return animation[index].get("fly", None)[2] or default

            i = 0
            previous_z = get_z(i) # height of the first frame
            while i < len(animation):
                current_z = get_z(i, previous_z)

                if previous_z == 0:
                    while current_z > previous_z:
                        i += 1
                        previous_z = current_z
                        current_z = get_z(i, previous_z)

                i += 1
                previous_z = current_z

    @staticmethod
    def _remove_repeats(animation):
        to_remove = ("fly", "fly_gps", "yaw", "led_color", "led_effect", "armed")
        previous_frame = dict()
        for frame in animation:
            for func in to_remove:
                val = frame.get(func)
                if val is not None and val == previous_frame.get(func, None):
                    frame.pop(func, None)
                else:
                    previous_frame.update({func: val})
        return animation

    @staticmethod
    def _compress_empty(animation):
        i = 0
        while i < len(animation) - 1:
            if not animation[i + 1]:
                frame = animation[i]
                frame.update({"skip": frame.get("skip", 0) + 1})
                animation.pop(i + 1)
            else:
                i += 1
        return animation

    @staticmethod
    def _remove_idle_frames(animation):  #delete unnececary flight functions while copter landed
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

