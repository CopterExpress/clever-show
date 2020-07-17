import bpy
from bpy.types import PropertyGroup

from bpy.props import PointerProperty, StringProperty, BoolProperty, EnumProperty, FloatProperty, IntProperty
from . operators.export import ExportSwarmAnimation
from . operators.check import CheckSwarmAnimation
from . ui.drone_panel import DronePanel
from . ui.swarm_panel import SwarmPanel

bl_info = {
    "name": "clever-show animation (.anim)",
    "author": "Artem Vasiunik & Arthur Golubtsov",
    "version": (0, 6, 2),
    "blender": (2, 83, 0),
    "location": "File > Export > clever-show animation (.anim)",
    "description": "Export > clever-show animation (.anim)",
    "doc_url": "https://github.com/CopterExpress/clever-show/blob/master/blender-addon/README.md",
    "tracker_url": "https://github.com/CopterExpress/clever-show/issues",
    "category": "Import-Export"
}


# noinspection PyArgumentList
class CleverShowProperties(PropertyGroup):

    add_takeoff: BoolProperty(
        name="Auto takeoff",
        description="Add takeoff command before animation start",
        default=True,
        options=set(),  # not animateable
    )

    add_land: BoolProperty(
        name="Auto land",
        description="Add land command after animation end",
        default=True,
        options=set(),  # not animateable
    )

    use_armed: BoolProperty(
        name="Use 'Armed' property",
        description="Add takeoff and land according to 'Armed' property",
        default=True,
    )

    detect_animation: BoolProperty(
        name="Detect in motion",
        description="Detect takeoff and land in drone object motion",
        default=False,
        options=set(),  # not animateable
    )

    takeoff_frames: IntProperty(
        name="Takeoff duration",
        description="Duration of takeoff in frames",
        default=70,
        min=1,
    )

    land_frames: IntProperty(
        name="Land duration",
        description="Duration of landing in frames",
        default=100,
        min=1,
    )

    filter_obj: EnumProperty(
        name="Filter objects:",
        items=[('all', "No filter (all objects)", ""),
               ('selected', "Only selected", ""),
               ('name', "By object name", ""),
               ('prop', "By object property", ""),
               ],
        default="prop",
        options=set(),  # not animateable
    )

    drones_name: StringProperty(
        name="Name identifier",
        description="Name identifier for all drone objects",
        default="clever",
        options=set(),  # not animateable
    )

    speed_limit: FloatProperty(
        name="Speed limit",
        description="Limit of drone movement speed (m/s)",
        unit='VELOCITY',
        default=3,
        min=0,
    )
    distance_limit: FloatProperty(
        name="Distance limit",
        description="Closest possible distance between drones (m)",
        unit='LENGTH',
        default=1.5,
        min=0,
    )


class CleverDroneProperties(PropertyGroup):
    is_drone: BoolProperty(name="Is drone")

    armed: BoolProperty(
        name="Armed",
        default=True,
    )


class CleverLedProperties(PropertyGroup):
    is_led: BoolProperty(
        name="Is LED color",)
    led_effect: EnumProperty(
        name="LED effect",
        items=[('fill', 'Fill', ""),
               ('blink', 'Blink', ""),
               ('blink_fast', 'Blink fast', ""),
               ('fade', 'Fade', ""),
               ('wipe', 'Wipe', ""),
               ('flash', 'Flash', ""),
               ('rainbow', 'Rainbow', ""),
               ('rainbow_fill', 'Rainbow fill', ""),
               ],

        default="fill",
    )


classes = (CleverShowProperties, CleverDroneProperties, CleverLedProperties,
           ExportSwarmAnimation, CheckSwarmAnimation,
           DronePanel, SwarmPanel,
           )

def menu_func(self, context):
    self.layout.operator(
        ExportSwarmAnimation.bl_idname,
        text="clever-show animation (.anim)"
    )


def register():
    from bpy.utils import register_class

    for cls in classes:
        register_class(cls)

    bpy.types.Scene.clever_show = PointerProperty(type=CleverShowProperties)
    bpy.types.Object.drone = PointerProperty(type=CleverDroneProperties)
    bpy.types.Material.led = PointerProperty(type=CleverLedProperties)

    bpy.types.TOPBAR_MT_file_export.append(menu_func)


def unregister():
    from bpy.utils import unregister_class

    for cls in reversed(classes):
        unregister_class(cls)

    del bpy.types.Scene.clever_show
    del bpy.types.Object.drone
    del bpy.types.Material.led

    bpy.types.TOPBAR_MT_file_export.remove(menu_func)


if __name__ == "__main__":
    register()
