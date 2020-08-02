from bpy.types import Panel

class SwarmPanel(Panel):
    bl_label = "Clever Show"
    bl_idname = "SCENE_PT_swarm"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "scene"

    def draw(self, context):
        layout = self.layout

        takeoff_box = layout.box()
        takeoff_box.label(text="Takeoff & land")
        col1 = takeoff_box.column()

        col1.prop(context.scene.clever_show, "use_armed")
        row1 = col1.row()
        row1.prop(context.scene.clever_show, "add_takeoff")
        row1.prop(context.scene.clever_show, "add_land")

class SwarmFilteringPanel(Panel):
    bl_label = "Filtering"
    bl_idname = "SCENE_PT_swarm_filtering"
    bl_parent_id = "SCENE_PT_swarm"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "scene"

    def draw(self, context):
        layout = self.layout
        clever_show = context.scene.clever_show
        layout.use_property_split = True
        layout.use_property_decorate = False

        col = layout.column(align=True)
        col.prop(clever_show, "filter_obj")

        row1 = col.row()
        row1.enabled = (clever_show.filter_obj == "name")
        row1.prop(clever_show, "drones_name")

        col.prop(clever_show, "filter_mats")
        row2 = col.row()
        row2.enabled = (clever_show.filter_mats == "name")
        row2.prop(clever_show, "leds_name")
