from bpy.types import Panel

class SwarmPanel(Panel):
    bl_label = "Clever Show"
    bl_idname = "SCENE_PT_swarm"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "scene"

    def draw(self, context):
        layout = self.layout
        box1 = layout.box()
        box1.label(text="Takeoff & land")
        col1 = box1.column()

        col1.prop(context.scene.clever_show, "use_armed")
        row1 = col1.row()
        row1.prop(context.scene.clever_show, "add_takeoff")
        row1.prop(context.scene.clever_show, "add_land")



