from bpy.types import Panel

class DronePanel(Panel):
    bl_label = "Clever-Show Drone"
    bl_idname = "OBJECT_PT_drone"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "object"

    def draw_header(self, context):
        self.layout.prop(context.object.drone, "is_drone", text="")

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        # layout.use_property_decorate = True

        layout.enabled = context.object.drone.is_drone

        layout.prop(context.object.drone, "armed")
