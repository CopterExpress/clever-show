from bpy.types import Panel

class LedPanel(Panel):
    bl_label = "Clever-Show LED"
    bl_idname = "MATERIAL_PT_led"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "material"

    def draw_header(self, context):
        self.layout.prop(context.material.led, "is_led", text="")

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = True

        layout.enabled = context.material.led.is_led

        layout.prop(context.material.led, "group")
        layout.prop(context.material.led, "effect")
