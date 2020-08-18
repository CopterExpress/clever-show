from bpy.types import Panel


class OperatorsPanel(Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Clever Swarm"
    bl_label = "Clever Swarm Operators"

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

        row2 = layout.row()
        row2.operator("clever_show.select")
        row2.enabled = (clever_show.filter_obj != "selected")
