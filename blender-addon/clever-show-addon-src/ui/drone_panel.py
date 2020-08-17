from bpy.types import Panel, UIList


class CustomDroneItems(UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        split = layout.split(factor=0.66, align=True)
        split.prop(item, "name", text="", emboss=False)
        row = split.row(align=True)
        row.label(text="Active ")
        checkbox = "CHECKBOX_HLT" if item.active else "CHECKBOX_DEHLT"
        row.prop(item, "active", text="", emboss=False, icon=checkbox)


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
        drone = context.object.drone
        layout.use_property_split = True
        layout.use_property_decorate = True

        layout.enabled = drone.is_drone

        layout.prop(drone, "armed")
        items = len(drone.custom_props)

        rows = 3
        if items > 0:
            rows = 5

        row = layout.row()
        row.template_list("CustomDroneItems", "", drone, "custom_props", drone, "active_index", rows=rows)

        col = row.column(align=True)
        col.operator("clever_show.list_action", icon='ADD', text="").action = 'ADD'
        col.operator("clever_show.list_action", icon='REMOVE', text="").action = 'REMOVE'
        col.separator()
        # col.menu("", icon='DOWNARROW_HLT', text="")

        if items > 0:
            col.separator()
            sub = col.column(align=True)
            sub.operator("clever_show.list_action", icon='TRIA_UP', text="").action = 'UP'
            sub.operator("clever_show.list_action", icon='TRIA_DOWN', text="").action = 'DOWN'

            sub.enabled = items > 1

            try:
                item = drone.custom_props[drone.active_index]
            except IndexError:
                return

            layout.prop(item, "args")

