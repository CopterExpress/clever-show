from server import Client


class WebCopter:
    def __init__(self, ip, name):
        self.anim_id = None
        self.batt_voltage = None
        self.cell_voltage = None
        self.selfcheck = None
        self.time = None
        self.ip = ip
        self.name = name
        self.refresh()

    def refresh(self):
        Client.clients[self.ip].get_response("anim_id", save, callback_args=(self, 'anim_id'))
        Client.clients[self.ip].get_response("batt_voltage", save, callback_args=(self, 'batt_voltage'))
        Client.clients[self.ip].get_response("cell_voltage", save, callback_args=(self, 'cell_voltage'))
        Client.clients[self.ip].get_response("selfcheck", save, callback_args=(self, 'selfcheck'))
        Client.clients[self.ip].get_response("time", save, callback_args=(self, 'time'))


def save(m, self, param_name):
    if param_name == 'anim_id':
        self.anim_id = m
    elif param_name == 'batt_voltage':
        self.batt_voltage = m
    elif param_name == 'cell_voltage':
        self.cell_voltage = m
    elif param_name == 'selfcheck':
        self.selfcheck = m
    elif param_name == 'time':
        self.time = m
