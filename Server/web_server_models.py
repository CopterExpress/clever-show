from server import Client

copters = []
delay = 0


def set_delay_manually(_delay):
    global delay
    delay = _delay


def get_delay_manually():
    return delay


class WebCopter:
    def __init__(self, ip, client):
        self.anim_id = None
        self.batt_voltage = None
        self.cell_voltage = None
        self.selfcheck = None
        self.time = None
        self.ip = ip
        self.name = client.copter_id
        self.client = client
        self.refresh()

    def refresh(self):
        self.client.get_response("anim_id", save, callback_args=(self, 'anim_id'))
        self.client.get_response("batt_voltage", save, callback_args=(self, 'batt_voltage'))
        self.client.get_response("cell_voltage", save, callback_args=(self, 'cell_voltage'))
        self.client.get_response("selfcheck", save, callback_args=(self, 'selfcheck'))
        self.client.get_response("time", save, callback_args=(self, 'time'))


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
