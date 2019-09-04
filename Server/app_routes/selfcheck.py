from flask import Blueprint, request, jsonify
from web_server_models import copters, WebCopter
from server import Client

selfcheck_api = Blueprint('selfcheck_api', __name__, template_folder='templates')


@selfcheck_api.route('/selfcheck/selected', methods=["GET", "POST"])
def selfcheck_selected():
    data = dict()
    ip = request.args.get("ip")
    for copter in copters:
        if copter.ip == ip:
            time = copter.refresh()
            data = {
                'anim_id': copter.anim_id,
                'batt_voltage': ((float(copter.batt_voltage) - 3.2) / (4.2 - 3.2)) * 100,
                'cell_voltage': copter.cell_voltage,
                'selfcheck': copter.selfcheck,
                'time': round(float(copter.time) - time, 3),
                'name': copter.name,
            }
    # data = {"anim_id": "No animation", "batt_voltage": 3.259999990463257, "cell_voltage": 1.0850000381469727,
    #        "ip": "192.168.43.31", "name": "CLever7", "selfcheck": "OK", "time": 1554723269.57106}
    return jsonify(data)


@selfcheck_api.route('/selfcheck/all', methods=["GET", "POST"])
def selfcheck_all():
    data = []
    for copter in copters:
        time = copter.refresh()
        data.append({
            'anim_id': copter.anim_id,
            'batt_voltage': ((float(copter.batt_voltage) - 3.2) / (4.2 - 3.2)) * 100,
            'cell_voltage': copter.cell_voltage,
            'selfcheck': copter.selfcheck,
            'ip': copter.ip,
            'time': round(float(copter.time) - time(), 3),
            'name': copter.name,
        })
    # return jsonify([{"anim_id": "No animation", "batt_voltage": 12.333000183105469, "cell_voltage": 4.111999988555908,
    #                 "ip": "192.168.1.191", "name": "Clever8", "selfcheck": "OK", "time": 1554720157.469769},
    #                {"anim_id": "Cool animation", "batt_voltage": 12.244999885559082,
    #                 "cell_voltage": 4.0879998207092285,
    #                 "ip": "192.168.1.134", "name": "Clever7", "selfcheck": "OK", "time": 1554720082.641047}])
    return jsonify(data)


@selfcheck_api.route('/refresh_copters')
def refresh_copters():
    try:
        for client_ip in Client.clients.keys():
            is_in = False
            for copter in copters:
                if client_ip == copter.ip:
                    is_in = True
            if not is_in:
                copters.append(WebCopter(client_ip, Client.clients[client_ip]))
        return jsonify({'m': 'Ok'})
    except:
        return jsonify({'m': 'Error'})


@selfcheck_api.route('/test_led/selected', methods=['GET', 'POST'])
def test_led_selected():
    ip = request.args.get("ip")
    for copter in copters:
        if copter.ip == ip:
            copter.client.send_message("led_test")
    return jsonify({'m': 'ok'})


@selfcheck_api.route('/reboot_fcu/selected', methods=['GET', 'POST'])
def reboot_fcu_selected():
    ip = request.args.get("ip")
    for copter in copters:
        if copter.ip == ip:
            copter.client.send_message("reboot_fcu")
    return jsonify({'m': 'ok'})


@selfcheck_api.route('/calibrate_gyro/selected', methods=['GET', 'POST'])
def calibrate_gyro_selected():
    ip = request.args.get("ip")
    for copter in copters:
        if copter.ip == ip:
            copter.client.send_message("calibrate_gyro")
    return jsonify({'m': 'ok'})


@selfcheck_api.route('/calibrate_level/selected', methods=['GET', 'POST'])
def calibrate_level_selected():
    ip = request.args.get("ip")
    for copter in copters:
        if copter.ip == ip:
            copter.client.send_message("calibrate_level")
    return jsonify({'m': 'ok'})
