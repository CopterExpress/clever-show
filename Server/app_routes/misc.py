from flask import Blueprint, request, jsonify
from web_server_models import set_delay_manually, get_delay_manually, copters
from server import Client
from time import time

misc_api = Blueprint('misc_api', __name__, template_folder='templates')


@misc_api.route('/set/delay', methods=['GET', 'POST'])
def set_delay():
    set_delay_manually(int(request.args.get('delay')))
    return jsonify({'m': 'ok'})


@misc_api.route('/get/delay', methods=['GET', 'POST'])
def get_delay():
    return jsonify(get_delay_manually())


@misc_api.route('/stop/all', methods=['GET', 'POST'])
def stop_all():
    Client.broadcast_message("stop")
    return jsonify({'m': 'ok'})


@misc_api.route('/disarm/all', methods=['GET', 'POST'])
def disarm_all():
    Client.broadcast_message("disarm")
    return jsonify({'m': 'ok'})


@misc_api.route('/land/all', methods=['GET', 'POST'])
def land_all():
    Client.broadcast_message("land")
    return jsonify({'m': 'ok'})


@misc_api.route('/flip/selected', methods=['GET', 'POST'])
def flip_selected():
    ip = request.args.get("ip")
    for copter in copters:
        if copter.ip == ip:
            if takeoff_checks(copter):
                copter.client.send_message("flip")
    return jsonify({'m': 'ok'})


@misc_api.route('/takeoff/selected', methods=['GET', 'POST'])
def takeoff_selected():
    ip = request.args.get("ip")
    for copter in copters:
        if copter.ip == ip:
            if takeoff_checks(copter):
                copter.client.send_message("takeoff")
    return jsonify({'m': 'ok'})


@misc_api.route('/pause/selected', methods=['GET', 'POST'])
def pause_selected():
    ip = request.args.get("ip")
    for copter in copters:
        if copter.ip == ip:
            copter.client.send_message("pause")
    return jsonify({'m': 'ok'})


@misc_api.route('/resume/selected', methods=['GET', 'POST'])
def resume_selected():
    ip = request.args.get("ip")
    for copter in copters:
        if copter.ip == ip:
            copter.client.send_message("resume")
    return jsonify({'m': 'ok'})


def all_checks(copter):
    copter.refresh()
    checks = [check_anim(copter.anim_id),
              check_bat_p(((float(copter.batt_voltage) - 3.2) / (4.2 - 3.2)) * 100),
              check_bat_v(copter.cell_voltage),
              check_selfcheck(copter.selfcheck),
              check_time_delta(round(float(copter.time) - time(), 3))]
    return not (False in checks)


def takeoff_checks(copter):
    copter.refresh()
    checks = [check_bat_p(((float(copter.batt_voltage) - 3.2) / (4.2 - 3.2)) * 100),
              check_bat_v(copter.cell_voltage),
              check_selfcheck(copter.selfcheck)]
    return not (False in checks)


def check_anim(item):
    if not item:
        return None
    if str(item) == 'No animation':
        return False
    else:
        return True


def check_bat_v(item):
    if not item:
        return None
    if float(item) > 3.2:  # todo config
        return True
    else:
        return False


def check_bat_p(item):
    if not item:
        return None
    if float(item) > 30:  # todo config
        return True
    else:
        return False
        # return True #For testing


def check_selfcheck(item):
    if not item:
        return None
    if item == "OK":
        return True
    else:
        return False


def check_time_delta(item):
    if not item:
        return None
    if abs(float(item)) < 1:
        return True
    else:
        return False
