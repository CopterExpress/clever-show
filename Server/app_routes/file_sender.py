import os
from json import loads
from flask import Blueprint, request, jsonify
from web_server_models import copters
from server import ConfigOption
import configparser

file_sender_api = Blueprint('file_sender_api', __name__, template_folder='templates')


@file_sender_api.route('/set/config', methods=['GET', 'POST'])
def set_config():
    if request.method == 'POST':
        key_name = ""
        ips = request.values.get('ips').split(',')

        for _key_name in request.files.keys():
            key_name = _key_name
            request.files[key_name].save(os.path.join('files', _key_name))
        sendable_config = configparser.ConfigParser()
        sendable_config.read('files/' + key_name)
        options = []
        for section in sendable_config.sections():
            for option in dict(sendable_config.items(section)):
                value = sendable_config[section][option]
                options.append(ConfigOption(section, option, value))

        for copter in copters:
            for ip in ips:
                if copter.ip == ip:
                    copter.client.send_config_options(*options)
        os.remove('files/' + key_name)
    return jsonify({'m': 'ok'})


@file_sender_api.route('/set/aruco', methods=['GET', 'POST'])
def set_aruco():
    if request.method == 'POST':
        for key_name in request.files.keys():
            request.files[key_name].save(os.path.join('files', key_name))
            for copter in copters:
                copter.client.send_file('files/' + key_name,
                                        "/home/pi/catkin_ws/src/clever/aruco_pose/map/animation_map.txt")
                copter.client.send_message("service_restart", {"name": "clever"})
            os.remove('files/' + key_name)
    return jsonify({'m': 'ok'})


@file_sender_api.route('/set/animation', methods=['GET', 'POST'])
def set_animation():
    if request.method == 'POST':
        files = []
        names = []
        for key_name in request.files.keys():
            names.append(key_name.replace('.csv', ''))
            request.files[key_name].save(os.path.join('files', key_name))
            files.append('files/' + key_name)
        for file, name in zip(files, names):
            for copter in copters:
                if name == copter.name:
                    copter.client.send_file(file, "animation.csv")
        for filename in files:
            os.remove(filename)
    return jsonify({'m': 'ok'})


@file_sender_api.route('/set/launch', methods=['GET', 'POST'])
def set_launch():
    if request.method == 'POST':
        ips = request.values.get('ips').split(',')
        files = []
        names = []
        for key_name in request.files.keys():
            names.append(key_name)
            request.files[key_name].save(os.path.join('files', key_name))
            files.append('files/' + key_name)
        if len(files) > 0:
            for copter in copters:
                if copter.ip in ips:
                    for file, name in zip(files, names):
                        copter.client.send_file('files/' + name,
                                                "/home/pi/catkin_ws/src/clever/launch/" + name)
                    copter.client.send_message("service_restart", {"name": "clever"})
        for filename in files:
            os.remove(filename)

    return jsonify({'m': 'ok'})
