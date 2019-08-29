import os
from json import loads
from flask import Blueprint, request, jsonify
from web_server_models import copters

file_sender_api = Blueprint('file_sender_api', __name__, template_folder='templates')


@file_sender_api.route('/set/config', methods=['GET', 'POST'])
def set_config():
    if request.method == 'POST':
        f = request.files['file']
        print(f, 'ip', request.args.get('ip'))
    return jsonify({'m': 'ok'})


@file_sender_api.route('/set/aruco', methods=['GET', 'POST'])
def set_aruco():
    if request.method == 'POST':
        for key_name in request.files.keys():
            request.files[key_name].save(os.path.join('files', key_name))
            for ip in loads(request.args.get('ips')):
                for copter in copters:
                    if copter.ip == ip:
                        copter.client.send_file('files/' + key_name, "/home/pi/catkin_ws/src/clever/aruco_pose/map/animation_map.txt")
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
