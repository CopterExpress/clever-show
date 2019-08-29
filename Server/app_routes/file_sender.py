from flask import Blueprint, request, jsonify
from web_server_models import copters

file_sender_api = Blueprint('file_sender_api', __name__, template_folder='templates')


@file_sender_api.route('/set/animation', methods=['GET', 'POST'])
def set_animation():
    if request.method == 'POST':
        f = request.files['file']
        print(f, 'ip', request.args.get('ip'))
    return jsonify({'m': 'ok'})


@file_sender_api.route('/set/config', methods=['GET', 'POST'])
def set_config():
    if request.method == 'POST':
        f = request.files['file']
        print(f, 'ip', request.args.get('ip'))
    return jsonify({'m': 'ok'})


@file_sender_api.route('/set/aruco', methods=['GET', 'POST'])
def set_aruco():
    if request.method == 'POST':
        f = request.files['file']
        print(f, 'ip', request.args.get('ip'))
    return jsonify({'m': 'ok'})
