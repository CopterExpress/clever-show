from flask import Blueprint, request, jsonify
from web_server_models import delay, set_delay_manually, get_delay_manually

misc_api = Blueprint('misc_api', __name__, template_folder='templates')


@misc_api.route('/set/delay', methods=['GET', 'POST'])
def set_delay():
    set_delay_manually(int(request.args.get('delay')))
    return jsonify({'m': 'ok'})


@misc_api.route('/get/delay', methods=['GET', 'POST'])
def get_delay():
    return jsonify(get_delay_manually())
