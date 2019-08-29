from server import Server
from web_server_models import copters, get_delay_manually
from flask import Flask, render_template, jsonify, request
from app_routes.selfcheck import selfcheck_api, refresh_copters
from app_routes.file_sender import file_sender_api
from app_routes.misc import misc_api

app = Flask(__name__, static_url_path='/static')
app.register_blueprint(selfcheck_api)
app.register_blueprint(file_sender_api)
app.register_blueprint(misc_api)


@app.route('/')
def home():
    data = dict()
    refresh_copters()
    return render_template('main.html', data=data)


@app.route('/start_animation', methods=['GET', 'POST'])
def resume_selected():
    for copter in copters:
        server.send_starttime(copter.client, get_delay_manually())
    return jsonify({'m': 'ok'})


@app.route('/em_land', methods=['GET', 'POST'])
def em_land_selected():
    for copter in copters:
        copter.client.send_message("land")
    return jsonify({'m': 'ok'})


server = Server()
server.start()
app.run(host='0.0.0.0', debug=False)
