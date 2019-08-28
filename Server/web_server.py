import threading
from web_server_models import WebCopter
from server import *
from flask import Flask, render_template, jsonify, request, send_from_directory

app = Flask(__name__, static_url_path='/static')
copters = []


@app.route('/')
def home():
    data = dict()
    data['clients'] = []
    refresh_copters()
    for client in Client.clients.keys():
        data['clients'].append([client, Client.clients[client].copter_id])
    return render_template('main.html', data=data)


@app.route('/refresh_copters')
def refresh_copters():
    try:
        for client_ip in Client.clients.keys():
            is_in = False
            for copter in copters:
                if client_ip == copter.ip:
                    is_in = True
            if not is_in:
                copters.append(WebCopter(client_ip, Client.clients[client_ip].copter_id))
        return jsonify({'m': 'Ok'})
    except:
        return jsonify({'m': 'Error'})


@app.route('/selfcheck/selected', methods=["GET", "POST"])
def selfcheck_selected():
    data = dict()
    ip = request.args.get("ip")
    for copter in copters:
        if copter.ip == ip:
            copter.refresh()
            data = {
                'anim_id': copter.anim_id,
                'batt_voltage': copter.batt_voltage,
                'cell_voltage': copter.cell_voltage,
                'selfcheck': copter.selfcheck,
                'time': copter.time,
                'name': copter.name,
            }
    data = {"anim_id": "No animation", "batt_voltage": 3.259999990463257, "cell_voltage": 1.0850000381469727,
            "ip": "192.168.43.31", "name": "CLever7", "selfcheck": "OK", "time": 1554723269.57106}
    return jsonify(data)


@app.route('/selfcheck/all', methods=["GET", "POST"])
def selfcheck_all():
    data = []
    for copter in copters:
        copter.refresh()
        data.append({
            'anim_id': copter.anim_id,
            'batt_voltage': copter.batt_voltage,
            'cell_voltage': copter.cell_voltage,
            'selfcheck': copter.selfcheck,
            'ip': copter.ip,
            'time': copter.time,
            'name': copter.name,
        })
    data = [{"anim_id": "No animation", "batt_voltage": 4.259999990463257, "cell_voltage": 4.0850000381469727,
             "ip": "192.168.43.31", "name": "CLever7", "selfcheck": "OK", "time": 4554723269.57106}]
    data *= 12
    return jsonify(data)


@app.route('/set/animation', methods=['GET', 'POST'])
def set_animation():
    if request.method == 'POST':
        f = request.files['file']
        print(f, 'ip', request.args.get('ip'))
    return jsonify({'m': 'ok'})


@app.route('/set/config', methods=['GET', 'POST'])
def set_config():
    if request.method == 'POST':
        f = request.files['file']
        print(f, 'ip', request.args.get('ip'))
    return jsonify({'m': 'ok'})


@app.route('/set/aruco', methods=['GET', 'POST'])
def set_aruco():
    if request.method == 'POST':
        f = request.files['file']
        print(f, 'ip', request.args.get('ip'))
    return jsonify({'m': 'ok'})


class ServerThread(threading.Thread):
    def run(self):
        server = Server()
        server.start()
        while True:
            pass


server_thread = ServerThread()
server_thread.start()
app.run(host='0.0.0.0', debug=False)
