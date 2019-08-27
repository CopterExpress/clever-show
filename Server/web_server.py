import threading
from web_server_models import WebCopter
from server import *
from flask import Flask, render_template, jsonify, request, send_from_directory

app = Flask(__name__, static_url_path='/static')
copters = []


def response_handler(res, name, ip):
    print('\n\n\n', res, '\n\n\n', name, '\n\n\n', ip, '\n\n\n')
    return 1


@app.route('/')
def home():
    data = dict()
    data['clients'] = []
    # refresh_copters_list()
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


@app.route('/selfcheck/selected')
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
    return jsonify(data)


class ServerThread(threading.Thread):
    def run(self):
        server = Server()
        server.start()
        while True:
            pass


# server_thread = ServerThread()
# server_thread.start()
app.run(host='0.0.0.0', debug=False)
