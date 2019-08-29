from server import Server
from flask import Flask, render_template
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


server = Server()
server.start()
app.run(host='0.0.0.0', debug=False)
