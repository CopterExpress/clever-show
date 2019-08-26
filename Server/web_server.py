import os
import glob
import math
import time
import threading
from server import *
from flask import Flask

app = Flask(__name__)


@app.route('/')
def home():
    return 'I hope COEX:Swarm works<br/>' + str(Client.clients)


class ServerThread(threading.Thread):
    def run(self):
        server = Server()
        server.start()
        while True:
            pass


server_thread = ServerThread()
server_thread.start()
app.run(host='0.0.0.0', port=5000, debug=False)
