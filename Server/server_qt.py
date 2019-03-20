from PyQt5 import QtWidgets
from PyQt5.QtGui import QStandardItem
from PyQt5.QtGui import QStandardItemModel
from PyQt5.QtCore import QModelIndex
from PyQt5.QtCore import Qt
from PyQt5.QtCore import pyqtSlot

from PyQt5.QtWidgets import QFileDialog

# Importing gui form
from server_gui import Ui_MainWindow


import os
import sys
import glob
import time
import json
import struct
import socket
import threading
import collections
import configparser

# All imports sorted in pyramid

# Functions


def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    my_ip = s.getsockname()[0]
    s.close()
    return my_ip


def auto_connect():
    while True:
        ServerSocket.listen(1)
        c, addr = ServerSocket.accept()
        print("Got connection from:", str(addr))
        if not any(client_addr == addr[0] for client_addr in Client.clients.keys()):
            client = Client(addr[0])
            print("New client")
        else:
            print("Reconnected client")
        Client.clients[addr[0]].connect(c, addr)


def ip_broadcast(server_ip, server_port):
    msg = bytes(Client.form_message("server_ip ", {"host": server_ip, "port": str(server_port)}), "UTF-8")
    broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    while True:
        broadcast_sock.sendto(msg, ('255.255.255.255', broadcast_port))
        print("Broadcast sent")
        time.sleep(10)


NTP_DELTA = 2208988800  # 1970-01-01 00:00:00
NTP_QUERY = b'\x1b' + bytes(47)


def get_ntp_time(ntp_host, ntp_port):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.sendto(NTP_QUERY, (ntp_host, ntp_port))
        msg, _ = s.recvfrom(1024)
    return int.from_bytes(msg[-8:], 'big') / 2 ** 32 - NTP_DELTA


def requires_connect(f):
    def wrapper(*args, **kwargs):
        if args[0].connected:
            return f(*args, **kwargs)
        else:
            print("Function requires client to be connected!")
    return wrapper


class Client:
    clients = {}

    def __init__(self, ip):
        self.socket = None
        self.addr = None

        self._send_queue = collections.deque()
        self._received_queue = collections.deque()
        self._request_queue = collections.OrderedDict()

        self.copter_id = None
        self.malfunction = False

        Client.clients[ip] = self

        self.connected = False

    def connect(self, client_socket, client_addr):
        print("Client connected")
        # self._send_queue = collections.deque()  # comment for resuming queue after reconnection

        self.socket = client_socket
        self.addr = client_addr

        self.socket.setblocking(0)
        self.connected = True
        client_thread = threading.Thread(target=self._run, args=())
        client_thread.start()
        if self.copter_id is None:
            self.copter_id = self.get_response("id")
            print("Got copter id:", self.copter_id)
            model.appendRow((QStandardItem(self.copter_id), ))  # TODO: get responses for another columns

    def _send_all(self, msg):
        self.socket.sendall(struct.pack('>I', len(msg)) + msg)

    def _receive_all(self, n):
        data = b''
        while len(data) < n:
            packet = self.socket.recv(min(n - len(data), BUFFER_SIZE))
            if not packet:
                return None
            data += packet
        return data

    def _receive_message(self):
        raw_msglen = self._receive_all(4)
        if not raw_msglen:
            return None
        msglen = struct.unpack('>I', raw_msglen)[0]
        msg = self._receive_all(msglen)
        return msg

    def _run(self):
        while self.connected:
            try:
                if self._send_queue:
                    msg = self._send_queue.popleft()
                    print("Send", msg, "to", self.addr)
                    try:
                        self._send_all(msg)
                    except socket.error as e:
                        print("Attempt to send failed")
                        self._send_queue.appendleft(msg)
                        raise e
                else:
                    msg = "ping"
                    # self._send_all(msg)

                try:  # check if data in buffer
                    check = self.socket.recv(BUFFER_SIZE, socket.MSG_PEEK)
                    if check:
                        received = self._receive_message()
                        if received:
                            received = received.decode("UTF-8")
                            print("Received", received, "from", self.addr)
                            command, args = Client.parse_message(received)
                            if command == "response":
                                for key, value in self._request_queue.items():
                                    if not value:
                                        self._request_queue[key] = args['value']
                                        print("Request successfully closed")
                                        break
                            else:
                                self._received_queue.appendleft(received)
                except socket.error:
                    pass

            except socket.error as e:
                print("Client error: {}, disconnected".format(e))
                self.connected = False
                self.socket.close()
                break
            # time.sleep(0.05)

    @staticmethod
    def form_message(command: str, dict_arguments: dict = None):
        if dict_arguments is None:
            dict_arguments = {}
        msg_dict = {command: str(dict_arguments).replace(",", '').replace("'", '')[1:-1]}
        msg = json.dumps(msg_dict)
        return msg

    @staticmethod
    def parse_message(msg):
        try:
            j_message = json.loads(msg)
        except json.decoder.JSONDecodeError:
            print("Json string not in correct format")
            return None

        str_command = list(j_message.keys())[0]

        arguments = list(j_message.values())[0].replace(":", '').split()
        dict_arguments = collections.OrderedDict(zip(arguments[::2], arguments[1::2]))
        return str_command, dict_arguments

    @requires_connect
    def send(self, *messages):
        for message in messages:
            self._send_queue.append(bytes(message, "UTF-8"))

    @requires_connect
    def get_response(self, requested_value):
        self._request_queue[requested_value] = ""
        self.send(Client.form_message("request", {"value": requested_value}))

        while not self._request_queue[requested_value]:
            pass

        return self._request_queue.pop(requested_value)

    @staticmethod
    def send_to_selected(*messages):
        if Client.clients:
            for client in Client.clients.values():  # TODO change to selected
                client.send(*messages)
        else:
            print("No clients were connected!")

    @staticmethod
    def request_to_selected(requested_value):
        if Client.clients:
            for client in Client.clients.values():  # TODO change to selected
                client.get_response(requested_value)
        else:
            print("No clients were connected!")

    @staticmethod
    def broadcast(message, force_all=False):
        if Client.clients:
            for client in Client.clients.values():
                if (not client.malfunction) or force_all:
                    client.send(message)
        else:
            print("No clients were connected!")

    @requires_connect
    def send_file(self, filepath, dest_filename):
        print("Sending file ", dest_filename)
        self.send(Client.form_message("writefile", {"filename": dest_filename}))
        file = open(filepath, 'rb')
        chunk = file.read(BUFFER_SIZE)
        while chunk:
            self._send_queue.append(chunk)  # TODO os.sendfile
            chunk = file.read(BUFFER_SIZE)
        file.close()
        self.send(Client.form_message("/endoffile"))
        print("File sent")


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.initUI()
        self.show()

    def initUI(self):
        #Connecting
        self.ui.check_button.clicked.connect(self.check_selected)
        self.ui.start_button.clicked.connect(self.send_starttime)
        self.ui.pause_button.clicked.connect(self.pause_all)
        self.ui.stop_button.clicked.connect(self.stop_all)
        self.ui.takeoff_button.clicked.connect(self.takeoff_selected)
        self.ui.land_button.clicked.connect(self.land_all)
        self.ui.disarm_button.clicked.connect(self.disarm_all)

        self.ui.action_send_animations.triggered.connect(self.send_animations)

        #Initing table and table model
        self.ui.tableView.setModel(model)
        self.ui.tableView.horizontalHeader().setStretchLastSection(True)

    @pyqtSlot()
    def check_selected(self):
        #Client.request_to_selected("selfcheck")
        self.ui.start_button.setEnabled(True)
        self.ui.takeoff_button.setEnabled(True)

    @pyqtSlot()
    def send_starttime(self):
        dt = self.ui.start_delay_spin.value()
        if USE_NTP:
            timenow = get_ntp_time(NTP_HOST, NTP_PORT)
        else:
            timenow = time.time()
        print('Now:', time.ctime(timenow), "+ dt =", dt)
        Client.send_to_selected(Client.form_message("starttime", {"time": str(timenow + dt)}))

    @pyqtSlot()
    def stop_all(self):
        Client.broadcast("stop")

    @pyqtSlot()
    def pause_all(self):
        if self.ui.pause_button.text() == 'Pause':
            Client.broadcast('pause')
            self.ui.pause_button.setText('Resume')
        else:
            Client.broadcast('resume')
        self.ui.pause_button.setText('Pause')

    @pyqtSlot()
    def takeoff_selected(self):
        Client.send_to_selected("takeoff")

    @pyqtSlot()
    def land_all(self):
        Client.broadcast("land")

    @pyqtSlot()
    def disarm_all(self):
        Client.broadcast("disarm")

    @pyqtSlot()
    def send_animations(self):
        path = str(QFileDialog.getExistingDirectory(self, "Select Animation Directory"))
        if path:
            print("Selected directory:", path)
            files = [file for file in glob.glob(path + '/*.csv')]
            names = [os.path.basename(file).split(".")[0] for file in files]
            print(files)
            for file, name in zip(files, names):
                for copter in Client.clients.values():
                    if name == copter.copter_id:
                        copter.send_file(file, "animation.csv")  # TODO config
                    else:
                        print("Filename not matches with any drone connected")
        # dr = next(iter(Client.clients.values()))  # костыль для тестирования
        # ANS = dr.get_response("someshit")
        # print(ANS)


model = QStandardItemModel()
model.setHorizontalHeaderLabels(
    ('copter ID', 'animation ID', 'battery V', 'battery %', 'selfcheck', 'time UTC')
)
model.setColumnCount(6)
model.setRowCount(0)

# Pre-initialization
# reading config
config = configparser.ConfigParser()
config.read("server_config.ini")

port = int(config['SERVER']['port'])
broadcast_port = int(config['SERVER']['broadcast_port'])
BUFFER_SIZE = int(config['SERVER']['buffer_size'])
USE_NTP = bool(config['NTP']['use_ntp'])
NTP_HOST = config['NTP']['host']
NTP_PORT = int(config['NTP']['port'])

ServerSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
ServerSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
host = socket.gethostname()
ip = get_ip_address()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()

    print('Server started on', host, ip, ":", port)

    if USE_NTP:
        now = get_ntp_time(NTP_HOST, NTP_PORT)
    else:
        now = time.time()
    print('Now:', time.ctime(now))

    print('Waiting for clients...')
    ServerSocket.bind((ip, port))

    autoconnect_thread = threading.Thread(target=auto_connect)
    autoconnect_thread.daemon = True
    autoconnect_thread.start()

    broadcast_thread = threading.Thread(target=ip_broadcast, args=(ip, port,))
    broadcast_thread.daemon = True
    broadcast_thread.start()

    sys.exit(app.exec_())
