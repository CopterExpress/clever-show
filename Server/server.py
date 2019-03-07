from tkinter import *
from tkinter import ttk
from tkinter import filedialog
import ttkwidgets

import os
import sys
import glob
import time
import struct
import socket
import threading
import collections
import configparser

# All imports sorted in pyramid


def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    ip = s.getsockname()[0]
    s.close()
    return ip


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


def ip_broadcast(ip):
    ip = ip
    broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    while True:
        msg = bytes(Client.form_command("server_ip ", ip), "UTF-8")
        broadcast_sock.sendto(msg, ('255.255.255.255', 8181))  #TODO to config
        time.sleep(5)


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
            drone_list.insert("", "end", self.addr[0], text=self.copter_id)

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
                            print("Recived", received, "from", self.addr)
                            command, args = Client.parse_command(received)
                            if command == "response":
                                for key, value in self._request_queue.items():
                                    if not value:
                                        self._request_queue[key] = args[0]
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
    def form_command(command: str, args=()):  # Change for different protocol
        return " ".join([command, *args])

    @staticmethod
    def parse_command(command_input):
        args = command_input.split()
        command = args.pop(0)
        return command, args

    @requires_connect
    def send(self, *messages):
        for message in messages:
            self._send_queue.append(bytes(message, "UTF-8"))

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
        self.send(Client.form_command("writefile", (dest_filename,)))
        file = open(filepath, 'rb')
        chunk = file.read(BUFFER_SIZE)
        while chunk:
            self._send_queue.append(chunk)
            chunk = file.read(BUFFER_SIZE)
        file.close()
        self.send(Client.form_command("/endoffile"))
        print("File sent")

    @requires_connect
    def get_response(self, requested_value):
        self._request_queue[requested_value] = ""
        self.send(Client.form_command("request", (requested_value, )))

        while not self._request_queue[requested_value]:
            pass

        return self._request_queue.pop(requested_value)


# UI functions
def stop_swarm():
    Client.broadcast("stop")  # для тестирования


def land_all():
    Client.broadcast("land")


def disarm_all():
    Client.broadcast("disarm")


def takeoff_all():
    Client.broadcast("takeoff")


def send_animations():
    path = filedialog.askdirectory(title="Animation directory")
    if path:
        print("Selected directory:", path)
        files = [file for file in glob.glob(path+'/*.csv')]
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


def send_starttime(dt=15):
    timenow = time.time()
    print('Now:', time.ctime(timenow), "+ dt =", dt)
    Client.broadcast(Client.form_command("starttime", (str(timenow+dt), )))


# UI build here
root = Tk()
root.wm_title("Drone swarm operation server")
root.style = ttk.Style()
root.style.theme_use("default")

leftFrame = Frame(root)
leftFrame.grid(row=0, column=0, padx=10, pady=10)
rightFrame = Frame(root)
rightFrame.grid(row=0, column=1, padx=10, pady=10)

drone_list = ttkwidgets.CheckboxTreeview(leftFrame, columns=("addr", "connected"))
# drone_list["columns"] = ("addr")
# drone_list.column("name") #width=100
# drone_list.column("addr")
drone_list.heading("#0", text="Drone name")
drone_list.heading("#1", text="Connection adress")
drone_list.heading("#2", text="Connection status")
drone_list.pack()

button_frame = Frame(leftFrame, borderwidth=1, relief="solid")
button_frame.pack(fill=BOTH, expand=True)

land_all_btn = ttk.Button(button_frame, text="Disarm all", command=disarm_all)
land_all_btn.pack(side=RIGHT, padx=5, pady=5)

land_all_btn = ttk.Button(button_frame, text="Land all", command=land_all)
land_all_btn.pack(side=RIGHT, padx=5, pady=5)

stop_all_btn = ttk.Button(button_frame, text="Stop swarm", command=stop_swarm)
stop_all_btn.pack(side=RIGHT, padx=5, pady=5)


send_animation_btn = ttk.Button(button_frame, text="Send animations", command=send_animations)
send_animation_btn.pack(side=LEFT, padx=5, pady=5)

send_starttime_btn = Button(button_frame, bg='red', text="Takeoff all", command=takeoff_all)
send_starttime_btn.pack(side=LEFT, padx=5, pady=5)

send_starttime_btn = ttk.Button(button_frame, text="Start animation after...", command=send_starttime)
send_starttime_btn.pack(side=LEFT, padx=5, pady=5)


def gui_update():
    time.sleep(0.1)


# reading config
config = configparser.ConfigParser()
config.read("server_config.ini")

port = int(config['SERVER']['port'])
BUFFER_SIZE = int(config['SERVER']['buffer_size'])
NTP_HOST = config['NTP']['host']
NTP_PORT = int(config['NTP']['port'])


ServerSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
ServerSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
host = socket.gethostname()
ip = get_ip_address()

print('Server started on', host, ip, ":", port)
# print('Now:', time.ctime(get_ntp_time(NTP_HOST, NTP_PORT)))
print('Waiting for clients...')
ServerSocket.bind((ip, port))

autoconnect_thread = threading.Thread(target=auto_connect)
autoconnect_thread.daemon = True
autoconnect_thread.start()

broadcast_thread = threading.Thread(target=ip_broadcast, args=(ip, port, ))

if __name__ == '__main__':
    try:
        mainloop()
    except KeyboardInterrupt:
        print("Stopping server by keyboard interrupt")
    finally:
        ServerSocket.close()
        print("Server shutdown")
