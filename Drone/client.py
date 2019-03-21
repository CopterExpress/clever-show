from __future__ import print_function
import sys
import socket
import struct
import random
import time
import errno
import json
import logging
import threading
import ConfigParser
from contextlib import closing
from collections import OrderedDict

import rospy

#from FlightLib.FlightLib import FlightLib
from FlightLib2 import FlightLib
from FlightLib.FlightLib import LedLib  # TODO new ledlib

import play_animation

random.seed()

logging.basicConfig(  # TODO all prints as logs
    level=logging.INFO,
    format="%(asctime)s [%(name)-7.7s] [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s",
    handlers=[
        logging.FileHandler("client_logs"),
        logging.StreamHandler()
    ])

NTP_PACKET_FORMAT = "!12I"
NTP_DELTA = 2208988800L # 1970-01-01 00:00:00
NTP_QUERY = '\x1b' + 47 * '\0'  


def get_ntp_time(ntp_host, ntp_port):
    with closing(socket.socket(socket.AF_INET, socket.SOCK_DGRAM)) as s:
        s.sendto(NTP_QUERY, (ntp_host, ntp_port))
        msg, address = s.recvfrom(1024)
    unpacked = struct.unpack(NTP_PACKET_FORMAT, msg[0:struct.calcsize(NTP_PACKET_FORMAT)])
    return unpacked[10] + float(unpacked[11]) / 2**32 - NTP_DELTA


def reconnect(timeout=2, attempt_limit=10):
    global clientSocket, host, port
    print("Trying to connect to", host, ":", port, "...")
    connected = False
    attempt_count = 0
    while not connected:
        print("Waiting for connection, attempt", attempt_count)
        try:
            clientSocket = socket.socket()
            clientSocket.settimeout(timeout)
            clientSocket.connect((host, port))
            connected = True
            print("Connection successful")
            clientSocket.settimeout(None)
        except socket.error as e:
            print("Waiting for connection, can not connect:", e)
            time.sleep(timeout)
        attempt_count += 1

        if attempt_count >= attempt_limit:
            print("Too many attempts. Trying to get new server IP")
            broadcast_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            broadcast_client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            broadcast_client.bind(("", broadcast_port))
            while True:
                data, addr = broadcast_client.recvfrom(1024)
                print("Received broadcast message %s from %s" % (data, addr))
                command, args = parse_message(data.decode("UTF-8"))
                if command == "server_ip":
                    host, port = args["host"], int(args["port"])
                    print("Binding to new IP: ", host, port)
                    broadcast_client.close()
                    attempt_count = 0
                    break

def send_all(msg):
    clientSocket.sendall(struct.pack('>I', len(msg)) + msg)


def recive_all(n):
    data = b''
    while len(data) < n:
        packet = clientSocket.recv(min(n - len(data), BUFFER_SIZE))
        if not packet:
            return None
        data += packet
    return data


def recive_message():
    raw_msglen = recive_all(4)
    if not raw_msglen:
        return None
    msglen = struct.unpack('>I', raw_msglen)[0]
    msg = recive_all(msglen)
    return msg


def form_message(str_command, dict_arguments):
    msg_dict = {str_command: str(dict_arguments).replace(",", '').replace("'", '')[1:-1]}
    msg = json.dumps(msg_dict)
    return msg


def parse_message(msg):
    try:
        j_message = json.loads(msg)
<<<<<<< HEAD
    except ValueError:
=======
    except json.JSONDecodeError:
>>>>>>> 32c16bb2060a6cc7e46d6cea84d07a0c86094270
        print("Json string not in correct format")
        return None, None

    str_command = list(j_message.keys())[0]

    arguments = list(j_message.values())[0].replace(":", '').split()
    dict_arguments = OrderedDict(zip(arguments[::2], arguments[1::2]))
    return str_command, dict_arguments


def recive_file(filename):
    print("Receiving file:", filename)
    with open(filename, 'wb') as file:  # TODO add directory
        while True:
            data = recive_message() #clientSocket.recv(BUFFER_SIZE)
            if data:
                print(data)
                if parse_message(data.decode("UTF-8"))[0] == "/endoffile":
                    print("File received")
                    break
                file.write(data)


def animation_player(running_event, stop_event):
    print("Animation thread activated")
    frames = play_animation.read_animation_file()
    rate = rospy.Rate(1000 / 125)
    play_animation.takeoff()
    play_animation.animate_frame(frames[0]) #Reach first point at the same time with others
    rospy.sleep(5)  # TODO change to flightlib reach_point
    for frame in frames:
        running_event.wait()
        if stop_event.is_set():
            running_animation_event.clear()
            break

        play_animation.animate_frame(frame)
        rate.sleep()
    else:
        play_animation.land()
        print("Animation ended")
    print("Animation thread closed")


stop_animation_event = threading.Event()
running_animation_event = threading.Event()


def start_animation(*args, **kwargs):
    animation_thread = threading.Thread(target=animation_player, args=(running_animation_event, stop_animation_event))
    print("Starting animation!")
    running_animation_event.set()
    stop_animation_event.clear()
    animation_thread.start()


def resume_animation():
    print("Resuming animation")
    running_animation_event.set()


def pause_animation():
    print("Pausing animation")
    running_animation_event.clear()


def stop_animation():
    stop_animation_event.set()
    print("Stopping animation")
#    animation_thread.join()


def selfcheck():
    telemetry = FlightLib.get_telemetry('body')
    return FlightLib.selfcheck(), telemetry.voltage


def write_to_config(section, option, value):
    config.set(section, option, value)
    with open(CONFIG_PATH, 'w') as file:
        config.write(file)


def load_config():
    global config, CONFIG_PATH
    global broadcast_port, port, host, BUFFER_SIZE
    global USE_NTP, NTP_HOST, NTP_PORT
    global files_directory, animation_file
    global TAKEOFF_HEIGHT, TAKEOFF_TIME
    global  USE_LEDS, COPTER_ID
    CONFIG_PATH = "client_config.ini"
    config = ConfigParser.ConfigParser()
    config.read(CONFIG_PATH)

    broadcast_port = config.getint('SERVER', 'broadcast_port')
    port = config.getint('SERVER', 'port')
    host = config.get('SERVER', 'host')
    BUFFER_SIZE = config.getint('SERVER', 'buffer_size')
    USE_NTP = config.getboolean('NTP', 'use_ntp')
    NTP_HOST = config.get('NTP', 'host')
    NTP_PORT = config.getint('NTP', 'port')

    files_directory = config.get('FILETRANSFER', 'files_directory')
    animation_file = config.get('FILETRANSFER', 'animation_file')

    TAKEOFF_HEIGHT = config.getfloat('COPTERS', 'takeoff_height')
    TAKEOFF_TIME = config.getint('COPTERS', 'takeoff_time')

    USE_LEDS = config.getboolean('PRIVATE', 'use_leds')
    play_animation.USE_LEDS = USE_LEDS

    COPTER_ID = config.get('PRIVATE', 'id')
    if COPTER_ID == 'default':
        COPTER_ID = 'copter' + str(random.randrange(9999)).zfill(4)
        write_to_config('PRIVATE', 'id', COPTER_ID)
    elif COPTER_ID == '/hostname':
        COPTER_ID = socket.gethostname()

load_config()

rospy.init_node('Swarm_client', anonymous=True)
if USE_LEDS:
    LedLib.init_led()

print("Client started on copter:", COPTER_ID)
if USE_NTP:
    print("NTP time:", time.ctime(get_ntp_time(NTP_HOST, NTP_PORT)))
print("System time", time.ctime(time.time()))

reconnect()

print("Connected to server")

try:
    while True:
        try:
            message = recive_message()
            if message:
                message = message.decode("UTF-8")
                command, args = parse_message(message)
                print("Command from server:", command, args)
                if command == "writefile":
<<<<<<< HEAD
                    recive_file(list(args.values())[0])
=======
                    recive_file(list(args.values)[0]) #TODO repace
>>>>>>> 32c16bb2060a6cc7e46d6cea84d07a0c86094270
                elif command == 'config_write':
                    write_to_config(args['section'], args['option'], args['value'])
                elif command == 'config_reload':
                    load_config()
                elif command == "starttime":
<<<<<<< HEAD
                    starttime = float(list(args.values())[0])
=======
                    starttime = float(list(args.values)[0]) #TODO repace
>>>>>>> 32c16bb2060a6cc7e46d6cea84d07a0c86094270
                    print("Starting on:", time.ctime(starttime))
                    dt = starttime - get_ntp_time(NTP_HOST, NTP_PORT)
                    print("Until start:", dt)
                    rospy.Timer(rospy.Duration(dt), start_animation, oneshot=True)
                elif command == 'takeoff':
                    play_animation.takeoff()
                elif command == 'pause':
                    pause_animation()
                elif command == 'resume':
                    resume_animation()
                elif command == 'stop':
                    stop_animation()
                    FlightLib.interrupt()
                elif command == 'land':
                    play_animation.land()
                elif command == 'disarm':
                    FlightLib.arming(False)

                elif command == 'request':
                    request_target = args['value']
                    print("Got request for:", request_target)
                    response = ""
                    if request_target == 'test':
                        response = "test_success"
                    elif request_target == 'id':
                        response = COPTER_ID
                    elif request_target == 'selfcheck':
                        response = selfcheck()
                    send_all(bytes(form_message("response", {"status": "ok", "value": response})))
                    print("Request responded with:",  response)
        except socket.error as e:
            if e.errno != errno.EINTR:
                print("Connection lost due error:", e)
                print("Reconnecting...")
                reconnect()
                print("Re-connection successful")
            else:
                print("Interrupted")
                raise KeyboardInterrupt
except KeyboardInterrupt:
    print("Shutdown on keyboard interrupt")
finally:
    clientSocket.close()

