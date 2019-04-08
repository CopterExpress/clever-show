from __future__ import print_function
import os
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

import rospy
import pause

from .. import messaging_lib

from FlightLib import FlightLib
from FlightLib import LedLib

import play_animation

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
            else:
                break


def animation_player(running_event, stop_event):
    print("Animation thread activated")
    frames = play_animation.read_animation_file()
    if not frames:
        logging.error("Animation is empty, shutting down animation player")
        return

    delay_time = 0.125

    print("Takeoff")
    play_animation.takeoff(z=TAKEOFF_HEIGHT, safe_takeoff=SAFE_TAKEOFF)
    takeoff_time = starttime + TAKEOFF_TIME
    dt = takeoff_time - time.time()
    print("Wait until takeoff " + str(dt) + "s: " + time.ctime(takeoff_time))
    pause.until(takeoff_time)

    print("Reach first point")
    play_animation.reach_frame(frames[0], x0=X0+X0_COMMON, y0=Y0+Y0_COMMON) #Reach first point at the same time with others
    rfp_time = takeoff_time + RFP_TIME
    dt = rfp_time - time.time()
    print("Wait reaching first point " + str(dt) + "s: " + time.ctime(rfp_time))
    pause.until(rfp_time)

    next_frame_time = rfp_time
    print("Start animation at " + str(time.time()))
    for frame in frames:
        running_event.wait()
        play_animation.animate_frame(frame, x0=X0+X0_COMMON, y0=Y0+Y0_COMMON)
        next_frame_time += delay_time
        if stop_event.is_set():
            running_animation_event.clear()
            break
        #rate.sleep()
        pause.until(next_frame_time)
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
    return FlightLib.selfcheck()


def write_to_config(section, option, value):
    config.set(section, option, value)
    with open(CONFIG_PATH, 'w') as file:  # TODO as separate function
        config.write(file)


def load_config():
    global config, CONFIG_PATH
    global broadcast_port, port, host, BUFFER_SIZE
    global USE_NTP, NTP_HOST, NTP_PORT
    global files_directory, animation_file
    global FRAME_ID, TAKEOFF_HEIGHT, TAKEOFF_TIME, SAFE_TAKEOFF, RFP_TIME
    global  USE_LEDS, COPTER_ID
    global X0, X0_COMMON, Y0, Y0_COMMON
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

    FRAME_ID = config.get('COPTERS', 'frame_id') # TODO in play_animation
    TAKEOFF_HEIGHT = config.getfloat('COPTERS', 'takeoff_height')
    TAKEOFF_TIME = config.getfloat('COPTERS', 'takeoff_time')
    RFP_TIME = config.getfloat('COPTERS', 'reach_first_point_time')
    SAFE_TAKEOFF = config.getboolean('COPTERS', 'safe_takeoff')

    X0_COMMON = config.getfloat('COPTERS', 'x0_common')
    Y0_COMMON = config.getfloat('COPTERS', 'y0_common')
    X0 = config.getfloat('PRIVATE', 'x0')
    Y0 = config.getfloat('PRIVATE', 'y0')

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
                    recive_file(args['filename'])
                    if bool(args['clever_restart']):
                        os.system("systemctl restart clever")
                elif command == 'config_write':
                    write_to_config(args['section'], args['option'], args['value'])
                elif command == 'config_reload':
                    load_config()
                elif command == "starttime":
                    global starttime
                    starttime = float(args['time'])
                    print("Starting on:", time.ctime(starttime))
                    dt = starttime - time.time()
                    if USE_NTP:
                        dt = starttime - get_ntp_time(NTP_HOST, NTP_PORT)
                    print("Until start:", dt)
                    rospy.Timer(rospy.Duration(dt), start_animation, oneshot=True)
                elif command == 'takeoff':
                    play_animation.takeoff(safe_takeoff=SAFE_TAKEOFF)
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
                elif command == 'led_test':
                    LedLib.fill(255, 255, 255)
                    time.sleep(2)
                    LedLib.off()

                elif command == 'request':
                    request_target = args['value']
                    print("Got request for:", request_target)
                    response = ""
                    if request_target == 'test':
                        response = "test_success"
                    elif request_target == 'id':
                        response = COPTER_ID
                    elif request_target == 'selfcheck':
                        check = FlightLib.selfcheck()
                        response = check if check else "OK"
                    elif request_target == 'batt_voltage':
                        response = FlightLib.get_telemetry('body').voltage
                    elif request_target == 'cell_voltage':
                        response = FlightLib.get_telemetry('body').cell_voltage

                    send_all(bytes(form_message("response",
                                                {"status": "ok", "value": response, "value_name": str(request_target)})))
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

