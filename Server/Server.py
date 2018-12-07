# -*- coding: utf-8 -*- .
"""Code by Alexandr Osherov 10 class, phone - +79251834732,  email - allexandr2001@mail.ru """
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import time
import socket
import os
import easygui
import threading
from threading import Thread
import math
import requests
import json
import main_gui
import gui_telem
import ntplib
ip = [1, 2]
sq_rad = 0
sq_cet = 0
cr_rad = 0
cr_cet = 0
copters = 1
conn = []
conn_2 = []
file = ''
data = b''
addr = []
addr_2 = []
coord = []
d_time = 0
size_scene=[2,2,2]
sock = socket.socket()

sock.bind(('', 35001))  # назначается адресс и порт связи для отпраки команд на коптеры
sock.listen(1)

sock_2 = socket.socket()
sock_2.bind(('', 35002))  # назначается адресс и порт связи для приема данных с коптеров
sock_2.listen(1)


class Dialog(QMainWindow, gui_telem.Ui_Dialog):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

    def up(self):
        self.voltage_2.setText('1234567')


class Widget(QMainWindow, main_gui.Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.start_animation_button.clicked.connect(self.start_animation)
        self.stop_swarm_but.clicked.connect(self.stop_swarm)
        self.show_3d_scene_button.clicked.connect(self.show_3d)
        self.disarm_all_button.clicked.connect(self.disarm)
        self.turn_off_led_button.clicked.connect(self.off_leds)
        self.turn_on_led_button.clicked.connect(self.on_leds)
        self.upload_animation_button.clicked.connect(self.upload_animation)
        self.land_all_button.clicked.connect(self.land)
        self.take_off_button.clicked.connect(self.take_off)
        #self.number_animation_copters.clicked.connect(self.number_animation)   synch_button
        self.synch_button.clicked.connect(self.synch)
        self.take_off_n_button.clicked.connect(self.take_off_n)
        self.land_n_button.clicked.connect(self.land_n)
        self.disarm_n_button.clicked.connect(self.disarm_n)
        self.land_spinBox.valueChanged.connect(self.land_led)
        self.disarm_spinBox.valueChanged.connect(self.disarm_led)
        self.take_off_spinBox.valueChanged.connect(self.take_off_led)
        self.safty_button.clicked.connect(self.safty)
        self.connect_button.clicked.connect(self.connect)
        self.swarm_size_spinBox.valueChanged.connect(self.number_copters)

    

    def receiver(self):
        global copters
        global addr
        while True:
            try:
                for k in range(copters): 
                    a = requests.get('http://' + addr[k][0] + ':8081/aruco_map')
                    tem = json.loads(a.text)

                    coord[k] = str(tem['x']) + ',' + str(tem['y']) + ',' + str(tem['z']) + ',' + \
                               str(tem['mode']) + ',' + str(tem['armed']) + ',' + str(tem['frame_id']) + str(
                        tem['voltage']) + ',' + \
                               str(tem['yaw']) + ',' + str(tem['pitch']) + ',' + str(tem['roll']) + ',' + \
                               str(tem['vx']) + ',' + str(tem['vy']) + ',' + str(tem['vz'])

                    #time.sleep(0.05)


            except Exception as e:
                pass

    def sender(self, com, num):
        global conn
        global conn_2
        global copters
        print(com)
        print(num)
        try:
            if num == 'all':
                for i in range(copters):
                    conn[i].send(com + b'$$')
                    print(com + b'$$')
            elif int(num) > 0:
                conn[int(num) - 1].send(com + b'$$')
        except:
            pass

    def message(self, mes):
        pass

    def connect(self):
        global copters
        global conn
        global conn_2
        global addr
        global addr_2
        global coord

        addr_2 = []

        conn = []
        conn_2 = []

        self.message('Try connect')
        for i in range(copters):
            conn.append(0)
            addr.append(0)
            coord.append('0')

        t_0 = Thread(target=self.connect_init)
        t_0.daemon = True
        t_0.start()

    def connect_init(self):
        self.state_label.setText("<html><head/><body><p><span style=\" font-size:12pt; color:rgb(255,255,"
                                 "0);\">Wait</span></p></body></html>")
        global copters

        global conn
        global addr
        global conn_2
        global addr_2

        for i in range(copters):
            conn[i], addr[i] = sock.accept()
            print("connected_controllers:", addr[i])

        self.disarm_spinBox.setMaximum(copters)
        self.land_spinBox.setMaximum(copters)
        self.take_off_spinBox.setMaximum(copters)
        self.state_label.setText("<html><head/><body><p><span style=\" font-size:12pt; "
                                 "color:Green;\">Connect</span></p></body></html>")

        t = Thread(target=self.receiver)
        t.daemon = True
        t.start()

    def number_animation(self):
        global copters
        col = [b'(255,0,0)', b'(173,255,47)', b'(255,215,0)', b'(255,0,255)', b'(0,0,255)', b'(205,92,92)',
               b'(255,255,255)', b'(240,128,128)']
        for i in range(copters + 1):
            self.sender(b'led.fill' + col[i], str(i))
            time.sleep(0.5)
            self.sender(b'led.off()', str(i))
        for i in range(copters + 1):
            self.sender(b'led.fill' + col[i], str(i))

            time.sleep(0.5)

    def safty(self):
        self.message('safty check')
        self.sender(b'f.safety_check(False)', 'all')

    def take_off(self):
        self.message('take off')
        self.sender(b'f.takeoff(z=2, speed=2,speed_takeoff=2 , timeout=2)', 'all')

    def take_off_n(self):
        self.message('take off n')
        self.sender(b'f.takeoff()', str(self.take_off_spinBox.value()))

    def land(self):
        self.message('land')
        self.sender(b'f.land(preland=False)', 'all')

    def land_n(self):
        self.message('land n')
        self.sender(b'f.land(preland=False)', str(self.land_spinBox.value()))

    def disarm(self):
        self.sender(b'f.arming(False)', 'all')

    def disarm_n(self):
        self.sender(b'f.arming(False)', str(self.disarm_spinBox.value()))

    def off_leds(self):
        self.sender(b'led.off()', 'all')

    def land_led(self):
        self.sender(b'led.off()', 'all')
        self.sender(b'led.fill(255,0,0)', self.land_spinBox.value())
        # print(1)

    def disarm_led(self):
        self.sender(b'led.off()', 'all')
        self.sender(b'led.fill(0,255,0)', self.disarm_spinBox.value())

    def take_off_led(self):
        self.sender(b'led.off()', 'all')
        self.sender(b'led.fill(0,0,255)', self.take_off_spinBox.value())

    def on_leds(self):
        self.sender(b'led.fill(0,0,255)', 'all')

    def number_copters(self):
        global copters
        copters = self.swarm_size_spinBox.value()
    
    def upload_animation(self):
        global file
        file = easygui.fileopenbox(filetypes=["*.csv"],multiple=True)  # вызов окна проводника для выбора файла
        
    def start_animation(self):
        global file
        for counter, sub_file in enumerate(file):
            f = open(sub_file, 'r')
            prog = f.read()
            self.sender(b'programm' + bytes(prog, 'utf-8')+b'stop', str(counter))

    def stop_swarm(self):
        pass

    def synch(self):
        global d_time
        self.sender(b'synch', 'all')
        c = ntplib.NTPClient()
        response = c.request('ntp1.stratum2.ru')
        d_time =  response.tx_time-time.time()
        print(d_time)

    def show_3d(self):
        global size_scene
        global data
        global copters
        global coord
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        updateDialog = Dialog()
        updateDialog.show()

        while True:
            s = str(self.console_textEdit.toPlainText())
            print(s)
            if '>' in s:

                print(s[:-2], s[s.index('>') - 1])
                self.console_textEdit.setText('')
                if s[s.index('>') - 1] == '0':
                    self.sender(bytes(str(s[:s.index('>') - 1]), 'utf-8'), 'all')
                    print(s[:s.index('>') - 1], 'all')
                else:
                    self.sender(bytes(str(s[:s.index('>') - 1]), 'utf-8'), str(s[s.index('>') - 1]))
                    print('sender', s[s.index('>') - 1])
            try:

                i = updateDialog.number_spinBox.value()
                if i > 0:
                    i -= 1

                coord_drone = []
                coord_drone = coord[i].split(',')

                updateDialog.z_2.setText("<html><head/><body><p><span style=\" color:#c8c8c8;\">" + coord_drone[
                    0] + "</span></p></body></html>")
                updateDialog.x_2.setText("<html><head/><body><p><span style=\" color:#c8c8c8;\">" + coord_drone[
                    1] + "</span></p></body></html>")
                updateDialog.y_2.setText("<html><head/><body><p><span style=\" color:#c8c8c8;\">" + coord_drone[
                    2] + "</span></p></body></html>")
                updateDialog.mode_2.setText("<html><head/><body><p><span style=\" color:#c8c8c8;\">" + coord_drone[
                    3] + "</span></p></body></html>")
                updateDialog.armed_2.setText("<html><head/><body><p><span style=\" color:#c8c8c8;\">" + coord_drone[
                    4] + "</span></p></body></html>")
                updateDialog.frame_id_2.setText("<html><head/><body><p><span style=\" color:#c8c8c8;\">" + coord_drone[
                    5] + "</span></p></body></html>")
                updateDialog.voltage_2.setText("<html><head/><body><p><span style=\" color:#c8c8c8;\">" + coord_drone[
                    6] + "</span></p></body></html>")
                updateDialog.yaw2.setText("<html><head/><body><p><span style=\" color:#c8c8c8;\">" + coord_drone[
                    7] + "</span></p></body></html>")
                updateDialog.pitch_2.setText("<html><head/><body><p><span style=\" color:#c8c8c8;\">" + coord_drone[
                    8] + "</span></p></body></html>")
                updateDialog.roll_2.setText("<html><head/><body><p><span style=\" color:#c8c8c8;\">" + coord_drone[
                    9] + "</span></p></body></html>")
                updateDialog.vx_2.setText("<html><head/><body><p><span style=\" color:#c8c8c8;\">" + coord_drone[
                    10] + "</span></p></body></html>")
                updateDialog.vy_2.setText("<html><head/><body><p><span style=\" color:#c8c8c8;\">" + coord_drone[
                    11] + "</span></p></body></html>")
                updateDialog.vz_2.setText("<html><head/><body><p><span style=\" color:#c8c8c8;\">" + coord_drone[
                    12] + "</span></p></body></html>")
            except Exception as e:
                # print(e)
                pass
            try:

                n = 0
                # set size of scene

                ax.set_xlim(0, size_scene[0])
                ax.set_ylim(0, size_scene[1])
                ax.set_zlim(0, size_scene[2])

                ax.set_xlabel('x')
                ax.set_ylabel('y')
                ax.set_zlabel('z')

                plt.pause(0.01)

                ax.clear()
                try:
                    for i in coord:
                        co = (0, 0, 0)
                        n += 1

                        if self.land_spinBox.value() == n:
                            co = (1, 0, 0)
                        if self.take_off_spinBox.value() == n:
                            co = (1, 0, 0)
                        if self.disarm_spinBox.value() == n:
                            co = (1, 0, 0)

                        ax.scatter(float(i.split(',')[0]), float(i.split(',')[1]), float(i.split(',')[2]), s=50, c=co,
                                   marker='.')
                        ax.text(float(i.split(',')[0]), float(i.split(',')[1]), float(i.split(',')[2]), str(n), size=10,
                                zorder=1, color=(0, 0, 0))

                except Exception as e:
                    print(e)

                ax.set_xlim(0, size_scene[0])
                ax.set_ylim(0, size_scene[1])
                ax.set_zlim(0, size_scene[2])


                ax.set_xlabel('x')
                ax.set_ylabel('y')
                ax.set_zlabel('z')

                plt.draw()
            except KeyboardInterrupt:
                print('stop')
                break

        plt.show()



app = QApplication([])
w = Widget()
w.show()

app.exec()


