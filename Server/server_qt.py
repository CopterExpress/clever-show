import os
import re
import sys
import glob
import math
import time
import logging
import asyncio
import itertools
from functools import partial, wraps

from PyQt5 import QtWidgets, QtMultimedia, QtCore
from PyQt5.QtGui import QPixmap, QIcon
from PyQt5.QtCore import Qt, pyqtSlot, QUrl

from PyQt5.QtWidgets import QFileDialog, QMessageBox, QApplication, QInputDialog, QLineEdit, QStatusBar, \
    QSplashScreen, QProgressBar
from quamash import QEventLoop

# Importing gui form
from server_gui import Ui_MainWindow

from server import Server, Client, now
import messaging_lib as messaging
import config as cfg

import copter_table_models as table
from copter_table import CopterTableWidget
#from emergency import *
#  TODO uncomment


def multi_glob(*patterns):
    return itertools.chain.from_iterable(glob.iglob(pattern) for pattern in patterns)


def confirmation_required(text="Are you sure?", label="Confirm operation?"):
    def inner(f):
        @wraps(f)
        def wrapper(*args, **kwargs):
            reply = QMessageBox.question(
                args[0], label,
                text,
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            )
            if reply == QMessageBox.Yes:
                logging.debug("Dialog accepted")
                return f(*args, **kwargs)

            logging.debug("Dialog declined")

        return wrapper

    return inner


class ExitMsgbox(logging.Handler):
    def emit(self, record):
        loop.call_soon_threadsafe(self._emit, record)  # TODO replace loop call

    def _emit(self, record):
        # window.close()
        QMessageBox.warning(None, "Critical error in {}: {}". format(record.name, record.threadName), record.msg)
        QApplication.quit()
        sys.exit(record.msg)


class ServerQt(Server):
    def load_config(self):
        super().load_config()
        table.ModelChecks.battery_min = self.config.checks_battery_min
        table.ModelChecks.start_pos_delta_max = self.config.checks_start_pos_delta_max
        table.ModelChecks.time_delta_max = self.config.checks_time_delta_max


# noinspection PyCallByClass,PyArgumentList
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, server):
        super(MainWindow, self).__init__()

        self.server = server
        self.model = table.CopterDataModel()

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.init_ui()

        self.init_model()

        # self.statusBar = QStatusBar()
        # self.setStatusBar(self.statusBar)
        # self.statusBar.showMessage("Hey", 2000)

        self.player = QtMultimedia.QMediaPlayer()

    def init_ui(self):
        # Connecting
        self.ui.check_button.clicked.connect(self.selfcheck_selected)
        self.ui.start_button.clicked.connect(self.send_start_time_selected)
        self.ui.pause_button.clicked.connect(self.pause_resume_selected)
        self.ui.stop_button.clicked.connect(self.land_all)

        self.ui.emergency_button.clicked.connect(self.emergency)
        self.ui.disarm_button.clicked.connect(partial(self.send_to_selected, "disarm"))
        self.ui.disarm_all_button.clicked.connect(self.disarm_all)

        self.ui.leds_button.clicked.connect(partial(self.send_to_selected, "led_test"))
        self.ui.takeoff_button.clicked.connect(self.takeoff_selected)
        self.ui.flip_button.clicked.connect(self.flip_selected)
        self.ui.land_button.clicked.connect(partial(self.send_to_selected, "land"))

        self.ui.reboot_fcu.clicked.connect(partial(self.send_to_selected, "reboot_fcu"))
        self.ui.calibrate_gyro.clicked.connect(self.calibrate_gyro_selected)
        self.ui.calibrate_level.clicked.connect(self.calibrate_level_selected)

        self.ui.action_remove_row.triggered.connect(self.remove_selected)

        self.ui.action_send_animations.triggered.connect(self.send_animations)
        self.ui.action_send_calibrations.triggered.connect(self.send_calibrations)
        self.ui.action_send_configurations.triggered.connect(self.send_config)
        self.ui.action_send_Aruco_map.triggered.connect(self.send_aruco)
        self.ui.action_send_launch_file.triggered.connect(self.send_launch)
        self.ui.action_send_fcu_parameters.triggered.connect(self.send_fcu_parameters)
        self.ui.action_send_any_file.triggered.connect(self.send_any_file)
        self.ui.action_send_any_command.triggered.connect(self.send_any_command)
        self.ui.action_restart_clever.triggered.connect(
            partial(self.send_to_selected, "service_restart", {"name": "clever"}))
        self.ui.action_restart_clever_show.triggered.connect(
            partial(self.send_to_selected, "service_restart", {"name": "clever-show"}))
        self.ui.action_update_client_repo.triggered.connect(partial(self.send_to_selected, "update_repo"))
        self.ui.action_reboot_all.triggered.connect(partial(self.send_to_selected, "reboot_all"))
        self.ui.action_set_start_to_current_position.triggered.connect(partial(self.send_to_selected, "move_start"))
        self.ui.action_reset_start.triggered.connect(partial(self.send_to_selected, "reset_start"))
        self.ui.action_set_z_offset_to_ground.triggered.connect(partial(self.send_to_selected, "set_z_to_ground"))
        self.ui.action_reset_z_offset.triggered.connect(partial(self.send_to_selected, "reset_z_offset"))
        self.ui.action_restart_chrony.triggered.connect(partial(self.send_to_selected, "repair_chrony"))
        self.ui.action_select_music_file.triggered.connect(self.select_music_file)
        self.ui.action_play_music.triggered.connect(self.play_music)
        self.ui.action_stop_music.triggered.connect(self.stop_music)

        self.init_table()

        # Set most safety-important buttons disabled
        self.ui.start_button.setEnabled(False)
        self.ui.takeoff_button.setEnabled(False)
        self.ui.flip_button.setEnabled(False)

    def init_table(self):
        # remove standard table widget
        self.ui.horizontalLayout.removeWidget(self.ui.tableView)
        self.ui.tableView.close()

        # init our custom widget
        self.ui.copter_table = CopterTableWidget(self.model)
        self.ui.copter_table.setObjectName("copter_table")

        # add to layout
        self.ui.horizontalLayout.addWidget(self.ui.copter_table, 0)

    def init_model(self):
        # self.model.on_id_changed = self.set_copter_id

        # Connect model signals to UI
        self.model.selected_ready_signal.connect(self.ui.start_button.setEnabled)
        self.model.selected_takeoff_ready_signal.connect(self.ui.takeoff_button.setEnabled)
        self.model.selected_flip_ready_signal.connect(self.ui.flip_button.setEnabled)
        # Connect calibrating signal (testing)
        self.model.selected_calibrating_signal.connect(self.ui.check_button.setDisabled)
        self.model.selected_calibrating_signal.connect(self.ui.pause_button.setDisabled)
        self.model.selected_calibrating_signal.connect(self.ui.stop_button.setDisabled)
        self.model.selected_calibrating_signal.connect(self.ui.emergency_button.setDisabled)
        self.model.selected_calibrating_signal.connect(self.ui.disarm_button.setDisabled)
        self.model.selected_calibrating_signal.connect(self.ui.disarm_all_button.setDisabled)
        self.model.selected_calibrating_signal.connect(self.ui.leds_button.setDisabled)
        self.model.selected_calibrating_signal.connect(self.ui.land_button.setDisabled)
        self.model.selected_calibrating_signal.connect(self.ui.reboot_fcu.setDisabled)
        self.model.selected_calibration_ready_signal.connect(self.ui.calibrate_gyro.setEnabled)
        self.model.selected_calibration_ready_signal.connect(self.ui.calibrate_level.setEnabled)

        self.ui.action_select_all_rows.triggered.connect(self.model.select_all)

    def iterate_selected(self, f, *args, **kwargs):
        for copter in self.model.user_selected():
            yield f(copter, *args, **kwargs)

    @pyqtSlot()
    def send_to_selected(self, *args, **kwargs):
        return list(self.iterate_selected(lambda copter: copter.client.send_message(*args, **kwargs)))

    def new_client_connected(self, client: Client):
        logging.debug("Added client {}".format(client))
        self.ui.copter_table.add_client(copter_id=client.copter_id, client=client)

    def client_connection_changed(self, client: Client):
        logging.debug("Connection {} changed {}".format(client, client.connected))
        row_data = self.model.get_row_by_attr("client", client)

        if row_data is None:
            logging.error("No row for client presented")
            return

        if self.server.config.table_remove_disconnected and (not client.connected):
            client.remove()
            self.ui.copter_table.remove_client_data(row_data)
            logging.debug("Removing from table")
        else:
            row_num = self.model.get_row_index(row_data)
            if row_num is not None:
                self.ui.copter_table.update_data(row_num, 0, client.connected, table.ModelStateRole)
                logging.debug("Client status updated")

    @pyqtSlot()
    def selfcheck_selected(self):
        for copter_data_row in self.model.user_selected():
            client = copter_data_row.client
            client.get_response("telemetry", self.update_table_data)

    @pyqtSlot(object, dict)
    def update_table_data(self, client, telems: dict):
        cols_dict = {
            "git_version": 1,
            "animation_id": 2,
            "battery": 3,
            "system_status": 4,
            "calibration_status": 5,
            "mode": 6,
            "selfcheck": 7,
            "current_position": 8,
            "start_position": 9,
            "time": 10,
        }

        for key, value in telems.items():
            col = cols_dict.get(key, None)
            if col is None:
                logging.error("No column {} present!".format(key))
                continue

            row_data = self.model.get_row_by_attr("client", client)
            row_num = self.model.get_row_index(row_data)
            if row_num is not None:
                self.ui.copter_table.update_data(row_num, col, value, Qt.EditRole)

    @pyqtSlot()
    def remove_selected(self):
        for copter in self.model.user_selected():
            copter.client.remove()
            if not self.server.config.table_remove_disconnected:
                self.ui.copter_table.remove_client_data(copter)
            logging.info("Client removed from table!")

    @pyqtSlot()
    @confirmation_required("This operation will takeoff selected copters with delay and start animation. Proceed?")
    def send_start_time_selected(self):
        time_now = server.time_now()
        dt = self.ui.start_delay_spin.value()
        logging.info('Wait {} seconds to start animation'.format(dt))
        if self.ui.music_checkbox.isChecked():
            music_dt = self.ui.music_delay_spin.value()
            asyncio.ensure_future(self.play_music_at_time(music_dt + time_now), loop=loop)
            logging.info('Wait {} seconds to play music'.format(music_dt))
        # self.selfcheck_selected()
        for copter in filter(self.model.checks.all_checks, self.model.user_selected()):
            server.send_starttime(copter.client, dt + time_now)

    @pyqtSlot()
    def pause_resume_selected(self):
        if self.ui.pause_button.text() == 'Pause':
            self.send_to_selected("pause")
            self.ui.pause_button.setText('Resume')
        else:
            time_gap = 0.1  # TODO config? automatic delay detection?
            self.send_to_selected("resume", {"time": server.time_now() + time_gap})
            self.ui.pause_button.setText('Pause')

    @pyqtSlot()
    def land_all(self):
        Client.broadcast_message("land")

    @pyqtSlot()
    def disarm_all(self):
        Client.broadcast_message("disarm")

    @pyqtSlot()
    @confirmation_required("This operation will takeoff copters immediately. Proceed?")
    def takeoff_selected(self):
        for copter in self.model.user_selected():
            if self.model.checks.takeoff_checks(copter):
                if self.ui.z_checkbox.isChecked():
                    copter.client.send_message("takeoff_z", {"z": str(self.ui.z_spin.value())})  # todo int
                else:
                    copter.client.send_message("takeoff")

    @pyqtSlot()
    @confirmation_required("This operation will flip(!!!) copters immediately. Proceed?")
    def flip_selected(self):
        for copter in self.model.user_selected():
            if table.flip_checks(copter):
                copter.client.send_message("flip")

    @pyqtSlot()
    def calibrate_gyro_selected(self):  # TODO merge commands
        for copter_data_row in self.model.user_selected():
            client = copter_data_row.client
            # Update calibration status
            row = self.model.get_row_index(copter_data_row)
            col = 5
            data = 'CALIBRATING'
            self.signals.update_data_signal.emit(row, col, data, table.ModelDataRole)
            # Send request
            client.get_response("calibrate_gyro", self._get_calibration_info)

    @pyqtSlot()
    def calibrate_level_selected(self):
        for copter_data_row in self.model.user_selected():
            client = copter_data_row.client
            # Update calibration status
            row = self.model.get_row_index(copter_data_row)
            col = 5
            data = 'CALIBRATING'
            self.signals.update_data_signal.emit(row, col, data, table.ModelDataRole)
            # Send request
            client.get_response("calibrate_level", self._get_calibration_info)

    def _get_calibration_info(self, client, value):
        col = 5
        row_data = self.model.get_row_by_attr("client", client)
        row = self.model.get_row_index(row_data)
        if row is not None:
            data = str(value)
            self.signals.update_data_signal.emit(row, col, data, table.ModelDataRole)

    def _send_files(self, files, copters=None, client_path="", client_filename="", match_id=False, callback=None):
        if copters is None:
            copters = self.model.user_selected()
        copters = list(copters)

        for num, file in enumerate(files):
            filepath, filename = os.path.split(file)
            logging.info("Preparing file for sending: {} {}".format(filepath, filename))

            if match_id:
                name = os.path.splitext(filename)[0]
                to_send = [copter for copter in copters if re.fullmatch(name, copter.copter_id)]
            else:
                to_send = copters

            if not to_send:
                logging.warning(f"No copters to send file {filename} to")
                continue

            logging.info(f"Sending file {filename} to clients: {to_send}")
            filename = client_filename.format(num, filename) or filename

            for copter in to_send:
                copter.client.send_file(file, os.path.join(client_path, filename))
                if callback is not None:
                    callback(copter)

    def send_files(self, prompt, ext_filter, copters=None, client_path="", client_filename="", match_id=False,
                   onefile=False, callback=None):
        if onefile:
            file = QFileDialog.getOpenFileName(self, prompt, filter=ext_filter)[0]
            files = [file] if file else []
        else:
            files = QFileDialog.getOpenFileNames(self, prompt, filter=ext_filter)[0]

        if not files:
            return

        self._send_files(files, copters, client_path, client_filename, match_id, callback)

    def send_directory_files(self, prompt, extensions=(), copters=None, client_path="", client_filename="",
                             match_id=False, callback=None):
        path = QFileDialog.getExistingDirectory(self, prompt)

        if not path:
            return

        if extensions:
            patterns = [path + '/*' + ext for ext in extensions]
        else:
            patterns = [path+'/*.*']

        files = multi_glob(*patterns)
        self._send_files(files, copters, client_path, client_filename, match_id, callback)

    @pyqtSlot()
    def send_any_file(self):
        file = QFileDialog.getOpenFileName(self, "Select any file")[0]
        if not file:
            return

        c_path, ok = QInputDialog.getText(self, "Enter path (and name) to send on client", "Destination:",
                                          QLineEdit.Normal, "/home/pi/")   # TODO config?
        if not ok:
            return

        c_filename, c_filepath = os.path.split(c_path)
        files = [file]
        self._send_files(files, client_path=c_filepath, client_filename=c_filename)

    @pyqtSlot()
    def send_animations(self):
        self.send_directory_files("Select directory with animations", ('.csv', '.txt'), match_id=True,
                                  client_path="", client_filename="animation.csv")

    @pyqtSlot()
    def send_calibrations(self):
        self.send_directory_files("Select directory with calibrations", ('.yaml', ), match_id=True,
                                  client_path="/home/pi/catkin_ws/src/clever/clever/camera_info/",
                                  client_filename="calibration.yaml")  # TODO callback to reload clever?

        # from os.path import expanduser # TODO on client
        # home = expanduser("~") -> "~catkin_ws/src/clever/clever/camera_info/"

    @pyqtSlot()
    def send_aruco(self):
        def callback(copter):
            copter.client.send_message("service_restart", {"name": "clever"})

        self.send_files("Select aruco map configuration file", "Aruco map files (*.txt)", onefile=True,
                        client_path="/home/pi/catkin_ws/src/clever/aruco_pose/map/",
                        client_filename="animation_map.txt", callback=callback)

    @pyqtSlot()
    def send_launch(self):
        self.send_directory_files("Select directory with calibrations", ('.yaml', ), match_id=False,
                                  client_path='"/home/pi/catkin_ws/src/clever/clever/launch/')  # TODO clever restart callback?

    @pyqtSlot()
    def send_fcu_parameters(self):
        def request_callback(copter, value):
            logging.info("Send parameters to {} success: {}".format(copter.client.copter_id, value))

        def callback(copter):
            copter.client.get_response("load_params", request_callback)

        self.send_files("Select px4 param file", "px4 params (*.params)", onefile=True,
                        client_filename="temp.params", callback=callback)

    @pyqtSlot()
    def send_config(self):
        path = QFileDialog.getOpenFileName(self, "Select configuration file", filter="Configs (*.ini *.txt .cfg)")[0]
        if not path:
            return

        config = cfg.ConfigManager()
        config.load_only_config(path)
        data = config.full_dict
        logging.info(f"Loaded config from {path}")

        copters = self.model.user_selected()
        for copter in copters:
            copter.client.send_message("config", {"config": data, })

        # if path:
        #     print("Selected file:", path)
        #     sendable_config = configparser.ConfigParser()  # TODO
        #     sendable_config.read(path)
        #     options = []
        #     for section in sendable_config.sections():
        #         for option in dict(sendable_config.items(section)):
        #             value = sendable_config[section][option]
        #             logging.debug("Got item from config: {} {} {}".format(section, option, value))
        #             options.append(ConfigOption(section, option, value))
        #
        #     for copter in self.model.user_selected():
        #         copter.client.send_config_options(*options)

    @pyqtSlot()
    def send_any_command(self):
        text, okPressed = QInputDialog.getText(self, "Enter command to send on copter",
                                               "Command:", QLineEdit.Normal, "")
        if okPressed and text:
            self.send_to_selected("execute", {"command": text})

    @pyqtSlot()
    def select_music_file(self):
        path = QFileDialog.getOpenFileName(self, "Select music file", filter="Music files (*.mp3 *.wav)")[0]
        if not path:
            return

        media = QUrl.fromLocalFile(path)
        content = QtMultimedia.QMediaContent(media)
        self.player.setMedia(content)
        self.ui.action_select_music_file.setText(self.ui.action_select_music_file.text() + " (selected)")

    @pyqtSlot()
    def play_music(self):
        if self.player.mediaStatus() == QtMultimedia.QMediaPlayer.InvalidMedia:
            logging.info("Can't play media")
            return
        if self.player.mediaStatus() == QtMultimedia.QMediaPlayer.NoMedia:
            logging.info("No media file")
            return

        if self.player.state() == QtMultimedia.QMediaPlayer.StoppedState or \
                self.player.state() == QtMultimedia.QMediaPlayer.PausedState:
            self.ui.action_play_music.setText("Pause music")
            self.player.play()
        else:
            self.ui.action_play_music.setText("Play music")
            self.player.pause()

    @pyqtSlot()
    def stop_music(self):
        if self.player.mediaStatus() == QtMultimedia.QMediaPlayer.InvalidMedia:
            logging.error("Can't stop media")
            return
        if self.player.mediaStatus() == QtMultimedia.QMediaPlayer.NoMedia:
            logging.error("No media file")
            return
        self.player.stop()

    @asyncio.coroutine
    def play_music_at_time(self, t):
        if self.player.mediaStatus() == QtMultimedia.QMediaPlayer.InvalidMedia:
            logging.error("Can't play media")
            return
        if self.player.mediaStatus() == QtMultimedia.QMediaPlayer.NoMedia:
            logging.error("No media file")
            return
        self.player.stop()
        yield from asyncio.sleep(t - time.time())
        logging.info("Playing music")
        self.player.play()

    @pyqtSlot()
    def emergency(self):  # TODO refactor for the sake of god
        client_row_min = 0
        client_row_max = self.model.rowCount() - 1
        result = -1
        while (result != 0) and (result != 3) and (result != 4):
            # light_green_red(min, max)
            client_row_mid = int(math.ceil((client_row_max + client_row_min) / 2.0))
            print(client_row_min, client_row_mid, client_row_max)
            for row_num in range(client_row_min, client_row_mid):
                self.model.data_contents[row_num].client \
                    .send_message("led_fill", {"green": 255})
            for row_num in range(client_row_mid, client_row_max + 1):
                self.model.data_contents[row_num].client \
                    .send_message("led_fill", {"red": 255})

            Dialog = QtWidgets.QDialog()
            ui = Ui_Dialog()
            ui.setupUi(Dialog)
            Dialog.show()
            result = Dialog.exec()
            print("Dialog result: {}".format(result))

            if client_row_max != client_row_min:
                if result == 1:
                    for row_num in range(client_row_mid, client_row_max + 1):
                        self.model.data_contents[row_num].client \
                            .send_message("led_fill")
                    client_row_max = client_row_mid - 1

                elif result == 2:
                    for row_num in range(client_row_min, client_row_mid):
                        self.model.data_contents[row_num].client \
                            .send_message("led_fill")
                    client_row_min = client_row_mid

        if result == 0:
            Client.broadcast_message("led_fill")
        elif result == 3:
            for row_num in range(client_row_min, client_row_max + 1):
                self.model.data_contents[row_num].client \
                    .send_message("land")
        elif result == 4:
            for row_num in range(client_row_min, client_row_max + 1):
                self.model.data_contents[row_num].client \
                    .send_message("disarm")


@messaging.message_callback("telemetry")
def get_telem_data(self, **kwargs):
    message = kwargs.get("value")
    window.update_table_data(self, message)


def except_hook(cls, exception, traceback):
    sys.__excepthook__(cls, exception, traceback)


def set_taskbar_icon():
    import ctypes

    myappid = 'COEX.droneshow.droneserver'
    ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)


if __name__ == "__main__":
    msgbox_handler = ExitMsgbox()
    msgbox_handler.setLevel(logging.CRITICAL)

    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(name)-7.7s] [%(threadName)-19.19s] [%(levelname)-7.7s]  %(message)s",
        handlers=[
            logging.FileHandler("server_logs/{}.log".format(now)),
            logging.StreamHandler(),
            msgbox_handler
        ])
    
    sys.excepthook = except_hook  # for debugging (exceptions traceback)

    app = QApplication(sys.argv)
    splash_pix = QPixmap('icons/coex_splash.jpg')

    splash = QSplashScreen(splash_pix)
    splash.setEnabled(False)

    splash.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
    progressBar = QProgressBar(splash)
    progressBar.setGeometry(25, splash_pix.height() - 80, splash_pix.width(), 35)
    splash.showMessage("Loading clever-show server"+"\n\n\n\n\n", int(Qt.AlignBottom | Qt.AlignCenter), Qt.white)
    app.processEvents()
    splash.show()
    # time.sleep(3)

    app_icon = QIcon()
    app_icon.addFile('icons/image.ico', QtCore.QSize(256, 256))
    app.setWindowIcon(app_icon)

    if sys.platform == 'win32':
        set_taskbar_icon()

    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)

    # app.exec_()
    with loop:
        server = ServerQt()
        window = MainWindow(server)

        Client.on_first_connect = window.new_client_connected
        Client.on_connect = window.client_connection_changed
        Client.on_disconnect = window.client_connection_changed

        server.start()

        window.show()
       # window.send_directory_files("lol")
        splash.close()

        loop.run_forever()

    server.stop()
    sys.exit()
