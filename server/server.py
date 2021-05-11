import os
import re
import sys
import glob
import time
import random
import logging
import asyncio
import platform
import itertools
import subprocess

from functools import partial, wraps
from quamash import QEventLoop

# Import server routines
from modules.server_core import Server, Client, now

# Import modules from lib, that was added to PATH on the previous step
import messaging
import config as cfg
from lib import b_partial

# Import PyQt5 related functions
from PyQt5 import QtWidgets, QtMultimedia, QtCore
from PyQt5.QtGui import QPixmap, QIcon
from PyQt5.QtCore import Qt, pyqtSlot, QUrl
from PyQt5.QtWidgets import QFileDialog, QMessageBox, QApplication, QInputDialog, QLineEdit, QStatusBar, \
    QSplashScreen, QProgressBar

# Import gui form
from modules.ui.server_gui import Ui_MainWindow

# Import gui logic
import modules.copter_table_models as table
from modules.copter_table import CopterTableWidget, HeaderEditDialog
from modules.visual_land_dialog import VisualLandDialog
from modules.config_editor_models import ConfigDialog

startup_cwd = os.getcwd()
random.seed()

def multi_glob(*patterns):
    return itertools.chain.from_iterable(glob.iglob(pattern) for pattern in patterns)

def restart():  # move to core
    window.server.stop()
    window.on_quit()
    QApplication.quit()

    args = sys.argv[:]
    logging.info('Restarting {}'.format(args))
    args.insert(0, sys.executable)
    if sys.platform == 'win32':
        args = ['"%s"' % arg for arg in args]
    os.chdir(startup_cwd)
    os.execv(sys.executable, args)


def update_server():
    subprocess.call("git fetch && git pull --rebase", shell=True)
    restart()


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
        table.ModelChecks.check_git = self.config.checks_check_git_version
        table.ModelChecks.check_current_pos = self.config.checks_check_current_position
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

        self.register_callbacks()
        self.player = QtMultimedia.QMediaPlayer()

    def init_ui(self):
        self.init_table()

        # Connecting
        self.ui.check_button.clicked.connect(self.selfcheck_selected)
        self.ui.start_button.clicked.connect(self.send_start_time_selected)
        self.ui.pause_button.clicked.connect(self.pause_resume_selected)

        self.ui.z_checkbox.clicked.connect(self.ui.z_spin.setEnabled)
        self.ui.z_spin.setEnabled(False)

        self.ui.land_all_button.clicked.connect(b_partial(Client.broadcast_message, "land"))
        self.ui.land_selected_button.clicked.connect(b_partial(self.send_to_selected, "land"))
        self.ui.disarm_all_button.clicked.connect(b_partial(Client.broadcast_message, "disarm"))
        self.ui.disarm_selected_button.clicked.connect(b_partial(self.send_to_selected, "disarm"))
        self.ui.visual_land_button.clicked.connect(self.visual_land)
        self.ui.emergency_land_button.clicked.connect(b_partial(self.send_to_selected, "emergency_land"))
        self.ui.leds_button.clicked.connect(b_partial(self.send_to_selected, "led_test"))
        self.ui.takeoff_button.clicked.connect(self.takeoff_selected)
        self.ui.flip_button.clicked.connect(self.flip_selected)
        self.ui.reboot_fcu.clicked.connect(b_partial(self.send_to_selected, "reboot_fcu"))
        self.ui.calibrate_gyro.clicked.connect(self.calibrate_gyro_selected)
        self.ui.calibrate_level.clicked.connect(self.calibrate_level_selected)

        self.ui.action_select_music_file.triggered.connect(self.select_music_file)
        self.ui.action_play_music.triggered.connect(self.play_music)
        self.ui.action_stop_music.triggered.connect(self.stop_music)

        self.ui.action_edit_any_config.triggered.connect(ConfigDialog.call_standalone_dialog)
        self.ui.action_edit_server_config.triggered.connect(self.edit_server_config)

        self.ui.action_restart_server.triggered.connect(restart)
        self.ui.action_update_server_git.triggered.connect(update_server)

        self.ui.action_select_all.triggered.connect(partial(self.ui.copter_table.select_all, Qt.Checked))
        self.ui.action_deselect_all.triggered.connect(partial(self.ui.copter_table.select_all, Qt.Unchecked))
        self.ui.action_toggle_select.triggered.connect(self.ui.copter_table.toggle_select)
        self.ui.action_remove_row.triggered.connect(self.remove_selected)
        self.ui.action_configure_columns.triggered.connect(self.configure_columns)

        self.ui.action_send_animations.triggered.connect(self.send_animations)
        self.ui.action_send_calibrations.triggered.connect(self.send_calibrations)
        self.ui.action_send_animation.triggered.connect(self.send_animation)
        self.ui.action_send_configurations.triggered.connect(self.send_config)
        self.ui.action_send_aruco_map.triggered.connect(self.send_aruco)
        self.ui.action_send_launch_file.triggered.connect(self.send_launch)
        self.ui.action_send_fcu_parameters.triggered.connect(self.send_fcu_parameters)
        self.ui.action_send_any_file.triggered.connect(self.send_any_file)
        self.ui.action_send_any_command.triggered.connect(self.send_any_command)

        self.ui.action_retrive_any_file.triggered.connect(b_partial(self.request_any_file, client_path=None))

        self.ui.action_restart_clever.triggered.connect(
            b_partial(self.send_to_selected, "service_restart", command_kwargs={"name": "clover"}))
        self.ui.action_restart_clever_show.triggered.connect(self.restart_clever_show)
        self.ui.action_restart_chrony.triggered.connect(self.restart_chrony)
        self.ui.action_reboot_all.triggered.connect(b_partial(self.send_to_selected, "reboot_all"))

        self.ui.action_set_start_to_current_position.triggered.connect(b_partial(self.send_to_selected, "move_start"))
        self.ui.action_reset_start.triggered.connect(b_partial(self.send_to_selected, "reset_start"))
        self.ui.action_set_z_offset_to_ground.triggered.connect(b_partial(self.send_to_selected, "set_z_to_ground"))
        self.ui.action_reset_z_offset.triggered.connect(b_partial(self.send_to_selected, "reset_z_offset"))

        self.ui.action_update_client_repo.triggered.connect(b_partial(self.send_to_selected, "update_repo"))

        menubar = self.menuBar()
        custom_menu = menubar.addMenu('Custom')

        send_dronepoint_command = QtWidgets.QAction('Dronepoint', self)
        send_dronepoint_command.triggered.connect(self.dronepoint_command)
        custom_menu.addAction(send_dronepoint_command)

    def init_table(self):
        # Remove standard table widget
        self.ui.horizontalLayout.removeWidget(self.ui.tableView)
        self.ui.tableView.close()
        # Init our custom widget
        self.ui.copter_table = CopterTableWidget(self.model, self.server.config)
        self.ui.copter_table.setObjectName("copter_table")
        # Insert to layout at right
        self.ui.horizontalLayout.insertWidget(0, self.ui.copter_table, 0)
        self.ui.copter_table.setFocus()

    def init_model(self):
        # Connect model signals to UI
        self.model.selected_ready_signal.connect(self.ui.start_button.setEnabled)
        self.model.selected_takeoff_ready_signal.connect(self.ui.takeoff_button.setEnabled)
        self.model.selected_flip_ready_signal.connect(self.ui.flip_button.setEnabled)
        # Connect calibrating signal (testing)
        self.model.selected_calibrating_signal.connect(self.ui.check_button.setDisabled)
        self.model.selected_calibrating_signal.connect(self.ui.pause_button.setDisabled)
        self.model.selected_calibrating_signal.connect(self.ui.land_all_button.setDisabled)
        self.model.selected_calibrating_signal.connect(self.ui.land_selected_button.setDisabled)
        self.model.selected_calibrating_signal.connect(self.ui.disarm_selected_button.setDisabled)
        self.model.selected_calibrating_signal.connect(self.ui.disarm_all_button.setDisabled)
        self.model.selected_calibrating_signal.connect(self.ui.visual_land_button.setDisabled)
        self.model.selected_calibrating_signal.connect(self.ui.emergency_land_button.setDisabled)
        self.model.selected_calibrating_signal.connect(self.ui.leds_button.setDisabled)
        self.model.selected_calibrating_signal.connect(self.ui.reboot_fcu.setDisabled)

        self.model.selected_calibration_ready_signal.connect(self.ui.calibrate_gyro.setEnabled)
        self.model.selected_calibration_ready_signal.connect(self.ui.calibrate_level.setEnabled)

        # Set most safety-important buttons disabled
        self.model.emit_signals()

    def show(self):
        self.ui.copter_table.load_columns()
        super().show()

    def showMaximized(self):  # TODO move to widget
        self.ui.copter_table.load_columns()
        super().showMaximized()

    def closeEvent(self, event):
        if not any(copter.connected for copter in Client.clients.values()):
            event.accept()
            return

        reply = QMessageBox.question(self, "Confirm exit", "There are copters connected to the server. "
                                     "Are you sure you want to exit?",
                                     QMessageBox.No | QMessageBox.Yes, QMessageBox.No)

        if reply != QMessageBox.Yes:
            event.ignore()
        else:
            event.accept()
            QApplication.quit()

    def on_quit(self):
        self.ui.copter_table.save_columns()
        self.server.config.write()
        logging.info("Exit actions completed: config saved")

    def iterate_selected(self, f, *args, **kwargs):
        for copter in self.model.user_selected():
            yield f(copter, *args, **kwargs)

    @pyqtSlot()
    def send_to_selected(self, command, command_args=(), command_kwargs=None):
        return list(self.iterate_selected(lambda copter: copter.client.send_message(
            command, command_args, command_kwargs)))

    def new_client_connected(self, client: Client):  # TODO merge with client_connection_changed
        if self.model.get_row_by_attr('client', client) is not None:
            logging.warning("Client is already in table! {}".format(client))
            return

        self.model.add_client(copter_id=client.copter_id, client=client)
        logging.debug("Added client {}".format(client))

    def client_connection_changed(self, client: Client):
        logging.debug("Connection {} changed {}".format(client, client.connected))
        row_data = self.model.get_row_by_attr("client", client)

        if row_data is None:
            logging.error("No row for client presented")
            return

        if self.server.config.table_remove_disconnected and (not client.connected):
            client.remove()
            self.model.remove_client_data(row_data)
            logging.debug("Removing from table")
        else:
            row_num = self.model.get_row_index(row_data)
            if row_num is not None:
                self.model.update_data(row_num, 0, client.connected, table.ModelStateRole)
                logging.debug("Client status updated")

    @pyqtSlot()
    def selfcheck_selected(self):
        for copter in self.model.user_selected():
            copter.client.get_response("telemetry", self.update_table_data)

    @pyqtSlot(object, dict)
    def update_table_data(self, client, telems: dict):
        for key, value in telems.items():
            try:
                col = self.model.columns.index(key)
            except ValueError:
                logging.debug(f"No column {key} present!")
            else:
                row_data = self.model.get_row_by_attr("client", client)
                row_num = self.model.get_row_index(row_data)
                if row_num is not None:
                    self.model.update_data(row_num, col, value, Qt.EditRole)

    @pyqtSlot()
    def remove_selected(self):
        for copter in self.model.user_selected():
            copter.client.remove()
            if not self.server.config.table_remove_disconnected:
                self.model.remove_client_data(copter)
            logging.info("Client removed from table!")

    @pyqtSlot()
    @confirmation_required("This operation will takeoff selected copters with delay and start animation. Proceed?")
    def send_start_time_selected(self):
        time_now = server.time_now()
        time_lag = 0.1
        dt = self.ui.start_delay_spin.value()
        logging.info('Wait {} seconds to start animation'.format(dt))
        if self.ui.music_checkbox.isChecked():
            music_dt = self.ui.music_delay_spin.value()
            asyncio.ensure_future(self.play_music_at_time(music_dt + time_now + time_lag), loop=loop)
            logging.info('Wait {} seconds to play music'.format(music_dt))
        # This filter constraints takeoff in real world, when copter state was normal and then some checks were failed for a while
        # for copter in filter(lambda copter: copter.states.all_checks, self.model.user_selected()):
        for copter in self.model.user_selected():
            server.send_starttime(copter.client, dt + time_now + time_lag)

    @pyqtSlot()
    def pause_resume_selected(self):
        if self.ui.pause_button.text() == 'Pause':
            self.send_to_selected("pause")
            self.ui.pause_button.setText('Resume')
        else:
            time_gap = 0.1  # TODO config? automatic delay detection?
            self.send_to_selected("resume", command_kwargs={"time": server.time_now() + time_gap})
            self.ui.pause_button.setText('Pause')

    @pyqtSlot()
    @confirmation_required("This operation will takeoff copters immediately. Proceed?")
    def takeoff_selected(self):
        for copter in self.model.user_selected():
            if table.takeoff_checks(copter):
                if self.ui.z_checkbox.isChecked():
                    copter.client.send_message("takeoff_z", kwargs={"z": str(self.ui.z_spin.value())})  # todo int, merge commands
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
            self.model.update_data(row, col, data, table.ModelDataRole)
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
            self.model.update_data(row, col, data, table.ModelDataRole)
            # Send request
            client.get_response("calibrate_level", self._get_calibration_info)

    def _get_calibration_info(self, client, value):
        col = 5
        row_data = self.model.get_row_by_attr("client", client)
        row = self.model.get_row_index(row_data)
        if row is not None:
            data = str(value)
            self.model.update_data(row, col, data, table.ModelDataRole)

    def _send_files(self, files, copters=None, client_path="", client_filename="", match_id=False, callback=None, clover_dir=False):
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
                logging.error(f"No copters to send file {filename} to")
                continue

            logging.info(f"Sending file {filename} to clients: {to_send}")
            filename = client_filename.format(num, filename) or filename

            for copter in to_send:
                if clover_dir:
                    if copter.client.clover_dir != 'error':
                        path_to_send = os.path.realpath(os.path.join(copter.client.clover_dir, client_path))
                    else:
                        logging.error("Can't send files to clover ROS package on {}".format(copter.copter_id))
                else:
                    path_to_send = client_path
                copter.client.send_file(file, os.path.join(path_to_send, filename))
                if callback is not None:
                    callback(copter)

    def send_files(self, prompt, ext_filter, copters=None, client_path="", client_filename="", match_id=False,
                   onefile=False, callback=None, clover_dir=False):
        if onefile:
            file = QFileDialog.getOpenFileName(self, prompt, filter=ext_filter)[0]
            files = [file] if file else []
        else:
            files = QFileDialog.getOpenFileNames(self, prompt, filter=ext_filter)[0]

        if not files:
            return

        self._send_files(files, copters, client_path, client_filename, match_id, callback, clover_dir)

    def send_directory_files(self, prompt, extensions=(), copters=None, client_path="", client_filename="",
                             match_id=False, callback=None, clover_dir=False):
        path = QFileDialog.getExistingDirectory(self, prompt)

        if not path:
            return

        if extensions:
            patterns = [path + '/*' + ext for ext in extensions]
        else:
            patterns = [path+'/*.*']

        files = multi_glob(*patterns)
        self._send_files(files, copters, client_path, client_filename, match_id, callback, clover_dir)

    def request_any_file(self, client_path=None, copters=None):
        if client_path is None:
            _client_path, ok = QInputDialog.getText(self, "Enter path of file to request from client", "Source:",
                                                    QLineEdit.Normal, "")
            if not ok:
                return
            client_path = _client_path

        save_path = QFileDialog.getSaveFileName(self, "Save file to:", directory=os.path.split(client_path)[1],
                                                filter=f"Current ext(*{os.path.splitext(client_path)[1]});;"
                                                       f"All files(*.*)")[0]
        if not save_path:
            return

        if copters is None:
            copters = self.model.user_selected()
        copters = list(copters)

        logging.info(f'Requesting file {client_path} to  local {save_path} from clients: {copters}')
        for copter in copters:
            if len(copters) > 1:
                save_path = cfg.modify_filename(save_path, f"{{}}_{copter.copter_id}")
            copter.client.get_file(client_path, save_path)
        logging.info('Files requested')

    @pyqtSlot()
    def send_any_file(self):
        file = QFileDialog.getOpenFileName(self, "Select any file")[0]
        if not file:
            return

        c_path, ok = QInputDialog.getText(self, "Enter path (and name) to send on client", "Destination:",
                                          QLineEdit.Normal, "")   # TODO config?
        if not ok:
            return

        c_filepath, c_filename = os.path.split(c_path)  # c stands for client
        files = [file]
        self._send_files(files, client_path=c_filepath, client_filename=c_filename)

    @pyqtSlot()
    def send_animations(self):
        self.send_directory_files("Select directory with animations", ('.csv', '.txt'), match_id=True,
                                  client_path="", client_filename="animation.csv")

    @pyqtSlot()
    def send_animation(self):
        self.send_files("Select animation file", "Animation files (*.csv)", onefile=True, client_filename="animation.csv")

    @pyqtSlot()
    def send_calibrations(self):
        self.send_directory_files("Select directory with calibrations", ('.yaml', ), match_id=True,
                                  client_path="camera_info", client_filename="calibration.yaml", clover_dir=True)

    @pyqtSlot()
    def send_aruco(self):
        def callback(copter):
            copter.client.send_message("service_restart", kwargs={"name": "clover"})

        self.send_files("Select aruco map configuration file", "Aruco map files (*.txt)", onefile=True,
                        client_path="../aruco_pose/map/", client_filename="animation_map.txt",
                        callback=callback, clover_dir=True)

    @pyqtSlot()
    def send_launch(self):
        self.send_directory_files("Select directory with launch files", ('.launch', '.yaml'), match_id=False,
                                  client_path="launch", clover_dir=True)

    @pyqtSlot()
    def send_fcu_parameters(self):
        def request_callback(client, value):
            logging.info("Send parameters to {} success: {}".format(client.copter_id, value))

        def callback(copter):
            copter.client.get_response("load_params", request_callback)

        self.send_files("Select px4 param file", "px4 params (*.params)", onefile=True,
                        client_filename="temp.params", callback=callback)

    @pyqtSlot()
    def send_config(self):
        mode, ok = QInputDialog.getItem(self, "Select config sending mode", "Mode:",
                                        ("Modify", "Rewrite"), 0, False)
        if not ok or not mode:
            return

        path = QFileDialog.getOpenFileName(self, "Select configuration file", filter="Configs (*.ini *.txt *.cfg)")[0]
        if not path:
            return

        config = cfg.ConfigManager()
        config.load_only_config(path)
        data = config.full_dict(include_defaults=False)
        logging.info(f"Loaded config from {path}")

        copters = self.model.user_selected()
        for copter in copters:
            copter.client.send_message("config", kwargs={"config": data, "mode": mode.lower()})

    @pyqtSlot()
    def send_any_command(self):
        text, ok = QInputDialog.getText(self, "Enter command to send on copter",
                                        "Command:", QLineEdit.Normal, "")
        if ok and text:
            self.send_to_selected("execute", command_kwargs={"command": text})

    @pyqtSlot()
    def restart_clever_show(self):
        for copter in self.model.user_selected():
            copter.client.send_message("service_restart", kwargs={"name": "clever-show"})

    @pyqtSlot()
    def restart_chrony(self):
        if platform.system() == 'Linux':
            os.system("pkexec systemctl restart chrony")
        for copter in self.model.user_selected():
            copter.client.send_message("repair_chrony")

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
        self.ui.action_play_music.setText("Play music")

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
    def visual_land(self):
        VisualLandDialog(self.model).start()

    @pyqtSlot()
    def configure_columns(self):
        HeaderEditDialog(self.ui.copter_table, self.server.config).exec()

    @pyqtSlot()
    def edit_server_config(self):
        config = self.server.config

        def save_callback():
            config.write()

        ConfigDialog().call_config_dialog(config, save_callback, restart, name="server config")

    @pyqtSlot()
    def dronepoint_command(self):
        dronepoint_id = random.randint(1, 3)
        container_id = random.randint(1, 3)
        logging.info(f"Dronepoint_id: {dronepoint_id}; Container_id: {container_id}")
        self.send_to_selected("dronepoint", command_kwargs={
            "dronepoint_id": dronepoint_id,
            "container_id": container_id,
        })

    def register_callbacks(self):
        @messaging.message_callback("telemetry")
        def get_telem_data(client, value, **kwargs):
            self.update_table_data(client, value)


def except_hook(cls, exception, traceback):
    sys.__excepthook__(cls, exception, traceback)


def set_taskbar_icon():
    import ctypes

    myappid = 'COEX.droneshow.droneserver'
    ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)


if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.realpath(__file__))
    msgbox_handler = ExitMsgbox()
    msgbox_handler.setLevel(logging.CRITICAL)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)-7.7s] [%(threadName)-19.19s] [%(levelname)-7.7s]  %(message)s",
        handlers=[
            logging.FileHandler(os.path.join(current_dir, "server_logs", "{}.log".format(now))),
            logging.StreamHandler(),
            msgbox_handler
        ])

    sys.excepthook = except_hook  # for debugging (exceptions traceback)

    app = QApplication(sys.argv)
    splash_pix = QPixmap(os.path.join(current_dir, "icons", "coex_splash.jpg"))

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
    app_icon.addFile(os.path.join(current_dir, "icons", "image.ico"), QtCore.QSize(256, 256))
    app.setWindowIcon(app_icon)

    if sys.platform == 'win32':
        set_taskbar_icon()

    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)

    # app.exec_()
    with loop:
        server = ServerQt(config_path=os.path.join(current_dir, "config", "server.ini"))
        window = MainWindow(server)

        Client.on_first_connect = window.new_client_connected
        Client.on_connect = window.client_connection_changed
        Client.on_disconnect = window.client_connection_changed

        app.aboutToQuit.connect(window.on_quit)

        server.start()

        window.showMaximized()
        splash.close()

        loop.run_forever()

    server.stop()
    sys.exit()
