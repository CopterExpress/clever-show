import os
import glob
import math
import time
import asyncio
import functools

from PyQt5 import QtWidgets, QtMultimedia
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from PyQt5.QtCore import Qt, pyqtSlot, pyqtSignal, QObject, QUrl

from PyQt5.QtWidgets import QFileDialog, QMessageBox
from quamash import QEventLoop, QThreadExecutor

# Importing gui form
from server_gui import Ui_MainWindow

from server import *
import messaging_lib as messaging
from copter_table_models import *
from emergency import *

import threading


def wait(end, interrupter=threading.Event(), maxsleep=0.1):
    # Added features to interrupter sleep and set max sleeping interval

    while not interrupter.is_set():  # Basic implementation of pause module until()
        now = time.time()
        diff = min(end - now, maxsleep)
        if diff <= 0:
            break
        else:
            time.sleep(diff / 2)


def confirmation_required(text="Are you sure?", label="Confirm operation?"):
    def inner(f):

        @functools.wraps(f)
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

# noinspection PyArgumentList,PyCallByClass
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.init_ui()

        self.model = CopterDataModel()
        self.proxy_model = CopterProxyModel()
        self.signals = SignalManager()
        self.player = QtMultimedia.QMediaPlayer()

        self.init_model()
        
        self.show()
        
    def init_model(self):
        # self.model.on_id_changed = self.set_copter_id

        self.proxy_model.setDynamicSortFilter(True)
        self.proxy_model.setSourceModel(self.model)

        # Initiate table and table self.model
        self.ui.tableView.setModel(self.proxy_model)
        self.ui.tableView.resizeColumnsToContents()

        self.ui.tableView.doubleClicked.connect(self.selfcheck_info_dialog)

        # Connect signals to manipulate model from threads
        self.signals.update_data_signal.connect(self.model.update_item)
        self.signals.add_client_signal.connect(self.model.add_client)
        self.signals.remove_client_signal.connect(self.model.remove_client)

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

    def new_client_connected(self, client: Client):
        self.signals.add_client_signal.emit(StatedCopterData(copter_id=client.copter_id, client=client))

    def client_connection_changed(self, client: Client):
        logging.debug("Start remove {}".format(client.copter_id))
        row_data = self.model.get_row_by_attr("client", client)
        row_num = self.model.get_row_index(row_data)
        logging.debug("Removing {}".format(client.copter_id))
        if row_num is not None:
            if Server().remove_disconnected and (not client.connected):
                client.remove()
                self.signals.remove_client_signal.emit(row_num)
            else:
                self.signals.update_data_signal.emit(row_num, 0, client.connected, ModelStateRole)
        logging.debug("{} removed".format(client.copter_id))

    def init_ui(self):
        # Connecting
        self.ui.check_button.clicked.connect(self.selfcheck_selected)
        self.ui.start_button.clicked.connect(self.send_starttime_selected)
        self.ui.pause_button.clicked.connect(self.pause_resume_selected)
        self.ui.stop_button.clicked.connect(self.land_all)

        self.ui.emergency_button.clicked.connect(self.emergency)
        self.ui.disarm_button.clicked.connect(self.disarm_selected)
        self.ui.disarm_all_button.clicked.connect(self.disarm_all)

        self.ui.leds_button.clicked.connect(self.test_leds_selected)
        self.ui.takeoff_button.clicked.connect(self.takeoff_selected)
        self.ui.flip_button.clicked.connect(self.flip_selected)
        self.ui.land_button.clicked.connect(self.land_selected)        

        self.ui.reboot_fcu.clicked.connect(self.reboot_selected)
        self.ui.calibrate_gyro.clicked.connect(self.calibrate_gyro_selected)
        self.ui.calibrate_level.clicked.connect(self.calibrate_level_selected)

        self.ui.action_remove_row.triggered.connect(self.remove_selected)

        self.ui.action_send_animations.triggered.connect(self.send_animations)
        self.ui.action_send_calibrations.triggered.connect(self.send_calibrations)
        self.ui.action_send_configurations.triggered.connect(self.send_configurations)
        self.ui.action_send_Aruco_map.triggered.connect(self.send_aruco)
        self.ui.action_send_launch_file.triggered.connect(self.send_launch)
        self.ui.action_send_fcu_parameters.triggered.connect(self.send_fcu_parameters)
        self.ui.action_restart_clever.triggered.connect(self.restart_clever)
        self.ui.action_restart_clever_show.triggered.connect(self.restart_clever_show)
        self.ui.action_update_client_repo.triggered.connect(self.update_client_repo)
        self.ui.action_reboot_all.triggered.connect(self.reboot_all_on_selected)
        self.ui.action_set_start_to_current_position.triggered.connect(self.update_start_to_current_position)
        self.ui.action_reset_start.triggered.connect(self.reset_start)
        self.ui.action_set_z_offset_to_ground.triggered.connect(self.set_z_offset_to_ground)
        self.ui.action_reset_z_offset.triggered.connect(self.reset_z_offset)
        self.ui.action_restart_chrony.triggered.connect(self.restart_chrony)
        self.ui.action_select_music_file.triggered.connect(self.select_music_file)
        self.ui.action_play_music.triggered.connect(self.play_music)
        self.ui.action_stop_music.triggered.connect(self.stop_music)

        # Set most safety-important buttons disabled
        self.ui.start_button.setEnabled(False)
        self.ui.takeoff_button.setEnabled(False)
        self.ui.flip_button.setEnabled(False)

    @pyqtSlot()
    def selfcheck_selected(self):
        for copter_data_row in self.model.user_selected():
            client = copter_data_row.client
            client.get_response("telemetry", self.update_table_data)

    @pyqtSlot(str)
    def update_table_data(self, message):
        fields = message.split('`')
        # copter_id git_version animation_id battery_v battery_p system_status calibration_status mode selfcheck current_position start_position copter_time
        copter_id = fields[0]
        git_version = fields[1]
        animation_id = fields[2]
        battery_v = fields[3]
        battery_p = fields[4]
        if battery_v == 'nan' or battery_p == 'nan':
            battery_info = "NO_INFO"
        else:
            battery_info = "{}V {}%".format(battery_v, battery_p)
        sys_status = fields[5]
        cal_status = fields[6]
        mode = fields[7]
        selfcheck = fields[8]
        current_pos = fields[9]
        start_pos = fields[10]
        copter_time = fields[11]
        time_delta = "{}".format(round(float(copter_time) - time.time(), 3))
        row = self.model.get_row_index(self.model.get_row_by_attr('copter_id', copter_id))
        self.signals.update_data_signal.emit(row, 1, git_version, ModelDataRole)
        self.signals.update_data_signal.emit(row, 2, animation_id, ModelDataRole)
        self.signals.update_data_signal.emit(row, 3, battery_info, ModelDataRole)
        self.signals.update_data_signal.emit(row, 4, sys_status, ModelDataRole)
        self.signals.update_data_signal.emit(row, 5, cal_status, ModelDataRole)
        self.signals.update_data_signal.emit(row, 6, mode, ModelDataRole)
        self.signals.update_data_signal.emit(row, 7, selfcheck, ModelDataRole)
        self.signals.update_data_signal.emit(row, 8, current_pos, ModelDataRole)
        self.signals.update_data_signal.emit(row, 9, start_pos, ModelDataRole)
        self.signals.update_data_signal.emit(row, 10, time_delta, ModelDataRole)

    @pyqtSlot(QtCore.QModelIndex)
    def selfcheck_info_dialog(self, index):
        col = index.column()
        if col == 6:
            data = self.proxy_model.data(index, role=ModelDataRole)
            if data and data != "OK":
                dialog = QMessageBox()
                dialog.setIcon(QMessageBox.NoIcon)
                dialog.setStandardButtons(QMessageBox.Ok)
                dialog.setWindowTitle("Selfcheck info")
                dialog.setText("\n".join(data[:10]))
                dialog.setDetailedText("\n".join(data))
                dialog.exec()

    def _selfcheck_shortener(self, data):
        shortened = []
        for line in data:
            if len(line) > 89:
                pass
        return shortened

    @pyqtSlot()
    def remove_selected(self):
        for copter in self.model.user_selected():
            row_num = self.model.get_row_index(copter)
            if row_num is not None:
                copter.client.remove()

                if not Server().remove_disconnected:
                    self.signals.remove_client_signal.emit(row_num)
                
                logging.info("Client removed from table!")
            else:
                logging.error("Client is not in table!")

    @pyqtSlot()
    @confirmation_required("This operation will takeoff selected copters with delay and start animation. Proceed?")
    def send_starttime_selected(self, **kwargs):
        time_now = server.time_now()
        dt = self.ui.start_delay_spin.value()
        logging.info('Wait {} seconds to start animation'.format(dt))
        if self.ui.music_checkbox.isChecked():
            music_dt = self.ui.music_delay_spin.value()
            asyncio.ensure_future(self.play_music_at_time(music_dt+time_now), loop=loop)
            logging.info('Wait {} seconds to play music'.format(music_dt))
        # self.selfcheck_selected()
        for copter in self.model.user_selected():
            if all_checks(copter):
                server.send_starttime(copter.client, dt+time_now)

    @pyqtSlot()
    def pause_resume_selected(self):
        if self.ui.pause_button.text() == 'Pause':
            for copter in self.model.user_selected():
                copter.client.send_message("pause")
            self.ui.pause_button.setText('Resume')
        else:
            self._resume_selected()

    def _resume_selected(self, **kwargs):
        time_gap = 0.1
        for copter in self.model.user_selected():
            copter.client.send_message('resume', {"time": server.time_now() + time_gap})
        self.ui.pause_button.setText('Pause')

    @pyqtSlot()
    def land_all(self):
        Client.broadcast_message("land")

    @pyqtSlot()
    def disarm_selected(self):
        for copter in self.model.user_selected():
            copter.client.send_message("disarm")

    @pyqtSlot()
    def test_leds_selected(self):
        for copter in self.model.user_selected():
            copter.client.send_message("led_test")
    
    @pyqtSlot()
    def disarm_all(self):
        Client.broadcast_message("disarm")

    @pyqtSlot()
    @confirmation_required("This operation will takeoff copters immediately. Proceed?")
    def takeoff_selected(self, **kwargs):
        for copter in self.model.user_selected():
            if takeoff_checks(copter):
                if self.ui.z_checkbox.isChecked():
                    copter.client.send_message("takeoff_z", {"z":str(self.ui.z_spin.value())})
                else:
                    copter.client.send_message("takeoff")

    @pyqtSlot()
    @confirmation_required("This operation will flip(!!!) copters immediately. Proceed?")
    def flip_selected(self, **kwargs):
        for copter in self.model.user_selected():
            if flip_checks(copter):
                copter.client.send_message("flip")

    @pyqtSlot()
    def land_selected(self):
        for copter in self.model.user_selected():
            copter.client.send_message("land")

    @pyqtSlot()
    def reboot_selected(self):
        for copter in self.model.user_selected():
            copter.client.send_message("reboot_fcu")   

    @pyqtSlot()
    def calibrate_gyro_selected(self):
        for copter_data_row in self.model.user_selected():
            client = copter_data_row.client
            # Update calibration status
            row = self.model.get_row_index(copter_data_row)
            col = 5
            data = 'CALIBRATING'
            self.signals.update_data_signal.emit(row, col, data, ModelDataRole)
            # Send request
            client.get_response("calibrate_gyro", self._get_calibration_info, callback_args=(copter_data_row, ))

    @pyqtSlot()
    def calibrate_level_selected(self):
        for copter_data_row in self.model.user_selected():
            client = copter_data_row.client
            # Update calibration status
            row = self.model.get_row_index(copter_data_row)
            col = 5
            data = 'CALIBRATING'
            self.signals.update_data_signal.emit(row, col, data, ModelDataRole)
            # Send request
            client.get_response("calibrate_level", self._get_calibration_info, callback_args=(copter_data_row, ))

    def _get_calibration_info(self, value, copter_data_row):
        col = 5
        row = self.model.get_row_index(copter_data_row)
        data = str(value)
        self.signals.update_data_signal.emit(row, col, data, ModelDataRole)

    @pyqtSlot()
    def send_animations(self):
        path = str(QFileDialog.getExistingDirectory(self, "Select Animation Directory"))

        if path:
            print("Selected directory:", path)
            files = [file for file in glob.glob(path + '/*.csv')]
            names = [os.path.basename(file).split(".")[0] for file in files]
            for file, name in zip(files, names):
                for copter in self.model.user_selected():
                    if name == copter.copter_id:
                        copter.client.send_file(file, "animation.csv")  # TODO config
                else:
                    logging.info("Filename has no matches with any drone selected")

    @pyqtSlot()
    def send_calibrations(self):
        path = str(QFileDialog.getExistingDirectory(self, "Select directory with calibration files"))

        if path:
            print("Selected directory:", path)
            files = [file for file in glob.glob(path + '/*.yaml')]
            names = [os.path.basename(file).split(".")[0] for file in files]
            # print(files)
            for file, name in zip(files, names):
                for copter in self.model.user_selected():
                    if name == copter.copter_id:
                        copter.client.send_file(file, "/home/pi/catkin_ws/src/clever/clever/camera_info/calibration.yaml")
                else:
                    logging.info("Filename has no matches with any drone selected")

    @pyqtSlot()
    def send_configurations(self):
        path = QFileDialog.getOpenFileName(self, "Select configuration file", filter="Configs (*.ini *.txt .cfg)")[0]
        if path:
            print("Selected file:", path)
            sendable_config = configparser.ConfigParser()
            sendable_config.read(path)
            options = []
            for section in sendable_config.sections():
                for option in dict(sendable_config.items(section)):
                    value = sendable_config[section][option]
                    logging.debug("Got item from config: {} {} {}".format(section, option, value))
                    options.append(ConfigOption(section, option, value))

            for copter in self.model.user_selected():
                copter.client.send_config_options(*options)

    @pyqtSlot()
    def send_aruco(self):
        path = QFileDialog.getOpenFileName(self, "Select aruco map configuration file", filter="Aruco map files (*.txt)")[0]
        if path:
            filename = os.path.basename(path)
            print("Selected file:", path, filename)
            for copter in self.model.user_selected():
                copter.client.send_file(path, "/home/pi/catkin_ws/src/clever/aruco_pose/map/animation_map.txt")
                copter.client.send_message("service_restart", {"name": "clever"})

    @pyqtSlot()
    def send_launch(self):
        path = str(QFileDialog.getExistingDirectory(self, "Select directory with launch files"))
        if path:
            print("Selected directory:", path)
            files = [file for file in glob.glob(path + '/*.launch')]
            for copter in self.model.user_selected():
                for file in files:
                    filename = os.path.basename(file)
                    copter.client.send_file(file, "/home/pi/catkin_ws/src/clever/clever/launch/{}".format(filename))
                # copter.client.send_message("service_restart", {"name": "clever"})
    
    @pyqtSlot()
    def send_fcu_parameters(self):
        path = QFileDialog.getOpenFileName(self, "Select px4 param file", filter="px4 params (*.params)")[0]
        if path:
            filename = os.path.basename(path)
            print("Selected file:", path, filename)
            for copter in self.model.user_selected():
                copter.client.send_file(path, "temp.params")
                copter.client.get_response("load_params", self._print_send_fcu_params_result, callback_args=(copter, ))

    def _print_send_fcu_params_result(self, value, copter):
        logging.info("Send parameters to {} success: {}".format(copter.client.copter_id, value))
                # copter.client.send_message("service_restart", {"name": "clever"})
    
    @pyqtSlot()
    def restart_clever(self):
        for copter in self.model.user_selected():
            copter.client.send_message("service_restart", {"name": "clever"})
    
    @pyqtSlot()
    def restart_clever_show(self):
        for copter in self.model.user_selected():
            copter.client.send_message("service_restart", {"name": "clever-show"})

    @pyqtSlot()
    def update_client_repo(self):
        for copter in self.model.user_selected():
            copter.client.send_message("update_repo") 

    @pyqtSlot()
    def reboot_all_on_selected(self):
        for copter in self.model.user_selected():
            copter.client.send_message("reboot_all") 

    @pyqtSlot()
    def update_start_to_current_position(self):
        for copter in self.model.user_selected():
            copter.client.send_message("move_start")

    @pyqtSlot()
    def reset_start(self):
        for copter in self.model.user_selected():
            copter.client.send_message("reset_start")

    @pyqtSlot()
    def set_z_offset_to_ground(self):
        for copter in self.model.user_selected():
            copter.client.send_message("set_z_to_ground")

    @pyqtSlot()
    def reset_z_offset(self):
        for copter in self.model.user_selected():
            copter.client.send_message("reset_z_offset")

    @pyqtSlot()
    def restart_chrony(self):
        for copter in self.model.user_selected():
            copter.client.send_message("repair_chrony")

    @pyqtSlot()
    def select_music_file(self):
        path = QFileDialog.getOpenFileName(self, "Select music file", filter="Music files (*.mp3 *.wav)")[0]
        if path:
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
    def emergency(self):
        client_row_min = 0
        client_row_max = self.model.rowCount() - 1
        result = -1
        while (result != 0) and (result != 3) and (result != 4):
            # light_green_red(min, max)
            client_row_mid = int(math.ceil((client_row_max+client_row_min) / 2.0))
            print(client_row_min, client_row_mid, client_row_max)
            for row_num in range(client_row_min, client_row_mid):
                self.model.data_contents[row_num].client\
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

@messaging.message_callback("telem")
def get_telem_data(*args, **kwargs):
    message = kwargs.get("message", None)
    window.update_table_data(message)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)

    #app.exec_()
    with loop:
        window = MainWindow()

        Client.on_first_connect = window.new_client_connected
        Client.on_connect = window.client_connection_changed
        Client.on_disconnect = window.client_connection_changed

        server = Server(on_stop=app.quit)
        server.start()
        loop.run_forever()

    server.stop()
    sys.exit()
