import os
import glob

from PyQt5 import QtWidgets
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from PyQt5.QtCore import Qt, pyqtSlot

from PyQt5.QtWidgets import QFileDialog, QMessageBox

# Importing gui form
from server_gui import Ui_MainWindow

from server import *


# noinspection PyArgumentList,PyCallByClass
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.init_ui()
        self.show()

    def init_ui(self):
        # Connecting
        self.ui.check_button.clicked.connect(self.check_selected)
        self.ui.start_button.clicked.connect(self.send_starttime)
        self.ui.pause_button.clicked.connect(self.pause_all)
        self.ui.stop_button.clicked.connect(self.stop_all)
        self.ui.test_Button.clicked.connect(self.test)

        self.ui.leds_button.clicked.connect(self.test_leds)
        self.ui.takeoff_button.clicked.connect(self.takeoff_selected)
        self.ui.land_button.clicked.connect(self.land_all)
        self.ui.disarm_button.clicked.connect(self.disarm_all)

        self.ui.action_send_animations.triggered.connect(self.send_animations)
        self.ui.action_send_configurations.triggered.connect(self.send_configurations)
        self.ui.action_send_Aruco_map.triggered.connect(self.send_aruco)

        # Initing table and table model
        self.ui.tableView.setModel(model)
        self.ui.tableView.horizontalHeader().setStretchLastSection(True)

    @pyqtSlot()
    def check_selected(self):
        for row_num in range(model.rowCount()):
            item = model.item(row_num, 0)
            if item.isCheckable() and item.checkState() == Qt.Checked:
                print("Copter {} checked".format(model.item(row_num, 0).text()))
                copter = Client.get_by_id(item.text())
                copter.get_response("anim_id", self._set_copter_data, callback_args=(row_num, 1))
                copter.get_response("batt_voltage", self._set_copter_data, callback_args=(row_num, 2))
                copter.get_response("cell_voltage", self._set_copter_data, callback_args=(row_num, 3))
                copter.get_response("selfcheck", self._set_copter_data, callback_args=(row_num, 4))
                copter.get_response("time", self._set_copter_data, callback_args=(row_num, 5))

        self.ui.start_button.setEnabled(True)
        self.ui.takeoff_button.setEnabled(True)

    def _set_copter_data(self, value, row, col):
        if col == 1:
            model.setData(model.index(row, col), value)
        elif col == 2:
            model.setData(model.index(row, col), "{} V.".format(round(float(value), 3)))
        elif col == 3:
            batt_percent = ((float(value) - 3.2) / (4.2 - 3.2)) * 100
            model.setData(model.index(row, col), "{} %".format(round(batt_percent, 3)))
        elif col == 4:
            if value != "OK":
                model.setData(model.index(row, col), str(value))  # TODO different handling
            else:
                model.setData(model.index(row, col), str(value))
        elif col == 5:
            model.setData(model.index(row, col), time.ctime(int(value)))

    @pyqtSlot()
    def send_starttime(self):
        dt = self.ui.start_delay_spin.value()
        reply = QMessageBox.question(
            self, "Confirm operation",
            "This operation will takeoff selected copters and start animation after {} seconds. Proceed?".format(dt),
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            print("Accepted")
            for row_num in range(model.rowCount()):
                item = model.item(row_num, 0)
                if item.isCheckable() and item.checkState() == Qt.Checked:
                    if True:  # TODO checks for batt/selfckeck here
                        copter = Client.get_by_id(item.text())
                        server.send_starttime(copter, dt)
        else:
            print("Cancelled")

    @pyqtSlot()
    def stop_all(self):
        Client.broadcast_message("stop")

    @pyqtSlot()
    def pause_all(self):
        if self.ui.pause_button.text() == 'Pause':
            Client.broadcast_message('pause')
            self.ui.pause_button.setText('Resume')
        else:
            Client.broadcast_message('resume')
        self.ui.pause_button.setText('Pause')

    @pyqtSlot()
    def test_leds(self):
        for row_num in range(model.rowCount()):
            item = model.item(row_num, 0)
            if item.isCheckable() and item.checkState() == Qt.Checked:
                if True:  # TODO checks for batt/selfckeck here
                    copter = Client.get_by_id(item.text())
                    copter.send_message("led_test")

    @pyqtSlot()
    def takeoff_selected(self):
        reply = QMessageBox.question(
            self, "Confirm operation",
            "This operation will takeoff copters immediately. Proceed?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            print("Accepted")
            for row_num in range(model.rowCount()):
                item = model.item(row_num, 0)
                if item.isCheckable() and item.checkState() == Qt.Checked:
                    if True:  # TODO checks for batt/selfckeck here
                        copter = Client.get_by_id(item.text())
                        copter.send_message("takeoff")
        else:
            print("Cancelled")
            pass

    @pyqtSlot()
    def land_all(self):
        Client.broadcast_message("land")

    @pyqtSlot()
    def disarm_all(self):
        Client.broadcast_message("disarm")

    @pyqtSlot()
    def send_animations(self):
        path = str(QFileDialog.getExistingDirectory(self, "Select Animation Directory"))
        if path:
            print("Selected directory:", path)
            files = [file for file in glob.glob(path + '/*.csv')]
            names = [os.path.basename(file).split(".")[0] for file in files]
            print(files)
            for file, name in zip(files, names):
                for row_num in range(model.rowCount()):
                    item = model.item(row_num, 0)
                    if item.isCheckable() and item.checkState() == Qt.Checked:
                        copter = Client.get_by_id(item.text())
                        if name == copter.copter_id:
                            copter.send_file(file, "animation.csv")  # TODO config
                        else:
                            print("Filename not matches with any drone connected")

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
                    logging.debug("Got item from config:".format(section, option, value))
                    options.append(ConfigOption(section, option, value))
            for row_num in range(model.rowCount()):
                item = model.item(row_num, 0)
                if item.isCheckable() and item.checkState() == Qt.Checked:
                    copter = Client.get_by_id(item.text())
                    copter.send_config_options(*options)

    @pyqtSlot()
    def send_aruco(self):
        path = QFileDialog.getOpenFileName(self, "Select aruco map configuration file", filter="Aruco map files (*.txt)")[0]
        if path:
            filename = os.path.basename(path)
            print("Selected file:", path, filename)
            for row_num in range(model.rowCount()):
                item = model.item(row_num, 0)
                if item.isCheckable() and item.checkState() == Qt.Checked:
                    copter = Client.get_by_id(item.text())
                    copter.send_file(path, "/home/pi/catkin_ws/src/clever/aruco_pose/map/animation_map.txt")
                    copter.send_message("service_restart", {"name": "clever"})
    @pyqtSlot()
    def test(self):
        for row_num in range(model.rowCount()):
                item = model.item(row_num, 0)
                if item.isCheckable() and item.checkState() == Qt.Checked:
                    copter = Client.get_by_id(item.text())
                    copter.send_message("test")


model = QStandardItemModel()
model.setHorizontalHeaderLabels(
    ('copter ID', 'animation ID', 'battery V', 'battery %', 'selfcheck', 'time UTC')
)
model.setColumnCount(6)
model.setRowCount(0)


def client_connected(self: Client):
    copter_id_item = QStandardItem(self.copter_id)
    copter_id_item.setCheckable(True)
    model.appendRow((copter_id_item, ))


Client.on_first_connect = client_connected


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()

    server = Server(on_stop=app.quit)
    server.start()

    app.exec_()
    server.stop()
    sys.exit()
