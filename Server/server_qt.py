import os
import glob

from PyQt5 import QtWidgets
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from PyQt5.QtCore import Qt, pyqtSlot, QAbstractTableModel

from PyQt5.QtWidgets import QFileDialog, QMessageBox

# Importing gui form
from server_gui import Ui_MainWindow

from server import *
from copter_table_models import *
from emergency import *


class MyTableModel(QAbstractTableModel):
    def __init__(self, parent, headers, *args):
        QAbstractTableModel.__init__(self, parent, *args)


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

        self.init_model()
        
        self.show()
        
    def init_model(self):
        self.proxy_model.setDynamicSortFilter(True)
        self.proxy_model.setSourceModel(self.model)

        # Initing table and table self.model
        self.ui.tableView.setModel(self.proxy_model)
        self.ui.tableView.horizontalHeader().setStretchLastSection(True)
        self.ui.tableView.setSortingEnabled(True)

        self.signals.update_data_signal.connect(self.model.update_item)
        self.signals.add_client_signal.connect(self.model.add_client)

    def client_connected(self, client: Client):
        self.signals.add_client_signal.emit(CopterData(copter_id=client.copter_id, client=client))

    def init_ui(self):
        # Connecting
        self.ui.check_button.clicked.connect(self.selfcheck_selected)
        self.ui.start_button.clicked.connect(self.send_starttime)
        self.ui.pause_button.clicked.connect(self.pause_all)
        self.ui.stop_button.clicked.connect(self.stop_all)
        self.ui.emergency_button.clicked.connect(self.emergency)

        self.ui.leds_button.clicked.connect(self.test_leds)
        self.ui.takeoff_button.clicked.connect(self.takeoff_selected)
        self.ui.land_button.clicked.connect(self.land_all)
        self.ui.disarm_button.clicked.connect(self.disarm_all)
        self.ui.flip_button.clicked.connect(self.flip)
        self.ui.action_send_animations.triggered.connect(self.send_animations)
        self.ui.action_send_configurations.triggered.connect(self.send_configurations)
        self.ui.action_send_Aruco_map.triggered.connect(self.send_aruco)

    @pyqtSlot()
    def selfcheck_selected(self):
        for copter_data in self.model.user_selected():
            copter = copter_data.client

            copter.get_response("anim_id", self._set_copter_data, callback_args=(1, copter_data.copter_id))
            copter.get_response("batt_voltage", self._set_copter_data, callback_args=(2, copter_data.copter_id))
            copter.get_response("cell_voltage", self._set_copter_data, callback_args=(3, copter_data.copter_id))
            copter.get_response("selfcheck", self._set_copter_data, callback_args=(4, copter_data.copter_id))
            copter.get_response("time", self._set_copter_data, callback_args=(5, copter_data.copter_id))

        #self.ui.start_button.setEnabled(True)
        #self.ui.takeoff_button.setEnabled(True)

    def _set_copter_data(self, value, col, copter_id):
        row = self.model.data_contents.index(next(
            filter(lambda x: x.copter_id == copter_id, self.model.data_contents)))

        if col == 1:
            data = value
        elif col == 2:
            data = "{} V.".format(round(float(value), 3))
        elif col == 3:
            batt_percent = ((float(value) - 3.2) / (4.2 - 3.2)) * 100  # TODO config
            data = "{} %".format(round(batt_percent, 3))
        elif col == 4:
            data = str(value)
        elif col == 5:
            data = time.ctime(int(value))
            data2 = "{} sec.".format(round(int(value) - time.time(), 3))
            self.signals.update_data_signal.emit(row, col + 1, data2)
        else:
            print("No column matched for response")
            return

        self.signals.update_data_signal.emit(row, col, data)

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
            for row_num in range(self.model.rowCount()):
                item = self.model.item(row_num, 0)
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
        for row_num in range(self.model.rowCount()):
            item = self.model.item(row_num, 0)
            if item.isCheckable() and item.checkState() == Qt.Checked:
                if True:
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
            for row_num in range(self.model.rowCount()):
                item = self.model.item(row_num, 0)
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
                for row_num in range(self.model.rowCount()):
                    item = self.model.item(row_num, 0)
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
                    
            for row_num in range(self.model.rowCount()):
                item = self.model.item(row_num, 0)
                if item.isCheckable() and item.checkState() == Qt.Checked:
                    copter = Client.get_by_id(item.text())
                    copter.send_config_options(*options)

    @pyqtSlot()
    def send_aruco(self):
        path = QFileDialog.getOpenFileName(self, "Select aruco map configuration file", filter="Aruco map files (*.txt)")[0]
        if path:
            filename = os.path.basename(path)
            print("Selected file:", path, filename)
            for row_num in range(self.model.rowCount()):
                item = self.model.item(row_num, 0)
                if item.isCheckable() and item.checkState() == Qt.Checked:
                    copter = Client.get_by_id(item.text())
                    copter.send_file(path, "/home/pi/catkin_ws/src/clever/aruco_pose/map/animation_map.txt")
                    copter.send_message("service_restart", {"name": "clever"})
    @pyqtSlot()
    def emergency(self):
        for row_num in range(self.model.rowCount()):
                item = self.model.item(row_num, 0)
                if item.isCheckable() and item.checkState() == Qt.Checked:
                    copter = Client.get_by_id(item.text())
                    copter.send_message("emergency")
        Dialog = QtWidgets.QDialog()
        ui = Ui_Dialog()
        ui.setupUi(Dialog)
        Dialog.show()
        Dialog.exec_()

    @pyqtSlot()    
    def flip(self):
        reply = QMessageBox.question(
            self, "Confirm operation",
            "You are ready to turn the copter?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            print("Accepted")
            for row_num in range(self.model.rowCount()):
                item = self.model.item(row_num, 0)
                if item.isCheckable() and item.checkState() == Qt.Checked:
                    if True:  # TODO checks for batt/selfckeck here
                        copter = Client.get_by_id(item.text())
                        copter.send_message("flip")
        else:
            print("Cancelled")
            pass


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()

    Client.on_first_connect = window.client_connected

    server = Server(on_stop=app.quit)
    server.start()

    app.exec_()
    server.stop()
    sys.exit()
