# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'server_gui.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(850, 479)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.check_button = QtWidgets.QPushButton(self.centralwidget)
        self.check_button.setGeometry(QtCore.QRect(680, 20, 150, 40))
        self.check_button.setObjectName("check_button")
        self.start_button = QtWidgets.QPushButton(self.centralwidget)
        self.start_button.setEnabled(False)
        self.start_button.setGeometry(QtCore.QRect(680, 120, 150, 40))
        self.start_button.setFlat(False)
        self.start_button.setObjectName("start_button")
        self.start_delay_spin = QtWidgets.QSpinBox(self.centralwidget)
        self.start_delay_spin.setGeometry(QtCore.QRect(760, 70, 50, 40))
        self.start_delay_spin.setObjectName("start_delay_spin")
        self.pause_button = QtWidgets.QPushButton(self.centralwidget)
        self.pause_button.setGeometry(QtCore.QRect(680, 170, 150, 40))
        self.pause_button.setObjectName("pause_button")
        self.stop_button = QtWidgets.QPushButton(self.centralwidget)
        self.stop_button.setGeometry(QtCore.QRect(680, 220, 150, 40))
        self.stop_button.setObjectName("stop_button")
        self.takeoff_button = QtWidgets.QPushButton(self.centralwidget)
        self.takeoff_button.setEnabled(False)
        self.takeoff_button.setGeometry(QtCore.QRect(680, 280, 150, 40))
        self.takeoff_button.setObjectName("takeoff_button")
        self.land_button = QtWidgets.QPushButton(self.centralwidget)
        self.land_button.setGeometry(QtCore.QRect(680, 330, 150, 40))
        self.land_button.setObjectName("land_button")
        self.disarm_button = QtWidgets.QPushButton(self.centralwidget)
        self.disarm_button.setGeometry(QtCore.QRect(680, 380, 150, 40))
        self.disarm_button.setObjectName("disarm_button")
        self.start_text = QtWidgets.QLabel(self.centralwidget)
        self.start_text.setGeometry(QtCore.QRect(680, 70, 71, 40))
        self.start_text.setObjectName("start_text")
        self.secs_text = QtWidgets.QLabel(self.centralwidget)
        self.secs_text.setGeometry(QtCore.QRect(820, 70, 10, 40))
        self.secs_text.setObjectName("secs_text")
        self.tableView = QtWidgets.QTableView(self.centralwidget)
        self.tableView.setGeometry(QtCore.QRect(20, 20, 640, 400))
        self.tableView.setObjectName("tableView")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 850, 39))
        self.menubar.setObjectName("menubar")
        self.menuOptions = QtWidgets.QMenu(self.menubar)
        self.menuOptions.setObjectName("menuOptions")
        MainWindow.setMenuBar(self.menubar)
        self.action_send_animations = QtWidgets.QAction(MainWindow)
        self.action_send_animations.setObjectName("action_send_animations")
        self.action_send_configurations = QtWidgets.QAction(MainWindow)
        self.action_send_configurations.setObjectName("action_send_configurations")
        self.menuOptions.addAction(self.action_send_animations)
        self.menuOptions.addAction(self.action_send_configurations)
        self.menubar.addAction(self.menuOptions.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Clever Drone Animation Player"))
        self.check_button.setText(_translate("MainWindow", "Preflight check"))
        self.start_button.setText(_translate("MainWindow", "Start animation"))
        self.pause_button.setText(_translate("MainWindow", "Pause"))
        self.stop_button.setText(_translate("MainWindow", "Stop"))
        self.takeoff_button.setText(_translate("MainWindow", "Takeoff"))
        self.land_button.setText(_translate("MainWindow", "Land"))
        self.disarm_button.setText(_translate("MainWindow", "Disarm"))
        self.start_text.setText(_translate("MainWindow", "Start after"))
        self.secs_text.setText(_translate("MainWindow", "s"))
        self.menuOptions.setTitle(_translate("MainWindow", "Actions"))
        self.action_send_animations.setText(_translate("MainWindow", "Send Animations"))
        self.action_send_configurations.setText(_translate("MainWindow", "Send Configurations"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

