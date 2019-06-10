import os
import glob

from PyQt5 import QtWidgets
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import QFileDialog, QMessageBox
from PyQt5 import QtCore, QtGui, QtWidgets
from server import *
from server_qt import *

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(632, 214)
        self.pushButton_2 = QtWidgets.QPushButton(Dialog)
        self.pushButton_2.setGeometry(QtCore.QRect(470, 110, 121, 61))
        self.pushButton_2.setSizeIncrement(QtCore.QSize(16, 16))
        self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton_3 = QtWidgets.QPushButton(Dialog)
        self.pushButton_3.setGeometry(QtCore.QRect(40, 100, 121, 61))
        self.pushButton_3.setSizeIncrement(QtCore.QSize(16, 16))
        self.pushButton_3.setObjectName("pushButton_3")
        self.pushButton_2.clicked.connect(self.btn_2)
        self.pushButton_3.clicked.connect(self.btn_3)
        self.label = QtWidgets.QLabel(Dialog)
        self.label.setGeometry(QtCore.QRect(40, 40, 561, 51))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.label.setFont(font)
        self.label.setObjectName("label")

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.pushButton_2.setText(_translate("Dialog", "PushButton"))
        self.pushButton_3.setText(_translate("Dialog", "PushButton"))
        self.label.setText(_translate("Dialog", "\n"
"Select a group in which the drone does not work correctly"))
    def btn_2(self):
        for row_num in range(model.rowCount()):
            item = model.item(row_num, 0)
            if item.isCheckable() and item.checkState() == Qt.Checked:
                copter = Client.get_by_id(item.text())
                copter.send_message("green")
    def btn_3(self):
        for row_num in range(model.rowCount()):
            item = model.item(row_num, 0)
            if item.isCheckable() and item.checkState() == Qt.Checked:
                copter = Client.get_by_id(item.text())
                copter.send_message("red")

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.show()
    sys.exit(app.exec_())

