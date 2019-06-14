# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'emergency.ui'
#
# Created by: PyQt5 UI code generator 5.11.3
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets
import os
import glob

from PyQt5 import QtWidgets
from PyQt5.QtGui import QStandardItemModel, QStandardItem
from PyQt5.QtCore import Qt, pyqtSlot, pyqtSignal, QObject

from PyQt5.QtWidgets import QDialog

# Importing gui form
from server_qt import *
from server import *

class Ui_Dialog(object):

    def __init__(self):
        self.Dialog = None
    def setupUi(self, Dialog):
        self.Dialog = Dialog
        Dialog.setObjectName("Dialog")
        Dialog.resize(746, 620)
        Dialog.setStyleSheet("QDialog{\n"
"background-color: #fffdd0;\n"
"}")
        self.two_button = QtWidgets.QPushButton(Dialog)
        self.two_button.setGeometry(QtCore.QRect(420, 120, 231, 171))
        self.two_button.setSizeIncrement(QtCore.QSize(16, 16))
        self.two_button.setStyleSheet("QPushButton{\n"
"color: white;\n"
"font-weight: 600;\n"
"font-size: 25pt;\n"
"background-color: red;\n"
"}")
        self.two_button.setObjectName("two_button")
        self.label = QtWidgets.QLabel(Dialog)
        self.label.setGeometry(QtCore.QRect(90, 30, 561, 51))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.one_button = QtWidgets.QPushButton(Dialog)
        self.one_button.setGeometry(QtCore.QRect(90, 120, 231, 171))
        self.one_button.setSizeIncrement(QtCore.QSize(16, 16))
        self.one_button.setStyleSheet("QPushButton{\n"
"color: white;\n"
"font-weight: 600;\n"
"font-size: 25pt;\n"
"background-color: RGB(118, 255, 122);\n"
"}")
        self.one_button.setObjectName("one_button")
        self.land_emergency_button = QtWidgets.QPushButton(Dialog)
        self.land_emergency_button.setGeometry(QtCore.QRect(90, 340, 561, 81))
        self.land_emergency_button.setStyleSheet("QPushButton{\n"
"font-weight: 600;\n"
"font-size: 25pt;\n"
"background-color: white;\n"
"}")
        self.land_emergency_button.setObjectName("land_emergency_button")
        self.disarm_emergency_button = QtWidgets.QPushButton(Dialog)
        self.disarm_emergency_button.setGeometry(QtCore.QRect(90, 460, 561, 81))
        self.disarm_emergency_button.setStyleSheet("QPushButton{\n"
"font-weight: 600;\n"
"font-size: 25pt;\n"
"background-color: white;\n"
"}")
        self.disarm_emergency_button.setObjectName("disarm_emergency_button")
        self.one_button.clicked.connect(self.one_button_click)
        self.two_button.clicked.connect(self.two_button_click)
        self.land_emergency_button.clicked.connect(self.land_emergency_click)
        self.disarm_emergency_button.clicked.connect(self.disarm_emergency_click)

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.two_button.setText(_translate("Dialog", "2"))
        self.label.setText(_translate("Dialog", "\n"
"Select a group in which the drone does not work correctly"))
        self.one_button.setText(_translate("Dialog", "1"))
        self.land_emergency_button.setText(_translate("Dialog", "Land"))
        self.disarm_emergency_button.setText(_translate("Dialog", "Disarm")) 
    def one_button_click(self):
        self.Dialog.done(1)
    def two_button_click(self):
        self.Dialog.done(2)
    def land_emergency_click(self):
        self.Dialog.done(3)
    def disarm_emergency_click(self):
        self.Dialog.done(4) 
        
        
if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.show()
    sys.exit(app.exec_())

