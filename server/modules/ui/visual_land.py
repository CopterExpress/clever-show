# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'visual_land.ui'
#
# Created by: PyQt5 UI code generator 5.13.0
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(746, 620)
        Dialog.setStyleSheet("")
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
        self.label.setGeometry(QtCore.QRect(60, 30, 631, 51))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.label.setFont(font)
        self.label.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.one_button = QtWidgets.QPushButton(Dialog)
        self.one_button.setGeometry(QtCore.QRect(90, 120, 231, 171))
        self.one_button.setSizeIncrement(QtCore.QSize(16, 16))
        self.one_button.setStyleSheet("QPushButton{\n"
"color: white;\n"
"font-weight: 600;\n"
"font-size: 25pt;\n"
"background-color: green;\n"
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

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Visual land"))
        self.two_button.setText(_translate("Dialog", "2"))
        self.label.setText(_translate("Dialog", "Select the group with the defective copter"))
        self.one_button.setText(_translate("Dialog", "1"))
        self.land_emergency_button.setText(_translate("Dialog", "Land"))
        self.disarm_emergency_button.setText(_translate("Dialog", "Disarm"))
