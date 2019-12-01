# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'config_editor.ui'
#
# Created by: PyQt5 UI code generator 5.13.2
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_config_dialog(object):
    def setupUi(self, config_dialog):
        config_dialog.setObjectName("config_dialog")
        config_dialog.resize(600, 700)
        config_dialog.setModal(False)
        self.gridLayout = QtWidgets.QGridLayout(config_dialog)
        self.gridLayout.setObjectName("gridLayout")
        self.gridLayout_2 = QtWidgets.QGridLayout()
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.checkBox = QtWidgets.QCheckBox(config_dialog)
        self.checkBox.setObjectName("checkBox")
        self.gridLayout_2.addWidget(self.checkBox, 0, 0, 1, 1)
        self.buttonBox = QtWidgets.QDialogButtonBox(config_dialog)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Save)
        self.buttonBox.setCenterButtons(False)
        self.buttonBox.setObjectName("buttonBox")
        self.gridLayout_2.addWidget(self.buttonBox, 0, 1, 1, 1)
        self.gridLayout.addLayout(self.gridLayout_2, 3, 0, 1, 1)
        self.config_view = QtWidgets.QTreeView(config_dialog)
        self.config_view.setEditTriggers(QtWidgets.QAbstractItemView.DoubleClicked|QtWidgets.QAbstractItemView.EditKeyPressed|QtWidgets.QAbstractItemView.SelectedClicked)
        self.config_view.setObjectName("config_view")
        self.config_view.header().setCascadingSectionResizes(False)
        self.config_view.header().setDefaultSectionSize(250)
        self.gridLayout.addWidget(self.config_view, 0, 0, 1, 1)
        self.gridLayout_3 = QtWidgets.QGridLayout()
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.delete_button = QtWidgets.QPushButton(config_dialog)
        self.delete_button.setObjectName("delete_button")
        self.gridLayout_3.addWidget(self.delete_button, 0, 1, 1, 1)
        self.add_option_button = QtWidgets.QPushButton(config_dialog)
        self.add_option_button.setObjectName("add_option_button")
        self.gridLayout_3.addWidget(self.add_option_button, 0, 3, 1, 1)
        self.add_section_button = QtWidgets.QPushButton(config_dialog)
        self.add_section_button.setObjectName("add_section_button")
        self.gridLayout_3.addWidget(self.add_section_button, 0, 2, 1, 1)
        self.hard_delete = QtWidgets.QPushButton(config_dialog)
        self.hard_delete.setObjectName("hard_delete")
        self.gridLayout_3.addWidget(self.hard_delete, 0, 0, 1, 1)
        self.gridLayout.addLayout(self.gridLayout_3, 1, 0, 1, 1)
        self.line = QtWidgets.QFrame(config_dialog)
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.gridLayout.addWidget(self.line, 2, 0, 1, 1)

        self.retranslateUi(config_dialog)
        self.buttonBox.accepted.connect(config_dialog.accept)
        self.buttonBox.rejected.connect(config_dialog.reject)
        QtCore.QMetaObject.connectSlotsByName(config_dialog)

    def retranslateUi(self, config_dialog):
        _translate = QtCore.QCoreApplication.translate
        config_dialog.setWindowTitle(_translate("config_dialog", "Config Editor"))
        self.checkBox.setText(_translate("config_dialog", "Restart"))
        self.checkBox.setShortcut(_translate("config_dialog", "R"))
        self.delete_button.setText(_translate("config_dialog", "Delete"))
        self.delete_button.setShortcut(_translate("config_dialog", "Del"))
        self.add_option_button.setText(_translate("config_dialog", "Add option"))
        self.add_section_button.setText(_translate("config_dialog", "Add section"))
        self.add_section_button.setShortcut(_translate("config_dialog", "Ctrl+A"))
        self.hard_delete.setText(_translate("config_dialog", "Mark for deletion"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    config_dialog = QtWidgets.QDialog()
    ui = Ui_config_dialog()
    ui.setupUi(config_dialog)
    config_dialog.show()
    sys.exit(app.exec_())
