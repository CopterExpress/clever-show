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
        self.config_view = QtWidgets.QTreeView(config_dialog)
        self.config_view.setEditTriggers(QtWidgets.QAbstractItemView.DoubleClicked|QtWidgets.QAbstractItemView.EditKeyPressed|QtWidgets.QAbstractItemView.SelectedClicked)
        self.config_view.setObjectName("config_view")
        self.config_view.header().setCascadingSectionResizes(False)
        self.config_view.header().setDefaultSectionSize(250)
        self.gridLayout.addWidget(self.config_view, 0, 0, 1, 1)
        self.gridLayout_2 = QtWidgets.QGridLayout()
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.buttonBox = QtWidgets.QDialogButtonBox(config_dialog)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Save)
        self.buttonBox.setCenterButtons(False)
        self.buttonBox.setObjectName("buttonBox")
        self.gridLayout_2.addWidget(self.buttonBox, 0, 2, 1, 1)
        self.do_restart = QtWidgets.QCheckBox(config_dialog)
        self.do_restart.setObjectName("do_restart")
        self.gridLayout_2.addWidget(self.do_restart, 0, 1, 1, 1)
        self.do_coloring = QtWidgets.QCheckBox(config_dialog)
        self.do_coloring.setChecked(True)
        self.do_coloring.setObjectName("do_coloring")
        self.gridLayout_2.addWidget(self.do_coloring, 0, 0, 1, 1)
        self.gridLayout.addLayout(self.gridLayout_2, 2, 0, 1, 1)
        self.line = QtWidgets.QFrame(config_dialog)
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.gridLayout.addWidget(self.line, 1, 0, 1, 1)

        self.retranslateUi(config_dialog)
        self.buttonBox.accepted.connect(config_dialog.accept)
        self.buttonBox.rejected.connect(config_dialog.reject)
        QtCore.QMetaObject.connectSlotsByName(config_dialog)

    def retranslateUi(self, config_dialog):
        _translate = QtCore.QCoreApplication.translate
        config_dialog.setWindowTitle(_translate("config_dialog", "Config Editor"))
        self.do_restart.setText(_translate("config_dialog", "Restart"))
        self.do_restart.setShortcut(_translate("config_dialog", "R"))
        self.do_coloring.setText(_translate("config_dialog", "Color Indication"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    config_dialog = QtWidgets.QDialog()
    ui = Ui_config_dialog()
    ui.setupUi(config_dialog)
    config_dialog.show()
    sys.exit(app.exec_())
