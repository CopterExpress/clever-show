import logging

from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import Qt as Qt
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtGui import QCursor, QStandardItem
from PyQt5.QtWidgets import QTableView, QMessageBox, QMenu, QAction, QCheckBox, QWidgetAction, QListView, QListWidget, \
    QAbstractItemView, QListWidgetItem

from server import Client
import copter_table_models as table


class CopterTableWidget(QTableView):
    def __init__(self, model, data_model=table.StatedCopterData):
        QTableView.__init__(self)

        self.model = model
        self._data_model = data_model

        self.proxy_model = table.CopterProxyModel()
        self.signals = table.SignalManager(self.model)

        self.proxy_model.setSourceModel(self.model)
        self.proxy_model.setDynamicSortFilter(True)

        # Initiate table and table self.model
        self.setModel(self.proxy_model)

        self.horizontalHeader().setSectionsMovable(True)
        header = self.horizontalHeader()
        header.setContextMenuPolicy(Qt.CustomContextMenu)
        #header.setContextMenuPolicy(CustomContextMenu)
        header.customContextMenuRequested.connect(self.showHeaderMenu)

        # self.horizontalHeader().contextMenuEvent = self.headercontextMenuEvent

        # Adjust properties
        self.resizeColumnsToContents()
        self.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.doubleClicked.connect(self.on_double_click)

    # Some fancy wrappers to simplify syntax

    def add_client(self, **kwargs):
        self.signals.add_client_signal.emit(self._data_model(**kwargs))

    def remove_client_data(self, row_data):
        self.signals.remove_client_signal.emit(row_data)

    def update_data(self, row, col, data, role=table.ModelDataRole):
        self.signals.update_data_signal.emit(row, col, data, role)

    @pyqtSlot(QtCore.QModelIndex)
    def on_double_click(self, index):
        col = index.column()
        if col == 7:
            data = self.proxy_model.data(index, role=table.ModelDataRole)
            if data and data != "OK":
                self._show_info("Selfcheck info", data)

    def _show_info(self, title, data):
        dialog = QMessageBox()
        dialog.setIcon(QMessageBox.NoIcon)
        dialog.setStandardButtons(QMessageBox.Ok)
        dialog.setWindowTitle(title)
        dialog.setText("\n".join(data[:10]))
        dialog.setDetailedText("\n".join(data))
        dialog.exec()

    # action = QAction(name, menu, checkable=True, checked=True)
    def showHeaderMenu1(self, event):
        menu = QMenu(self)
        names = []
        for column in range(self.proxy_model.columnCount()):
            name = self.proxy_model.headerData(column, Qt.Horizontal).strip()
            names.append(name)
        #l = max(map(len, names))
        for name in names:
            # name = "{0:<{1}}|".format(name, l)
            box = QCheckBox(menu)
            action = QWidgetAction(menu)
            box.setText(name)
            action.setDefaultWidget(box)
            menu.addAction(action)
            #act = menu.addAction(name.strip())
        menu.exec_(QCursor.pos())

    def showHeaderMenu(self, event):
        menu = QMenu(self)
        names = []
        for column in range(self.proxy_model.columnCount()):
            name = self.proxy_model.headerData(column, Qt.Horizontal).strip()
            names.append(name)
        #l = max(map(len, names))

        box = QListWidget(menu)
        action = QWidgetAction(menu)
        #box.setText(name)
        # model = QtGui.QStandardItemModel()
        # box.setModel(model)
        box.setDragDropMode(QAbstractItemView.InternalMove)
        box.setDefaultDropAction(Qt.MoveAction)
        # #box.setMovement(QListView.Snap)
        # box.setDragDropOverwriteMode(False)

        for name in names:
            item = QListWidgetItem(name, box)
            item.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled | Qt.ItemIsDragEnabled | Qt.ItemIsSelectable)
            item.setCheckState(Qt.Checked)
            #item = box.addItem(name)#, checkable=True, checked=True)
            print(item)
            #item.setCheckable(True)
            #box.model().appendRow(item)

        box.setFixedHeight((box.geometry().height()-6)*len(names))
        #box.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        action.setDefaultWidget(box)
        menu.addAction(action)
        #act = menu.addAction(name.strip())
        menu.exec_(QCursor.pos())

    def contextMenuEvent(self, event):
        menu = QMenu(self)

        menu.addAction("LOl")
        menu.exec_(QCursor.pos())

    # def _selfcheck_shortener(self, data):  # TODO!!!
    #     shortened = []
    #     for line in data:
    #         if len(line) > 89:
    #             pass
    #     return shortened