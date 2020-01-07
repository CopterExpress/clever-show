from functools import partial

from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import Qt as Qt
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtGui import QCursor
from PyQt5.QtWidgets import QTableView, QMessageBox, QMenu, QAction, QWidgetAction, QListWidget, \
    QAbstractItemView, QListWidgetItem

from config_editor_models import ConfigDialog
import copter_table_models as table


class CopterTableWidget(QTableView):
    config_dialog_signal = QtCore.pyqtSignal(object, object)

    def __init__(self, model, window, data_model=table.StatedCopterData):
        QTableView.__init__(self)

        self._window = window
        self.model = model
        self._data_model = data_model

        self.proxy_model = table.CopterProxyModel()
        self.signals = table.SignalManager(self.model)

        self.proxy_model.setSourceModel(self.model)
        self.proxy_model.setDynamicSortFilter(True)

        # Initiate table and table self.model
        self.setModel(self.proxy_model)

        header = self.horizontalHeader()
        header.setSectionsMovable(True)
        header.setContextMenuPolicy(Qt.CustomContextMenu)
        header.customContextMenuRequested.connect(self.showHeaderMenu)

        self.setContextMenuPolicy(Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self.open_menu)

        self.signal_connection = None
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

    def showHeaderMenu(self, event):
        menu = QMenu(self)
        header_view = HeaderListWidget(menu, self)
        header_view.setFixedHeight((header_view.geometry().height()-6)*len(header_view.names))
        #box.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        action = QWidgetAction(menu)
        action.setDefaultWidget(header_view)
        menu.addAction(action)
        menu.exec_(QCursor.pos())

    @pyqtSlot(QtCore.QPoint)
    def open_menu(self, point):
        menu = QMenu(self)
        index = self.indexAt(point)
        item = self.model.get_row_data(index)
        # print(item, index.row(), index.column())

        edit_config = QAction("Edit config")
        edit_config.triggered.connect(partial(self.edit_config, item))
        menu.addAction(edit_config)

        if item is None:
            edit_config.setDisabled(True)

        menu.exec_(QCursor.pos())

    @pyqtSlot()
    def edit_config(self, copter):
        if self.signal_connection is not None:
            self.config_dialog_signal.disconnect(self.signal_connection)

        call = ConfigDialog(self._window).call_copter_dialog
        self.signal_connection = self.config_dialog_signal.connect(call)
        copter.client.get_response("config", self.config_dialog_signal.emit)

    # def _selfcheck_shortener(self, data):  # TODO!!!
    #     shortened = []
    #     for line in data:
    #         if len(line) > 89:
    #             pass
    #     return shortened


class HeaderListWidget(QListWidget):
    def __init__(self, parent, source: CopterTableWidget):
        super().__init__(parent)
        self.source_widget = source
        self.source_model = source.proxy_model

        self.setDragDropMode(QAbstractItemView.InternalMove)
        self.setDefaultDropAction(Qt.MoveAction)

        self.names = list(self.get_names())
        self.populate_items()
        self.itemChanged.connect(self.on_itemChanged)

    def get_names(self):
        for column in range(self.source_model.columnCount()):
            yield self.source_model.headerData(column, Qt.Horizontal).strip()

    def populate_items(self):
        for column, name in enumerate(self.names):
            hidden = self.source_widget.isColumnHidden(column)
            flags = Qt.ItemIsUserCheckable | Qt.ItemIsSelectable | Qt.ItemIsDragEnabled | Qt.ItemIsEnabled
            state = Qt.Unchecked if hidden else Qt.Checked

            item = QListWidgetItem(name, self)
            item.setFlags(flags)
            item.setCheckState(state)

    @pyqtSlot(QListWidgetItem)
    def on_itemChanged(self, item):
        self.source_widget.setColumnHidden(self.names.index(item.text()), not bool(item.checkState()))
