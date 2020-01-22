from functools import partial
from copy import deepcopy
import logging

from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import Qt as Qt
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtGui import QCursor
from PyQt5.QtWidgets import QTableView, QMessageBox, QMenu, QAction, QWidgetAction, QListWidget, \
    QAbstractItemView, QListWidgetItem, QVBoxLayout, QHBoxLayout, QPushButton, QInputDialog, QLineEdit

from config_editor_models import ConfigDialog
import copter_table_models as table


class CopterTableWidget(QTableView):
    def __init__(self, model, config, data_model=table.StatedCopterData):
        QTableView.__init__(self)

        self.config = config
        self.model = model
        self._data_model = data_model

        self.proxy_model = table.CopterProxyModel()
        self.signals = table.SignalManager(self.model)

        self.proxy_model.setSourceModel(self.model)
        self.proxy_model.setDynamicSortFilter(True)

        # Initiate table and table self.model
        self.setModel(self.proxy_model)

        self.columns = list(table.columns_names.keys())  #[header.strip() for header in self.model.headers]  # header keys
        self.current_columns = self.columns[:]

        header = self.horizontalHeader()
        header.setCascadingSectionResizes(False)
        header.setStretchLastSection(True)
        header.setSectionsMovable(True)
        header.sectionMoved.connect(self.moved)
        header.setContextMenuPolicy(Qt.CustomContextMenu)
        header.customContextMenuRequested.connect(self.showHeaderMenu)

        self.setContextMenuPolicy(Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self.open_menu)

        # Adjust properties
        self.setTextElideMode(QtCore.Qt.ElideMiddle)
        self.setWordWrap(True)

        self.setSizeAdjustPolicy(QtWidgets.QAbstractScrollArea.AdjustToContents)
        self.resizeColumnsToContents()
        self.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.doubleClicked.connect(self.on_double_click)

    def moved(self, logical_index, old_index, new_index):
        name = self.current_columns.pop(old_index)
        self.current_columns.insert(new_index, name)

    def set_column_order(self, order):
        if set(order) != set(self.current_columns):
            raise ValueError

        for index_to, item in enumerate(order):
            index_from = self.current_columns.index(item)
            if index_to != index_from:
                self.horizontalHeader().moveSection(index_from, index_to)

    def load_columns(self, item_dict: dict=None):
        if item_dict is None:
            item_dict = self.config.table_presets[self.config.table_presets_current]

        self.set_column_order(list(item_dict.keys()))
        for index, show in enumerate(item_dict.values()):
            self.horizontalHeader().setColumnHidden(index, not show)

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
        header_view = HeaderEditWidget(self, self.config, menu_mode=True, parent=menu)
        #header_view.setFixedHeight((header_view.geometry().height()-2) * len(header_view.columns))
        # box.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        action = QWidgetAction(menu)
        action.setDefaultWidget(header_view)
        menu.addAction(action)
        menu.exec_(QCursor.pos())
        # todo header_view.

    @pyqtSlot(QtCore.QPoint)
    def open_menu(self, point):
        menu = QMenu(self)
        index = self.indexAt(point)
        item = self.model.get_row_data(index)
        # print(item, index.row(), index.column())

        edit_config = QAction("Edit config")
        edit_config.triggered.connect(partial(self.edit_copter_config, item))
        menu.addAction(edit_config)

        if item is None:
            edit_config.setDisabled(True)

        menu.exec_(QCursor.pos())

    @pyqtSlot()
    def edit_copter_config(self, copter):
        dialog = ConfigDialog()
        copter.client.get_response("config", dialog.call_copter_dialog)

    # def _selfcheck_shortener(self, data):  # TODO!!!
    #     shortened = []
    #     for line in data:
    #         if len(line) > 89:
    #             pass
    #     return shortened


class HeaderListWidget(QListWidget):
    ColumnKeyRole = 998

    def __init__(self, config, parent=None, default_items=None):
        super().__init__(parent)
        self.populated_items = {}
        if default_items is not None:
            self.populate_items(default_items)

        self.config = config
        self.setDragDropMode(QAbstractItemView.InternalMove)
        self.setDefaultDropAction(Qt.MoveAction)

    def populate_items(self, item_dict: dict):
        self.populated_items = item_dict
        self.clear()
        for name, visible in item_dict.items():
            flags = Qt.ItemIsUserCheckable | Qt.ItemIsSelectable | Qt.ItemIsDragEnabled | Qt.ItemIsEnabled
            state = Qt.Checked if visible else Qt.Unchecked

            item = QListWidgetItem(table.columns_names.get(name, "").strip() or name, self)
            item.setFlags(flags)
            item.setCheckState(state)
            item.setData(HeaderListWidget.ColumnKeyRole, name)

    @property
    def item_dict(self):
        return {self.item(i).data(HeaderListWidget.ColumnKeyRole): bool(self.item(i).checkState())
                for i in range(self.count())}

    def save_to_config(self, current=None, write=False):
        if self.count() != len(self.populated_items):  # Do not save when populating
           print("nosave")
           return

        if current is None:
            current = self.config.table_presets_current
        print("sAVe", current)
        presets = self.config.table_presets
        header_dict = self.item_dict

        for key in presets[HeaderEditWidget.default]:
            print(key, current, header_dict, presets)
            if key not in presets[current] and not header_dict[key]:
                header_dict.pop(key)

        presets[current] = header_dict
        if write:
            self.config.write()


class ActiveHeaderListWidget(HeaderListWidget):
    def __init__(self, source: CopterTableWidget, config, parent=None):
        super().__init__(config, parent=parent)
        self.source_widget = source

        self.config = config
        self.current_columns = source.current_columns
        self.columns = source.columns

        self._populate_from_widget()

        self.itemChanged.connect(self.on_itemChanged)

    def _populate_from_widget(self):
        item_dict = {}
        for column, name in enumerate(self.current_columns):
            visible = not self.source_widget.isColumnHidden(column)
            item_dict[name] = visible

        self.populate_items(item_dict)

    @pyqtSlot(QListWidgetItem)
    def on_itemChanged(self, item):
        key = item.data(HeaderListWidget.ColumnKeyRole)
        if key is None:
            return
        self.source_widget.setColumnHidden(self.columns.index(key),
                                           not bool(item.checkState()))
        self.save_to_config(write=True)

    def dropEvent(self, event: QtGui.QDropEvent):
        super().dropEvent(event)
        column_order = [self.item(i).data(HeaderListWidget.ColumnKeyRole) for i in range(self.count())]
        self.source_widget.set_column_order(column_order)
        self.save_to_config(write=True)


class HeaderEditWidget(QtWidgets.QWidget):
    add_new_text = "< add new >"
    default = "DEFAULT"

    def __init__(self, source, config_data, menu_mode=False, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # self.auto_apply = auto_apply
        self.source = source  # source = copter table
        self.config = config_data
        self.menu_mode = menu_mode

        self.preset_widget = QtWidgets.QComboBox()
        self.header_widget = ActiveHeaderListWidget(self.source, self.config) \
            if self.menu_mode else HeaderListWidget(self.config)

        self.previous = self.config.table_presets_current

        self.setupUi()

    def setupUi(self):
        self.header_widget.setSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)

        self.update_preset_list()
        self.preset_widget.currentTextChanged.connect(self.on_preset_changed)
        self.on_preset_changed(self.previous)  # to init

        vbox = QVBoxLayout()
        vbox.addWidget(self.header_widget)
        vbox.addWidget(self.preset_widget)

        hbox = QHBoxLayout()
        if not self.menu_mode:
            add_button = QPushButton("Add")
            add_button.clicked.connect(self.add_preset)
            remove_button = QPushButton("Remove")
            remove_button.setToolTip("Permanently remove preset from config")
            remove_button.clicked.connect(self.remove_preset)
            save_button = QPushButton("Save")
            save_button.clicked.connect(self.save_preset)
            apply_button = QPushButton("Apply")
            apply_button.clicked.connect(self.apply_preset)

            hbox.addWidget(add_button)
            hbox.addWidget(remove_button)
            hbox.addStretch()
            hbox.addWidget(save_button)
            hbox.addWidget(apply_button)
        else:
            dialog_button = QPushButton("Manage presets")
            hbox.addWidget(dialog_button)

        vbox.addLayout(hbox)
        self.setLayout(vbox)

    def update_preset_list(self):
        self.preset_widget.clear()
        for name, preset in self.config.table_presets.items():
            if isinstance(preset, dict):  # looking only for preset sections
                self.preset_widget.addItem(name)

        self.preset_widget.addItem(self.add_new_text)
        self.preset_widget.setCurrentText(self.previous)

    def on_preset_changed(self, index):
        if not index:
            return

        if index == self.add_new_text:
            self.add_preset()
            return

        self.previous = index
        presets = self.config.table_presets
        items = {key: value for key, value in presets[index].items()}
        items.update({key: False for key in presets[self.default] if key not in items})

        if self.menu_mode:
            self.source.set_column_order(list(items.keys()))
            self.config.table_presets_current = index
            print(index)
            self.config.write()
        self.header_widget.populate_items(items)

    def add_preset(self):
        name, ok = QInputDialog.getText(None, "Enter new preset name", "Name:",
                                        QLineEdit.Normal, "")
        if not ok or not name:
            self.preset_widget.setCurrentText(self.previous)
            return

        if name in self.config.table_presets or name == self.default or name == self.add_new_text:
            QMessageBox.warning(None, "Preset already exists!", "Preset already exists!")
            self.preset_widget.setCurrentText(self.previous)
            return

        self.config.table_presets[name] = deepcopy(dict(self.config.table_presets[self.default]))
        self.config.write()

        self.update_preset_list()
        self.preset_widget.setCurrentText(name)

    def remove_preset(self):
        if self.preset_widget.currentText() == self.default:
            QMessageBox.warning(None, "Can't delete default preset!", "Can't delete default preset!")
            return

        reply = QMessageBox.question(None, "Action can't be undone", "Remove anyway?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if reply != QMessageBox.Yes:
            return

        self.config.table_presets.pop(self.preset_widget.currentText())
        self.config.write()

        self.previous = self.default
        self.update_preset_list()

    def save_preset(self):
        self.header_widget.save_to_config(current=self.preset_widget.currentText(), write=True)

    def apply_preset(self):
        self.config.table_presets_current = self.preset_widget.currentText()
        self.save_preset()
        self.source.load_columns()


if __name__ == '__main__':
    import sys

    def except_hook(cls, exception, traceback):
        sys.__excepthook__(cls, exception, traceback)
    sys.excepthook = except_hook  # for debugging (exceptions traceback)

    app = QtWidgets.QApplication(sys.argv)
    import copter_table_models

    model = copter_table_models.CopterDataModel()
    # for i in range(10):
    #     model.add_client(copter_table_models.StatedCopterData())

    import config
    c = config.ConfigManager()
    c.load_config_and_spec("config\server.ini")
    #print(c.config)
    #print(c._name_dict)
    w1 = CopterTableWidget(model, c)
    w = HeaderEditWidget(w1, c)
    # print(*w1.current_columns, sep='\n')
    w.show()
    app.exec()
