from functools import partial
from copy import deepcopy

from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import Qt as Qt, QObject, QEvent, QModelIndex
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtGui import QCursor
from PyQt5.QtWidgets import QTableView, QMessageBox, QMenu, QAction, QWidgetAction, QListWidget, \
    QAbstractItemView, QListWidgetItem, QVBoxLayout, QHBoxLayout, QPushButton, QInputDialog, QLineEdit, QApplication

from modules.config_editor_models import ConfigDialog
import modules.copter_table_models as table


def save_preset(config, current, header_dict):
    presets = config.table_presets

    for key in presets[HeaderEditWidget.default]:
        if key not in presets[current] and not header_dict[key][0]:
            header_dict.pop(key)

    presets[current] = header_dict
    # config.write()


class HeaderViewFilter(QObject):
    def __init__(self, parent, header, *args):
        super().__init__(parent, *args)
        self.header = header
        self._parent = parent

    def eventFilter(self, object, event):
        if event.type() == QEvent.Enter:
            # logicalIndex = self.header.logicalIndexAt(event.pos())
            self.parent().cellHover.emit(QModelIndex())
        else:
            return False
        return True


class CopterTableWidget(QTableView):
    override_cursors = {
        "copter_id": Qt.IBeamCursor,
        "config_version": Qt.OpenHandCursor,
        "selfcheck": Qt.PointingHandCursor,
    }

    cellHover = QtCore.pyqtSignal(QModelIndex)
    cellEntered = QtCore.pyqtSignal(int, int)
    cellExited = QtCore.pyqtSignal(int, int)

    def __init__(self, model: table.CopterDataModel, config):
        QTableView.__init__(self)

        self.config = config
        self.model = model

        self.proxy_model = table.CopterProxyModel()
        self.proxy_model.setSourceModel(self.model)
        self.proxy_model.setDynamicSortFilter(True)

        # Initiate table and table self.model
        self.setModel(self.proxy_model)

        self.columns = self.model.columns  # [header.strip() for header in self.model.headers]  # header keys
        self.current_columns = self.columns[:]

        self._last_hover_index = QtCore.QModelIndex()
        self._previous_cursor = None

        self.cellHover.connect(self.cell_hover)
        self.cellExited.connect(self.cell_exited)
        self.cellEntered.connect(self.cell_entered)

        header = self.horizontalHeader()
        self.filter = HeaderViewFilter(self, header)
        header.installEventFilter(self.filter)
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
        self.setSortingEnabled(True)
        self.setSizeAdjustPolicy(QtWidgets.QAbstractScrollArea.AdjustToContents)
        self.resizeColumnsToContents()
        self.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectItems)
        self.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.doubleClicked.connect(self.on_double_click)
        self.setDragDropMode(QAbstractItemView.DragDrop)
        self.setMouseTracking(True)

    def mousePressEvent(self, event):
        super().mousePressEvent(event)
        index = self.indexAt(event.pos())
        if index.column() == -1 and index.row() == -1:
            self.clearSelection()

    def mouseMoveEvent(self, event):
        self.cell_hover(self.indexAt(event.pos()))
        super().mouseMoveEvent(event)

    def leaveEvent(self, event):
        self.cell_hover(QtCore.QModelIndex())

    def dragEnterEvent(self, *args, **kwargs):
        self.cell_hover(QtCore.QModelIndex())
        super().dragEnterEvent(*args, **kwargs)

    def cell_hover(self, index):
        if index != self._last_hover_index:
            self.cellExited.emit(self._last_hover_index.row(), self._last_hover_index.column())
            self.cellEntered.emit(index.row(), index.column())

            self._last_hover_index = QtCore.QPersistentModelIndex(index)

    @pyqtSlot(int, int)
    def cell_entered(self, row, column):
        if column != -1 and self.columns[column] in self.override_cursors:
            self._previous_cursor = QApplication.overrideCursor()
            if self._previous_cursor is None:
                QApplication.setOverrideCursor(self.override_cursors[self.columns[column]])

    @pyqtSlot(int, int)
    def cell_exited(self, row, column):
        # if self._previous_cursor is not None:
        #    QApplication.setOverrideCursor(self._previous_cursor)
        if self._previous_cursor is None:
            QApplication.restoreOverrideCursor()

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

    def load_columns(self, item_dict: dict = None):
        presets = self.config.table_presets
        if item_dict is None:
            item_dict = presets[self.config.table_presets_current]

        item_dict.update({key: (False, presets[HeaderEditWidget.default][key][1])
                          for key in presets[HeaderEditWidget.default] if key not in item_dict})

        self.set_column_order(item_dict.keys())
        # self.set_column_widths({key: val[1] for key, val in item_dict.items()})

        for name, value in item_dict.items():  # for index, name in enumerate(self.columns):
            index = self.columns.index(name)
            show, width = value
            self.setColumnHidden(index, not show)  # self.setColumnHidden(index, not item_dict.get(name, False))
            self.setColumnWidth(index, width)

    def _get_column_item(self, column):
        index = self.columns.index(column)
        presets = self.config.table_presets
        show = not self.isColumnHidden(index)
        # columnWidth is 0 when hidden, trying to get previous width from config or default
        width = self.columnWidth(index) or \
                presets[self.config.table_presets_current].get(column, 0)[1] or \
                presets[HeaderEditWidget.default][column][1]
        return show, width

    @property
    def item_dict(self):
        return {column: self._get_column_item(column) for column in self.current_columns}

    def save_columns(self):
        current = self.config.table_presets_current
        header_dict = self.item_dict
        save_preset(self.config, current, header_dict)

    def select_all(self, state):
        for i in range(self.model.rowCount()):
            self.model.update_data(i, 0, state, Qt.CheckStateRole)

    def toggle_select(self):
        if len(list(self.model.user_selected())) == self.model.rowCount():  # if all items are selected
            state = Qt.Unchecked
        else:
            state = Qt.Checked
        self.select_all(state)

    @pyqtSlot(QtCore.QModelIndex)
    def on_double_click(self, index):
        if self.model.is_column(index, "selfcheck"):
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
        self.save_columns()
        menu = QMenu(self)
        header_view = HeaderEditWidget(self, self.config, menu_mode=True, parent=menu)
        # header_view.setFixedHeight((header_view.geometry().height()-2) * len(header_view.columns))
        # box.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        action = QWidgetAction(menu)
        action.setDefaultWidget(header_view)
        menu.addAction(action)
        menu.exec_(QCursor.pos())
        header_view.save_preset()

    @pyqtSlot(QtCore.QPoint)
    def open_menu(self, point):
        menu = QMenu(self)
        id = self.indexAt(point).siblingAtColumn(0).data()
        item = self.model.get_row_by_attr('copter_id', id)

        edit_config = QAction("Edit config")
        edit_config.triggered.connect(partial(self.edit_copter_config, item))
        menu.addAction(edit_config)

        copy_config = QAction("Copy config to selected")
        copy_config.triggered.connect(partial(self.copy_config, item))
        menu.addAction(copy_config)

        if item is None:
            edit_config.setDisabled(True)
            copy_config.setDisabled(True)

        menu.exec_(QCursor.pos())

    @pyqtSlot()
    def edit_copter_config(self, copter):
        dialog = ConfigDialog()
        copter.client.get_response("config", dialog.call_copter_dialog, request_kwargs={'send_configspec': True})

    @pyqtSlot()
    def copy_config(self, copter):
        def send_callback(client, value):
            config = value["config"]
            config.pop("PRIVATE", None)  # delete private section

            for _copter in self.model.user_selected():
                if _copter.client is client:
                    continue  # don't send config back to the same copter
                _copter.client.send_message("config", kwargs={"config": config, "mode": "modify"})

        copter.client.get_response("config", send_callback, request_kwargs={'send_configspec': False})

    # def _selfcheck_shortener(self, data):  # TODO!!!
    #     shortened = []
    #     for line in data:
    #         if len(line) > 89:
    #             pass
    #     return shortened


class HeaderListWidget(QListWidget):
    ColumnKeyRole = Qt.UserRole + 1000
    ColumnWidthRole = Qt.UserRole + 1001

    dropped = QtCore.pyqtSignal(bool)

    def __init__(self, parent=None, default_items=None):
        super().__init__(parent)
        if default_items is not None:
            self.populate_items(default_items)

        self.setDragDropMode(QAbstractItemView.InternalMove)
        self.setDefaultDropAction(Qt.MoveAction)

    def populate_items(self, item_dict: dict):
        self.clear()
        for name, value in item_dict.items():
            visible, width = value
            flags = Qt.ItemIsUserCheckable | Qt.ItemIsSelectable | Qt.ItemIsDragEnabled | Qt.ItemIsEnabled
            state = Qt.Checked if visible else Qt.Unchecked

            item = QListWidgetItem(table.CopterDataModel.columns_dict.get(name, "").strip() or name, self)
            item.setFlags(flags)
            item.setCheckState(state)
            item.setData(self.ColumnKeyRole, name)
            item.setData(self.ColumnWidthRole, width)

    @property
    def item_dict(self):
        return {self.item(i).data(self.ColumnKeyRole):
                    (bool(self.item(i).checkState()), self.item(i).data(self.ColumnWidthRole))
                for i in range(self.count())}

    def dropEvent(self, event: QtGui.QDropEvent):
        super().dropEvent(event)
        self.dropped.emit(True)


class ActiveHeaderListWidget(HeaderListWidget):
    def __init__(self, source: CopterTableWidget, parent=None):
        super().__init__(parent=parent)
        self.source_widget = source

        self.current_columns = source.current_columns
        self.columns = source.columns

        self._populate_from_widget()

        self.itemChanged.connect(self.on_itemChanged)

    def _populate_from_widget(self):
        self.populate_items(self.source_widget.item_dict)

    @pyqtSlot(QListWidgetItem)
    def on_itemChanged(self, item):
        key = item.data(HeaderListWidget.ColumnKeyRole)
        if key is None:
            return
        self.source_widget.setColumnHidden(self.columns.index(key), not bool(item.checkState()))

    def dropEvent(self, event: QtGui.QDropEvent):
        super().dropEvent(event)
        column_order = [self.item(i).data(HeaderListWidget.ColumnKeyRole) for i in range(self.count())]
        self.source_widget.set_column_order(column_order)


class HeaderEditWidget(QtWidgets.QWidget):
    add_new_text = "< add new >"
    default = "DEFAULT"

    saved_signal = QtCore.pyqtSignal(bool)

    def __init__(self, source, config, menu_mode=False, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # self.auto_apply = auto_apply
        self.source = source  # source = copter table
        self.config = config
        self.menu_mode = menu_mode

        self.preset_widget = QtWidgets.QComboBox()
        self.header_widget = ActiveHeaderListWidget(self.source) \
            if self.menu_mode else HeaderListWidget()
        # self.header_widget.itemChanged.connect(partial(self.saved_signal.emit, False))
        self.header_widget.model().dataChanged.connect(partial(self.saved_signal.emit, False))
        self.header_widget.dropped.connect(partial(self.saved_signal.emit, False))

        self.previous = self.config.table_presets_current
        self.save = True

        self.setupUi()

    @pyqtSlot()
    def call_dialog(self):
        self.save_preset()
        self.save = False
        HeaderEditDialog(self.source, self.config).exec()

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
            apply_button.setDefault(True)
            apply_button.setFocus()

            hbox.addWidget(add_button)
            hbox.addWidget(remove_button)
            hbox.addStretch()
            hbox.addWidget(save_button)
            hbox.addWidget(apply_button)
        else:
            dialog_button = QPushButton("Manage presets")
            dialog_button.clicked.connect(self.call_dialog)
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
        item_dict = {key: value for key, value in presets[index].items()}
        item_dict.update({key: (False, presets[self.default][key][1])
                          for key in presets[self.default] if key not in item_dict})

        if self.menu_mode:
            self.source.set_column_order(list(item_dict.keys()))  # hidden\shown is hold by header widget's itemChanged
            for name, value in item_dict.items():
                self.source.setColumnWidth(self.source.columns.index(name), value[1])

            self.config.table_presets_current = index
        self.header_widget.populate_items(item_dict)
        self.saved_signal.emit(True)

    def add_preset(self):
        name, ok = QInputDialog.getText(None, "Enter new preset name", "Name:",
                                        QLineEdit.Normal, "")
        if not ok or not name:
            self.preset_widget.setCurrentText(self.previous)
            return

        name = name.strip()
        if name in self.config.table_presets or name == self.default or name == self.add_new_text:
            QMessageBox.warning(None, "Preset already exists!", "Preset already exists!")
            self.preset_widget.setCurrentText(self.previous)
            return

        self.config.table_presets[name] = deepcopy(dict(self.config.table_presets[self.default]))
        # self.config.write()

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
        # self.config.write()

        self.previous = self.default
        self.update_preset_list()

    @pyqtSlot()
    def save_preset(self):
        if not self.save:  # don't save after calling dialog to avoid overrides
            return

        current = self.preset_widget.currentText()
        header_dict = self.header_widget.item_dict
        save_preset(self.config, current, header_dict)
        self.saved_signal.emit(True)

    @pyqtSlot()
    def apply_preset(self):
        self.config.table_presets_current = self.preset_widget.currentText()
        self.save_preset()
        self.source.load_columns()


class HeaderEditDialog(QtWidgets.QDialog):
    def __init__(self, source, config, parent=None):
        super(HeaderEditDialog, self).__init__(parent=None)
        self.widget = HeaderEditWidget(source, config, menu_mode=False)
        self.setupUI()
        self.unsaved = False

        self.widget.saved_signal.connect(self.update_title)
        self.update_title(True)

    def setupUI(self):
        layout = QVBoxLayout()
        layout.addWidget(self.widget)
        self.setLayout(layout)

    @pyqtSlot(bool)
    def update_title(self, saved):
        unsaved = not saved
        self.unsaved = unsaved
        self.setWindowTitle(f"Column preset editor - {self.widget.preset_widget.currentText()}"
                            + "*" * unsaved)

    def closeEvent(self, event):
        if not self.unsaved:
            event.accept()
            return

        reply = QMessageBox.question(self, "Confirm exit", "There are unsaved changes in current preset. "
                                                           "Are you sure you want to exit?",
                                     QMessageBox.No | QMessageBox.Yes, QMessageBox.No)

        if reply != QMessageBox.Yes:
            event.ignore()
        else:
            event.accept()


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
    # print(c.config)
    # print(c._name_dict)
    w1 = CopterTableWidget(model, c)
    w = HeaderEditWidget(w1, c)
    print(w1.item_dict)
    # print(*w1.current_columns, sep='\n')
    # w.show()
    app.exec()
