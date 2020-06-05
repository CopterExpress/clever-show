import os
import sys
import pickle
import logging
from ast import literal_eval
from functools import partial
from copy import deepcopy

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtGui import QCursor, QKeySequence
from PyQt5.QtWidgets import QAbstractItemView, QTreeView, QMenu, QAction, QMessageBox, QInputDialog, QFileDialog, \
    QShortcut

import modules.ui.config_editor as config_editor

# Add parent dir to PATH to import messaging_lib and config_lib
current_dir = (os.path.dirname(os.path.realpath(__file__)))
lib_dir = os.path.realpath(os.path.join(current_dir, '../../lib'))
sys.path.insert(0, lib_dir)

import config


states_colors = {
    'normal': Qt.white,
    'unchanged': Qt.blue,
    'default': Qt.cyan,
    'edited': Qt.yellow,
    'added': Qt.green,
    'deleted': Qt.red,
}

StateRole = 999
TypeRole = 998


def convert_type(data):
    try:
        data = literal_eval(data) if data else None
    except (SyntaxError, ValueError):
        data = str(data)
    return data


class ConfigModelItem:
    def __init__(self, values=(None, None, None, None), item_type='option',
                 state='normal', default=None, parent=None):
        self.spec_default = default
        self.itemData = list(values)
        self.state = state
        self.type = item_type

        if isinstance(self.data(1), (list, tuple)):
            self.type = 'list'

        self.default_values = deepcopy(self.itemData)
        self.default_state = state

        self.childItems = []
        self.parentItem = parent

        self.setup_type()

        if self.parentItem is not None:
            self.parentItem.appendChild(self)

    def setup_type(self):
        if self.type == 'section':
            self.itemData[1:1] = ('<section>',)
            self.spec_default = self.data(1)

        elif self.type == 'list':
            self._setup_list(self.get_list_items())

    def _get_list_spec(self):
        data = self.data(1)
        comments = self.data(2)
        if comments:
            try:
                raw_spec = comments.split('\n')[-1].split()[1:]
                if raw_spec[0] == '__list__':  # and len(raw_spec[1:]) == len(data):
                    return raw_spec[1:]
            except IndexError:
                pass
        return list(map(str, range(len(data))))

    def get_list_items(self):
        spec = self._get_list_spec()
        values = self.data(1)
        if isinstance(self.spec_default, list):
            defaults = self.spec_default
        else:
            defaults = (None, )*len(spec)

        self.itemData[1] = '<list: {}>'.format(' '.join(spec))
        # self.spec_default = self.itemData[1]

        for key, value, default in zip(spec, values, defaults):
            yield ConfigModelItem((key, value, None, None), item_type='list_item',
                                  state=self.state, default=default)

    def _setup_list(self, items):  # use only at initialization
        for child in items:
            self.appendChild(child)

    @property
    def is_section(self):  # probably deprecated
        return self.type == 'section'

    def appendChild(self, item):
        self.childItems.append(item)
        item.parentItem = self

    def addChildren(self, items, row):
        if row == -1:
            row = 0
        self.childItems[row:row] = items

        for item in items:
            item.parentItem = self

    def child(self, row):
        return self.childItems[row]

    def childCount(self):
        return len(self.childItems)

    def columnCount(self):
        return len(self.itemData)

    def data(self, column):
        try:
            return self.itemData[column]
        except IndexError:
            return None

    def set_data(self, data, column):
        old_data = self.data(column)
        if old_data is None:
            data = convert_type(data)

        if data == '<list>':
            data = []

        try:
            self.itemData[column] = data
        except IndexError:
            return False

        if old_data != data:
            self.set_state('edited')
            self.check_state()

        return True

    def check_state(self):
        if self.spec_default is not None and self.data(1) == self.spec_default \
                and self.data(0) == self.default_values[0] and self.type != 'section':
            self.set_state('default')
            # print('def', self.data(1), self.data(0), self.spec_default)

        child_states = [child.state for child in self.childItems]
        if any(state in child_states for state in ['edited', 'added', 'deleted']):
            self.state = 'edited'
        if len(set(child_states)) == 1:  # if all states equal
            self.set_state(child_states[0], set_children=False)
            # print(child_states)

        if self.parentItem is not None:
            self.parentItem.check_state()

    def set_state(self, state, set_children=True):
        if self.state == 'unchanged' and state == 'default':
            return

        if self.state == 'added' and state in ('edited', 'unchanged', 'default', 'normal'):
            return

        self.state = state

        if set_children:  # to prevent cycle state set
            for child in self.childItems:
                child.set_state(state)

        # if state == 'edited':
        #    self.parentItem.state = state

    def set_type(self, item_type):
        self.type = item_type

    def parent(self):
        return self.parentItem

    def row(self):
        if self.parentItem is not None:
            return self.parentItem.childItems.index(self)
        return 0

    def removeChild(self, position):
        if position < 0 or position > len(self.childItems):
            return False
        child = self.childItems.pop(position)
        child.parentItem = None
        return True

    def __repr__(self):
        return str(self.itemData)


def ensure_unique_names(item, include_self=True):
    name = item.data(0)
    siblings_names = [child.data(0) for child in item.parent().childItems]
    if not include_self:
        siblings_names.remove(name)

    while name in siblings_names:
        if '_copy' in name:
            spl = name.split('_copy')
            num = int(spl[1]) if spl[1] else 0
            num += 1
            name = spl[0] + '_copy' + str(num)
        else:
            name = name + '_copy'

    item.set_data(name, 0)


class ConfigModel(QtCore.QAbstractItemModel):
    def __init__(self, parent=None, widget=None,
                 headers=("Option", "Value", 'Comment', 'Inline Comment')):
        self.rootItem = ConfigModelItem(headers)
        super(ConfigModel, self).__init__(parent)
        self.widget = widget

        self.do_color = True

        self.initial_comment = ''
        self.final_comment = ''

    @QtCore.pyqtSlot(int)
    def enable_color(self, value):
        self.do_color = value
        self.dataChanged.emit(QtCore.QModelIndex(), QtCore.QModelIndex(), (Qt.BackgroundRole, ))

    def headerData(self, section, orientation, role):
        if role == Qt.DisplayRole and orientation == Qt.Horizontal:
            return self.rootItem.data(section)

    def columnCount(self, parent):
        return self.rootItem.columnCount()

    def rowCount(self, parent):
        if parent.column() > 0:
            return 0

        if not parent.isValid():
            parentItem = self.rootItem
        else:
            parentItem = parent.internalPointer()

        return parentItem.childCount()

    def childrenIndexes(self, parent):
        column = parent.column()
        parent = self.index(parent.row(), 0, parent.parent())
        for i in range(self.rowCount(parent)):
            yield self.index(i, column, parent)

    def index(self, row, column, parent):
        if not self.hasIndex(row, column, parent):
            return QtCore.QModelIndex()

        parentItem = self.nodeFromIndex(parent)
        childItem = parentItem.child(row)

        if childItem:
            return self.createIndex(row, column, childItem)
        else:
            return QtCore.QModelIndex()

    def parent(self, index):
        if not index.isValid():
            return QtCore.QModelIndex()

        childItem = index.internalPointer()
        if not isinstance(childItem, ConfigModelItem):
            # print(childItem, index.column()), # index.row(), index.parent().internalPointer())
            return QtCore.QModelIndex()
        parentItem = childItem.parent()

        if parentItem == self.rootItem: #or parentItem is None:
            return QtCore.QModelIndex()

        return self.createIndex(parentItem.row(), 0, parentItem)

    def modifyCol(self, index, col):
        return self.index(index.row(), col, index.parent())

    def nodeFromIndex(self, index):
        if index.isValid():
            return index.internalPointer()
        return self.rootItem

    def data(self, index, role):
        if not index.isValid():
            return None

        item = index.internalPointer()

        if role == Qt.DisplayRole or role == Qt.EditRole:
            return item.data(index.column())

        if role == Qt.BackgroundRole and self.do_color:
            return QtGui.QBrush(states_colors[item.state])

        if role == StateRole:
            return item.state
        if role == TypeRole:
            return item.type

        return None

    def setData(self, index, value, role=Qt.EditRole):
        if not index.isValid():
            return False

        item = index.internalPointer()

        if role == Qt.EditRole:
            column = index.column()

            if column == 0 and value != item.data(column):
                if not self.widget.edit_caution():
                    return False

            item.set_data(value, column)

            if column == 0:
                ensure_unique_names(item, include_self=False)

            elif column == 1 and isinstance(item.data(1), (list, tuple)) \
                    and item.type not in ('list', 'list_item'):

                item.set_type('list')
                self.insertItems(0, list(item.get_list_items()), index)
                self.widget.ui.config_view.expandAll()

        elif role == StateRole:
            item.set_state(value)

        elif role == TypeRole:
            # if value != item.type and value == 'list':  # when list is created:
            #    pass
            item.set_type(value)

        self.dataChanged.emit(index, index, (role,))

        return True

    def flags(self, index):
        if not index.isValid():
            return QtCore.Qt.ItemIsDragEnabled | QtCore.Qt.ItemIsDropEnabled  # Qt.NoItemFlags
        item = index.internalPointer()

        flags = Qt.ItemIsEnabled | Qt.ItemIsSelectable

        if index.column() == 0:
            if item.type != 'list_item':
                flags |= int(Qt.ItemIsDragEnabled)

            if item.type == 'section':
                flags |= int(Qt.ItemIsDropEnabled)

        not_section = not (index.column() > 0 and item.type == 'section')
        not_list_item = not (index.column() > 1 and item.type == 'list_item')
        not_list_val = not (index.column() == 1 and item.type == 'list')

        if not_section and not_list_item and not_list_val:
            flags |= Qt.ItemIsEditable

        return flags

    def supportedDropActions(self):
        return QtCore.Qt.CopyAction | QtCore.Qt.MoveAction

    def mimeTypes(self):
        return ['app/configitem']

    def mimeData(self, indexes):
        mimedata = QtCore.QMimeData()
        index = indexes[0]
        mimedata.setData('app/configitem', pickle.dumps(self.nodeFromIndex(index)))
        return mimedata

    def dropMimeData(self, mimedata, action, row, column, parentIndex):
        if action == Qt.IgnoreAction:
            return True

        droppedNode = deepcopy(pickle.loads(mimedata.data('app/configitem')))

        self.insertItems(row, [droppedNode], parentIndex)
        self.dataChanged.emit(parentIndex, parentIndex)

        self.widget.ui.config_view.expandAll()

        if action & Qt.CopyAction:
            return False  # to not delete original item
        return True

    def removeRows(self, row, count, parent):
        self.beginRemoveRows(parent, row, row + count - 1)
        parentItem = self.nodeFromIndex(parent)
        for _ in range(count):
            parentItem.removeChild(row)

        self.endRemoveRows()
        return True

    def removeRow(self, index):
        parent = index.parent()
        self.beginRemoveRows(parent, index.row(), index.row())

        parentItem = self.nodeFromIndex(parent)
        parentItem.removeChild(index.row())

        self.endRemoveRows()
        return True

    def insertItems(self, row, items, parentIndex):
        parent = self.nodeFromIndex(parentIndex)
        self.beginInsertRows(parentIndex, row, row + len(items) - 1)  # parentIndex or QtCore.QModelIndex()

        parent.addChildren(items, row)

        self.endInsertRows()
        self.update_all()

        return True

    def update_all(self):
        self.dataChanged.emit(QtCore.QModelIndex(), QtCore.QModelIndex())

    def dict_setup(self, data: dict, parent=None, convert_types=False):
        if parent is None:
            parent = self.rootItem

        for key, value in data.items():
            if isinstance(value, dict):
                item = ConfigModelItem((key,), parent=parent, item_type='section')
                self.dict_setup(value, parent=item)
            else:
                if convert_types:
                    value = convert_type(value)
                parent.appendChild(ConfigModelItem((key, value, '', '')))

    def config_dict_setup(self, data: dict, convert_types=False, parent=None):
        if parent is None:
            parent = self.rootItem

            self.initial_comment = '\n'.join(data.pop('initial_comment', ['']))
            self.final_comment = '\n'.join(data.pop('final_comment', ['']))

        for key, item in data.items():
            if '__value__' in item:
                value = item.get('__value__')
                if convert_types:
                    value = convert_type(value)

                default = item['default']
                comments = '\n'.join(item.get('comments', '')) or ''
                inline_comment = item.get('inline_comment', '') or ''

                if item['unchanged']:
                    state = 'unchanged'
                elif value == default:
                    state = 'default'
                else:
                    state = 'normal'

                parent.appendChild(ConfigModelItem((key, value, comments, inline_comment),
                                                   state=state, default=default))

            else:
                section = ConfigModelItem((key,), parent=parent, item_type='section')
                self.config_dict_setup(item, convert_types=convert_types, parent=section)
                section.check_state()

    def to_dict(self, parent=None) -> dict:
        if parent is None:
            parent = self.rootItem

        data = {}
        for item in parent.childItems:
            item_name, item_data = item.data(0), item.data(1)
            if item.is_section:
                data[item_name] = self.to_dict(item)
            else:
                data[item_name] = item_data

        return data

    def to_config_dict(self, parent=None) -> dict:
        data = {}

        if parent is None:
            parent = self.rootItem
            data['initial_comment'] = self.initial_comment.split('\n')
            data['final_comment'] = self.final_comment.split('\n')

        for item in parent.childItems:
            key = item.data(0)

            if item.is_section:
                d = self.to_config_dict(item)
                if d:  # to prevent empty sections
                    data[key] = d

            elif item.state not in ('unchanged', 'deleted'):
                if item.type == 'list':
                    value = [child.data(1) for child in item.childItems]
                else:
                    value = item.data(1)

                d = {'__value__': value}

                comment = item.data(2)
                if comment:
                    d.update({'comments': comment.split('\n')})

                inline_comment = item.data(3)
                if inline_comment:
                    d.update({'inline_comment': inline_comment})

                data[key] = d

        return data

    @property
    def dict(self):
        return self.to_dict()


class ConfigTreeWidget(QTreeView):
    def __init__(self):
        QTreeView.__init__(self)

        self.setContextMenuPolicy(Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self.open_menu)

        self.setSelectionMode(self.SingleSelection)
        # self.setSelectionBehavior(self.SelectItems)

        self.setDragDropMode(QAbstractItemView.DragDrop)
        self.setDefaultDropAction(Qt.MoveAction)
        self.setDragEnabled(True)
        self.setAcceptDrops(True)
        self.setDropIndicatorShown(True)
        self.setAnimated(True)

        self.duplicate_shortcut = QShortcut(QKeySequence('Shift+D'), self)
        self.duplicate_shortcut.activated.connect(self.with_selected(self.duplicate))
        self.exclude_shortcut = QShortcut(QKeySequence('Alt+Del'), self)
        self.exclude_shortcut.activated.connect(self.with_selected(self.exclude))
        self.remove_shortcut = QShortcut(QKeySequence('Del'), self)
        self.remove_shortcut.activated.connect(self.with_selected(self.remove))
        self.clear_shortcut = QShortcut(QKeySequence('Shift+R'), self)
        self.clear_shortcut.activated.connect(self.with_selected(self.reset_item, 'clear_value'))
        self.default_shortcut = QShortcut(QKeySequence('Ctrl+R'), self)
        self.default_shortcut.activated.connect(self.with_selected(self.reset_item, 'default'))
        self.reset_shortcut = QShortcut(QKeySequence('Alt+R'), self)
        self.reset_shortcut.activated.connect(self.with_selected(self.reset_item, 'all'))
        self.item_shortcut = QShortcut(QKeySequence('Shift+A'), self)
        self.item_shortcut.activated.connect(self.with_selected(self.add_item, False))
        self.section_shortcut = QShortcut(QKeySequence('Ctrl+A'), self)
        self.section_shortcut.activated.connect(self.with_selected(self.add_item, True))

    def with_selected(self, f, *args, **kwargs):
        def decorated():
            index = self.selectedIndexes()[0]
            return f(index, *args, **kwargs)

        return decorated

    def open_menu(self, point):
        index = self.indexAt(point)
        item = index.internalPointer()

        menu = QMenu()

        duplicate = QAction("Duplicate")
        duplicate.setShortcut(self.duplicate_shortcut.key())
        duplicate.triggered.connect(partial(self.duplicate, index))
        menu.addAction(duplicate)

        exclude = QAction("Toggle exclude")
        exclude.setShortcut(self.exclude_shortcut.key())
        exclude.triggered.connect(partial(self.exclude, index))
        menu.addAction(exclude)

        remove = QAction("Remove from config")
        remove.setShortcut(self.remove_shortcut.key())
        remove.triggered.connect(partial(self.remove, index))
        menu.addAction(remove)

        menu.addSeparator()

        clear = QAction("Clear item value")
        clear.setShortcut(self.clear_shortcut.key())
        clear.triggered.connect(partial(self.reset_item, index, 'clear_value'))
        menu.addAction(clear)

        reset_default = QAction("Reset value to default")
        reset_default.setShortcut(self.default_shortcut.key())
        reset_default.triggered.connect(partial(self.reset_item, index, 'default'))
        menu.addAction(reset_default)

        reset_all = QAction("Reset all changes")
        reset_all.setShortcut(self.reset_shortcut.key())
        reset_all.triggered.connect(partial(self.reset_item, index, 'all'))
        menu.addAction(reset_all)

        menu.addSeparator()

        add_option = QAction("Add option")
        add_option.setShortcut(self.item_shortcut.key())
        add_option.triggered.connect(partial(self.add_item, index, False))
        menu.addAction(add_option)

        add_section = QAction("Add section")
        add_section.setShortcut(self.section_shortcut.key())
        add_section.triggered.connect(partial(self.add_item, index, True))
        menu.addAction(add_section)

        if item is None:
            clear.setDisabled(True)
            reset_all.setDisabled(True)
            reset_default.setDisabled(True)

            duplicate.setDisabled(True)
            remove.setDisabled(True)
            exclude.setDisabled(True)
        else:
            if item.type in ('list', 'list_item'):
                add_section.setDisabled(True)

            if item.type == 'list':
                clear.setDisabled(True)  # Temporary, cuz buggg

            # if item.type == 'section':
            #     clear.setDisabled(True)

        menu.exec_(QCursor.pos())

    def duplicate(self, index):
        item = deepcopy(index.internalPointer())
        item.set_state('added')
        ensure_unique_names(item)
        self.model().insertItems(index.row() + 1, [item], index.parent())
        self.expandAll()  # fixes not expanded duplicated section

    def remove(self, index):
        self.model().removeRow(index)

    def exclude(self, index):
        item = self.model().nodeFromIndex(index)
        if item.state == 'deleted':
            self.model().setData(index, item.previous_state, StateRole)
        else:
            self.model().setData(index, 'deleted', StateRole)

    def add_item(self, index, is_section):
        parentItem = self.model().nodeFromIndex(index)

        if parentItem.type in ('list', 'list_item'):
            if is_section:
                return
            item_type = 'list_item'
        else:
            item_type = 'section' if is_section else 'option'

        prompt = 'Enter {} name'.format(item_type.replace('_', ' '))
        text, ok = QInputDialog.getText(self, prompt, prompt)
        if not ok:
            return

        if parentItem.type in ('list', 'section'):  # to append at first index in section or list
            row = 0
            parent = index
        else:
            row = index.row()
            parent = index.parent()
            if row == -1:  # to append at last position e.g. at root
                row = parentItem.childCount()
            else:
                row += 1  # to append under current position

        item = ConfigModelItem((text, None, '', ''), item_type=item_type, state='added')
        self.model().insertItems(row, [item], parent)

        ensure_unique_names(item, include_self=False)
        # parent.internalPointer().set_state('edited')
        self.expandAll()

    def reset_item(self, index, reset_type):
        item = index.internalPointer()
        model = self.model()
        itemdataindex = model.modifyCol(index, 1)

        if reset_type == 'all':
            for i, default in enumerate(item.default_values):
                model.setData(model.modifyCol(index, i), default)

            model.setData(index, item.default_state, role=StateRole)

        elif reset_type == 'default':
            # if item.type == 'list' and \
            #         not isinstance(item.spec_default, (list, tuple)):
            #     self.reset_item(item, 'clear_value')

            model.setData(itemdataindex, item.spec_default)

            if item.default_state == 'unchanged':
                model.setData(index, 'unchanged', role=StateRole)

        elif reset_type == 'clear_value':
            item_type = model.data(itemdataindex, TypeRole)
            if item_type == 'list':
                return
            if item_type != 'section':
                model.setData(itemdataindex, None)

            # if model.data(itemdataindex, TypeRole) == 'list':  # TODO
            #     model.removeRows(0, item.childCount(), index)
            #     model.setData(index, 'option', role=TypeRole)
            #     return

        for child in model.childrenIndexes(index):
            self.reset_item(child, reset_type)


class ConfigDialog(QtWidgets.QDialog):
    copter_editor_signal = QtCore.pyqtSignal(object, object)

    def __init__(self, parent=None):
        super(ConfigDialog, self).__init__(parent)
        self.ui = config_editor.Ui_config_dialog()
        self.model = ConfigModel(widget=self)
        self._filename = None
        self.unsaved = False
        self.setupUi()
        self.copter_editor_signal.connect(self._call_copter_dialog)

    @property
    def filename(self):
        return self._filename or 'Untitled.ini'

    def setupModel(self, data, pure_dict=False, convert_types=False):
        if pure_dict:
            self.model.dict_setup(data, convert_types=convert_types)
        else:
            self.model.config_dict_setup(data, convert_types=convert_types)

        self.ui.config_view.expandAll()
        self.ui.config_view.resizeColumnToContents(0)
        self.ui.config_view.resizeColumnToContents(1)

        self.model.dataChanged.connect(self.unsaved_call)  # connect after setup

    def setupUi(self):
        self.ui.setupUi(self)

        self.ui.config_view = ConfigTreeWidget()
        self.ui.config_view.setObjectName("config_view")
        self.ui.config_view.setModel(self.model)
        self.ui.gridLayout.addWidget(self.ui.config_view, 0, 0, 1, 1)
        self.ui.config_view.expandAll()

        self.ui.do_coloring.stateChanged.connect(self.model.enable_color)
        self.ui.save_as_button.clicked.connect(self.save_as)

        # self.ui.delete_button.pressed.connect(self.remove_selected)

    def update_title(self):
        self.setWindowTitle(f"Config editor - {self.filename}" + "*"*self.unsaved)

    def unsaved_call(self):
        self.unsaved = True
        self.update_title()
        self.model.dataChanged.disconnect(self.unsaved_call)

    def closeEvent(self, event):
        if not self.unsaved or self.result():
            event.accept()
            return

        reply = QMessageBox.question(self, "Confirm exit", "There are unsaved changes in config file. "
                                                           "Are you sure you want to exit?",
                                     QMessageBox.No | QMessageBox.Yes, QMessageBox.No)

        if reply != QMessageBox.Yes:
            event.ignore()
        else:
            event.accept()

    def edit_caution(self):
        reply = QMessageBox().warning(self, "Editing caution",
                                      "Are you sure you want to edit section/option name? "
                                      "Proceed with caution!",
                                      QMessageBox.Yes | QMessageBox.No, QMessageBox.No
                                      )
        return reply == QMessageBox.Yes

    def save_as(self):
        save_path = QFileDialog.getSaveFileName(self, "Save as configuration file (.ini)",
                                                directory=self.filename+'.ini',
                                                options=QFileDialog.DontConfirmOverwrite,
                                                filter="Config files (*.ini);;All files (*.*)")[0]
        if not save_path:
            return

        split_path = save_path.split('.')

        if not (len(split_path) > 1 and split_path[-1] == 'ini'):
            save_path += '.ini'

        cfg = config.ConfigManager()
        cfg.load_from_dict(self.model.to_config_dict())
        cfg.config.filename = save_path
        cfg.write()

    @pyqtSlot()
    def run(self):
        self.show()
        self.exec()
        return self.result()

    def validation_loop(self, cfg, configspec=None):  # modifies cfg object
        filename = cfg.config.filename
        while True:
            if not self.run():
                return False

            try:
                cfg.load_from_dict(self.model.to_config_dict(), configspec=configspec)
            except config.ValidationError as error:
                msg = "Can not validate. Proceed with editing? Errors: \n" + "\n".join(error.flatten_errors())
                reply = QMessageBox.warning(self, "Validation error!", msg, QMessageBox.Yes | QMessageBox.Cancel)

                if reply == QMessageBox.Cancel:
                    return False

            else:
                return True

            finally:
                if filename is not None:
                    cfg.config.filename = filename

    def call_copter_dialog(self, client, value):
        self.copter_editor_signal.emit(client, value)

    @pyqtSlot(object, object)
    def _call_copter_dialog(self, client, value):
        logging.info("Opening copter config dialog")
        config_dict, spec_dict = value["config"], value["configspec"]
        cfg = config.ConfigManager()
        cfg.load_from_dict(config_dict, spec_dict)

        def save_callback():
            edited_dict = cfg.full_dict(include_defaults=False)
            client.send_message("config", kwargs={"config": edited_dict, "mode": "rewrite"})

        def restart_callback():
            client.send_message("service_restart", kwargs={"name": "clever-show"})

        if not self.call_config_dialog(cfg, save_callback, restart_callback, f"{client.copter_id}"):
            return False
        return True

    def call_config_dialog(self, cfg: config.ConfigManager, on_save=None, on_restart=None, name="Untitled.ini"):
        self.setupModel(cfg.full_dict(include_defaults=True), convert_types=(not cfg.validated))
        self.ui.do_restart.setEnabled(on_restart is not None)
        self._filename = name
        self.update_title()

        if not self.validation_loop(cfg, cfg.config.configspec):
            return False

        if on_save is not None:
            on_save()

        if on_restart is not None and self.ui.do_restart.isChecked():
            on_restart()
        return True

    @classmethod
    def call_standalone_dialog(cls):
        dialog = cls()
        dialog._call_standalone_dialog()

    def _call_standalone_dialog(self):
        path = QFileDialog.getOpenFileName(self, "Select configuration or specification file",
                                           filter="Config and spec files (*.ini)")[0]
        if not path:
            return False

        cfg = config.ConfigManager()
        try:
            cfg.load_from_file(path)
        except ValueError as error:  # When file do not exist or not validated properly
            QMessageBox.warning(self, "Error while opening file!",
                                "Config cannot be opened or validated: {}".format(error))
            return False

        def save_callback():
            if cfg.config.filename is None:
                save_path = QFileDialog.getSaveFileName(self, "Save configuration file",
                                                        directory=self.filename,
                                                        filter="Config files (*.ini)")[0]
                if not save_path:
                    return False
            else:
                save_path = cfg.config.filename

            cfg.config.filename = save_path
            cfg.write()

        if cfg.config.filename is not None:
            name = os.path.split(cfg.config.filename)[1]
        else:  # when editing only configspec-based file
            name = os.path.split(path)[1]

        if not self.call_config_dialog(cfg, on_save=save_callback, name=name):
            return False
        return True


if __name__ == '__main__':
    def except_hook(cls, exception, traceback):
        print(cls, exception, traceback)
        sys.__excepthook__(cls, exception, traceback)

    sys.excepthook = except_hook

    app = QtWidgets.QApplication(sys.argv)

    ui = ConfigDialog()
    ui.call_standalone_dialog()
    # d = {'section': {'opt': 1, "opt222": 'text'}}
    # ui.setupModel(d, pure_dict=True)
    # ui.show()
    # app.exec()
    # print(ui.model.to_config_dict())
