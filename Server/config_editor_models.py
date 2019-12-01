import pickle
from ast import literal_eval
from functools import partial
from copy import deepcopy

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt as Qt
from PyQt5.QtGui import QCursor, QStandardItemModel
from PyQt5.QtWidgets import QAbstractItemView, QTreeView, QMenu, QAction, QMessageBox, QInputDialog

import config_editor


def dict_walk(d: dict, keys):
    current = d
    for key in keys:
        try:
            current = current[key]
        except KeyError:
            return None
    return current


states_colors = {
    'normal': Qt.white,
    'unchanged': Qt.darkGray,
    'default': Qt.lightGray,
    'edited': Qt.yellow,
    'added': Qt.green,
    'deleted': Qt.red,
}


class ConfigModelItem:
    def __init__(self, values=(), is_section=False, state='normal', default=None, parent=None):
        self.spec_default = default

        values = list(values)
        if is_section:
            values[1:1] = ('<section>',)
            self.spec_default = values[1]

        self.itemData = values
        self.type = 'section' if is_section else None
        self.state = state

        self.default_values = deepcopy(self.itemData)
        self.default_state = state

        self.childItems = []
        self.parentItem = parent

        if self.parentItem is not None:
            self.parentItem.appendChild(self)

    @property
    def is_section(self):
        return self.type == 'section'

    def reset(self):
        self.set_data(self.spec_default, 1)

        self.check_state()
        if self.default_state == 'unchanged':
            self.set_state('unchanged')

        for child in self.childItems:
            child.reset()

    def reset_all(self):
        self.itemData = self.default_values
        self.set_state(self.default_state)

        for child in self.childItems:
            child.reset()

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
            data = literal_eval(data) if data else None

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
                and self.data(0) == self.default_values[0]:
            self.set_state('default')

    def set_state(self, state):
        # if self.state == 'unchanged' and state == 'default':
        #     return
        if self.state == 'added' and state in ('edited', 'unchanged', 'default', 'normal'):
            return

        self.state = state
        for child in self.childItems:
            child.set_state(state)

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
        super(ConfigModel, self).__init__(parent)
        self.widget = widget

        self.rootItem = ConfigModelItem(headers)
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
        parentItem = childItem.parent()

        if parentItem == self.rootItem or parentItem is None:
            return QtCore.QModelIndex()

        return self.createIndex(parentItem.row(), 0, parentItem)

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

        return None

    def setData(self, index, value, role=Qt.EditRole):
        if not index.isValid():
            return False

        item = index.internalPointer()
        if role == Qt.EditRole:
            if index.column() == 0 and (self.widget is not None) \
                    and value != item.data(index.column()):
                if not self.widget.edit_caution():
                    return False

            item.set_data(value, index.column())
            if index.column() == 0:
                ensure_unique_names(item, include_self=False)

        self.dataChanged.emit(index, index, (role,))

        return True

    def flags(self, index):
        if not index.isValid():
            return QtCore.Qt.ItemIsDragEnabled | QtCore.Qt.ItemIsDropEnabled  # Qt.NoItemFlags
        item = index.internalPointer()

        flags = Qt.ItemIsEnabled | Qt.ItemIsSelectable

        if index.column() == 0:
            flags |= int(QtCore.Qt.ItemIsDragEnabled)
            if item.is_section:
                flags |= int(QtCore.Qt.ItemIsDropEnabled)

        if not (index.column() > 0 and item.is_section):
            flags |= Qt.ItemIsEditable

        return flags

    def supportedDropActions(self):
        return QtCore.Qt.CopyAction | QtCore.Qt.MoveAction

    def mimeTypes(self):
        return ['app/configitem', 'text/xml']

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

        for x in range(count):
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
        self.beginInsertRows(parentIndex, row, row + len(items) - 1)

        parent.addChildren(items, row)

        self.endInsertRows()
        self.dataChanged.emit(parentIndex, parentIndex)
        return True

    def get_key_sequence(self, index):
        item = index.internalPointer()
        keys = []
        while item is not None:
            key = item.data(0)
            keys.append(key)
            item = item.parent()
        return list(reversed(keys[:-1]))

    def dict_setup(self, data: dict, parent=None):
        if parent is None:
            parent = self.rootItem

        for key, value in data.items():
            if isinstance(value, dict):
                item = ConfigModelItem((key,), parent=parent, is_section=True)
                self.dict_setup(value, parent=item)
            else:
                parent.appendChild(ConfigModelItem((key, value)))

    def config_dict_setup(self, data: dict, parent=None):
        if parent is None:
            parent = self.rootItem

            self.initial_comment = '\n'.join(data.pop('initial_comment', ['']))
            self.final_comment = '\n'.join(data.pop('final_comment', ['']))

        for key, item in data.items():
            if item.get('__option__', False):
                # {'__option__': True, 'value': 'Copter config', 'default': 'Copter config', 'unchanged': False, 'comments': [], 'inline_comment': None}
                value = item['value']
                default = item['default']
                comments = '\n'.join(item['comments']) or ''
                inline_comment = item['inline_comment'] or ''

                if item['unchanged']:
                    state = 'unchanged'
                elif value == default:
                    state = 'default'
                else:
                    state = 'normal'

                parent.appendChild(ConfigModelItem((key, value, comments, inline_comment),
                                                   state=state, default=default))

            else:
                section = ConfigModelItem((key,), parent=parent, is_section=True)
                self.config_dict_setup(item, parent=section)

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

            elif item.state != 'unchanged':
                d = {'__option__': True,
                     'value': item.data(1),
                     # 'default': item.default,
                     # 'unchanged': False,
                     'comments': (item.data(2) or '').split('\n'),
                     'inline_comment': item.data(3) or ''
                     }

                data[key] = d

        return data

    @property
    def dict(self):
        return self.to_dict()


class ConfigDialog(QtWidgets.QDialog):
    def __init__(self):
        super(ConfigDialog, self).__init__()
        self.ui = config_editor.Ui_config_dialog()
        self.model = ConfigModel(widget=self)
        self.setupUi()

    def setupModel(self, data, pure_dict=False):
        if pure_dict:
            self.model.dict_setup(data)
        else:
            self.model.config_dict_setup(data)

        self.ui.config_view.expandAll()

    def setupUi(self):
        self.ui.setupUi(self)

        self.ui.config_view = Tree()
        self.ui.config_view.setObjectName("config_view")
        self.ui.config_view.setModel(self.model)
        self.ui.gridLayout.addWidget(self.ui.config_view, 0, 0, 1, 1)
        self.ui.config_view.expandAll()

        self.ui.do_coloring.stateChanged.connect(self.model.enable_color)

        # self.ui.delete_button.pressed.connect(self.remove_selected)

        # index = self.config_view.selectedIndexes()[0]

    def edit_caution(self):
        reply = QMessageBox().warning(self, "Editing caution",
                                      "Are you sure you want to edit section/option name? "
                                      "Proceed with caution!",
                                      QMessageBox.Yes | QMessageBox.No, QMessageBox.No
                                      )
        return reply == QMessageBox.Yes


class Tree(QTreeView):
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

        # self.header()
        # header.setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        # header.setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        # self.resizeColumnToContents(1)
        # header.setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)

        self.setAnimated(True)

    def open_menu(self, point):
        index = self.indexAt(point)
        item = index.internalPointer()

        menu = QMenu()

        duplicate = QAction("Duplicate")
        duplicate.triggered.connect(partial(self.duplicate, index))
        menu.addAction(duplicate)

        remove = QAction("Remove from config")
        remove.triggered.connect(partial(self.remove, index))
        menu.addAction(remove)

        menu.addSeparator()

        reset = QAction("Reset value to default")
        reset.triggered.connect(partial(self.reset_item, index, False))
        menu.addAction(reset)

        reset_all = QAction("Reset all data")
        reset_all.triggered.connect(partial(self.reset_item, index, True))
        menu.addAction(reset_all)

        menu.addSeparator()

        add_option = QAction("Add option")
        add_option.triggered.connect(partial(self.add_item, index, False))
        menu.addAction(add_option)

        add_section = QAction("Add section")
        add_section.triggered.connect(partial(self.add_item, index, True))
        menu.addAction(add_section)

        if item is None:
            reset.setDisabled(True)
            reset_all.setDisabled(True)
            duplicate.setDisabled(True)
            remove.setDisabled(True)

        menu.exec_(QCursor.pos())

    def duplicate(self, index):
        item = deepcopy(index.internalPointer())
        item.set_state('added')
        ensure_unique_names(item)
        self.model().insertItems(index.row() + 1, [item], index.parent())
        self.expandAll()  # fix not expanded duplicated section

    def remove(self, index):
        self.model().removeRow(index)

    def add_item(self, index, is_section):
        prompt = 'Enter {} name'.format('section' if is_section else 'option')
        text, ok = QInputDialog.getText(self, prompt, prompt)
        if not ok:
            return

        item = ConfigModelItem((text, None, '', ''), is_section=is_section, state='added')
        row = index.row()
        if row == -1:  # to append at last position
            parentItem = self.model().nodeFromIndex(index)
            row = parentItem.childCount() - 1

        self.model().insertItems(row + 1, [item], index.parent())
        ensure_unique_names(item, include_self=False)

    def reset_item(self, index, reset_all):
        item = index.internalPointer()
        if reset_all:
            item.reset_all()
        else:
            item.reset()


if __name__ == '__main__':
    import sys


    def except_hook(cls, exception, traceback):
        sys.__excepthook__(cls, exception, traceback)


    sys.excepthook = except_hook

    app = QtWidgets.QApplication(sys.argv)

    # data = {"section 1": {"opt1": "str", "opt2": 123, "opt3": 1.23, "opt4": False, "...": {'subopt': 'bal'}},
    #         "section 2": {"opt1": "str", "opt2": [1.1, 2.3, 34], "opt3": 1.23, "opt4": False, "...": ""}}
    data = {
        'config_name': {'__option__': True, 'value': 'Copter config', 'default': 'Copter config', 'unchanged': False,
                        'comments': [], 'inline_comment': None},
        'config_version': {'__option__': True, 'value': 0.0, 'default': 0.0, 'unchanged': False, 'comments': [],
                           'inline_comment': None}, 'SERVER': {
            'port': {'__option__': True, 'value': 25000, 'default': 25000, 'unchanged': False, 'comments': [],
                     'inline_comment': None},
            'host': {'__option__': True, 'value': '192.168.1.103', 'default': '192.168.1.101', 'unchanged': False,
                     'comments': [], 'inline_comment': None},
            'buffer_size': {'__option__': True, 'value': 1024, 'default': 1024, 'unchanged': False, 'comments': [],
                            'inline_comment': None}}, 'BROADCAST': {
            'use': {'__option__': True, 'value': True, 'default': True, 'unchanged': False, 'comments': [],
                    'inline_comment': None},
            'port': {'__option__': True, 'value': 8181, 'default': 8181, 'unchanged': False, 'comments': [],
                     'inline_comment': None}}, 'NTP': {
            'use': {'__option__': True, 'value': False, 'default': False, 'unchanged': False, 'comments': [],
                    'inline_comment': None},
            'port': {'__option__': True, 'value': 123, 'default': 123, 'unchanged': False,
                     'comments': ['#host = ntp1.stratum2.ru'], 'inline_comment': None},
            'host': {'__option__': True, 'value': 'ntp1.stratum2.ru', 'default': 'ntp1.stratum2.ru', 'unchanged': True,
                     'comments': [], 'inline_comment': ''}}, 'PRIVATE': {
            'id': {'__option__': True, 'value': '/hostname', 'default': '/hostname', 'unchanged': True,
                   'comments': ['# avialiable options: /hostname ; /default ; /ip ; any string 63 characters lengh', 'newlibe'],
                   'inline_comment': None}},
        'initial_comment': ['# This is generated config_attrs with default_values', '# Modify to configure'],
        'final_comment': []}

    ui = ConfigDialog()
    ui.setupModel(data)
    ui.show()

    print(app.exec_())

    print(ui.result())
    print(ui.model.to_dict())
    print(ui.model.to_config_dict())

    sys.exit()
