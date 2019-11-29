from copy import deepcopy

import pickle

import config_editor
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt as Qt
from PyQt5.QtGui import QCursor, QStandardItemModel
from PyQt5.QtWidgets import QAbstractItemView, QTreeView, QMenu


class ConfigModelItem:
    def __init__(self, label, value="", is_section=False, state='default', parent=None):
        self.itemData = [label, value]
        self.is_section = is_section
        self.state = state

        self.childItems = []
        self.parentItem = parent

        if self.parentItem is not None:
            self.parentItem.appendChild(self)

    def appendChild(self, item):
        self.childItems.append(item)
        item.parentItem = self

    def addChildren(self, items, row):
        if row == -1:
            self.childItems.extend(items)
        else:
            #row -= 1
            print('row', row)
            self.childItems[row:row] = items

        print(self.childItems)

        for item in items:
            item.parentItem = self

    def child(self, row):
        return self.childItems[row]

    def childCount(self):
        return len(self.childItems)

    def columnCount(self):
        return 2

    def data(self, column):
        try:
            return self.itemData[column]
        except IndexError:
            return None

    def set_data(self, data, column):
        try:
            self.itemData[column] = data
        except IndexError:
            return False

        return True

    def parent(self):
        return self.parentItem

    def row(self):
        if self.parentItem is not None:
            return self.parentItem.childItems.index(self)

        return 0

    def removeChild(self, position):
        if position < 0 or position > len(self.childItems):
            return False
        print('removing', position)
        child = self.childItems.pop(position)
        child.parentItem = None
        return True

    def removeChildren(self, row, count):
        print(range(row, row+count))
        for pos in range(row, row+count):
            self.removeChild(pos)

        return True

    def __repr__(self):
        return str(self.itemData)


class ConfigModel(QtCore.QAbstractItemModel):
    def __init__(self, data, parent=None):
        super(ConfigModel, self).__init__(parent)

        self.rootItem = ConfigModelItem("Option", "Value")
        self.setup(data)

    def headerData(self, section, orientation, role):
        if role == Qt.DisplayRole and orientation == Qt.Horizontal:
            return self.rootItem.data(section)

    def columnCount(self, parent):
        return 2

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

        return None

    @QtCore.pyqtSlot()
    def setData(self, index, value, role=Qt.EditRole):
        if not index.isValid():
            return False

        item = index.internalPointer()
        if role == Qt.EditRole:
            item.set_data(value, index.column())

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

        if index.column() == 1 and not item.is_section:
            flags |= Qt.ItemIsEditable

        return flags

    def supportedDropActions(self):
        return QtCore.Qt.CopyAction | QtCore.Qt.MoveAction

    def mimeTypes(self):
        return ['bstream', 'text/xml']

    def mimeData(self, indexes):
        mimedata = QtCore.QMimeData()
        index = indexes[0]
        mimedata.setData('bstream', pickle.dumps(self.nodeFromIndex(index)))
        return mimedata

    def dropMimeData(self, mimedata, action, row, column, parentIndex):
        print(action)
        print('mim', row)
        if action == Qt.IgnoreAction:
            return True

        parentNode = self.nodeFromIndex(parentIndex)
        droppedNode = deepcopy(pickle.loads(mimedata.data('bstream')))
        print(droppedNode.itemData, 'node')
        #droppedNode = pickle.loads(mimedata.data('bstream')) #
        #self.removeRow()#self.index(row, column, parentIndex))#parentNode.child(row))
        self.insertItems(row, [droppedNode], parentIndex)
        self.dataChanged.emit(parentIndex, parentIndex)

        return True

    def removeRows1(self, row, count, parent):
        print('rem', row, count)
        self.beginRemoveRows(parent, row, row+count-1)
        parentItem = self.nodeFromIndex(parent)
        print(parentItem, parentItem.itemData)

        #parentItem.removeChild(row)
        parentItem.removeChildren(row, count)
        print(parentItem.childItems)

        self.endRemoveRows()
        print('removed')
        return True

    @QtCore.pyqtSlot()
    def removeRow(self, index):
        parent = index.parent()
        self.beginRemoveRows(parent, index.row(), index.row())

        parentItem = self.nodeFromIndex(parent)
        parentItem.removeChild(index.row())

        self.endRemoveRows()
        return True

    def insertItems(self, row, items, parentIndex):
        print('ins', row)
        parent = self.nodeFromIndex(parentIndex)
        self.beginInsertRows(parentIndex, row, row+len(items)-1)

        parent.addChildren(items, row)
        print(parent.childItems)

        self.endInsertRows()
        self.dataChanged.emit(parentIndex, parentIndex)
        return True

    def setup(self, data: dict, parent=None):
        if parent is None:
            parent = self.rootItem

        for key, value in data.items():
            if isinstance(value, dict):
                item = ConfigModelItem(key, parent=parent, is_section=True)
                self.setup(value, parent=item)
            else:
                parent.appendChild(ConfigModelItem(key, value))

    def to_dict(self, parent=None) -> dict:
        if parent is None:
            parent = self.rootItem

        data = {}
        for item in parent.childItems:
            item_name, item_data = item.itemData
            if item.childItems:
                data[item_name] = self.to_dict(item)
            else:
                data[item_name] = item_data

        return data

    @property
    def dict(self):
        return self.to_dict()


class ConfigDialog(config_editor.Ui_config_dialog):
    def __init__(self, data):
        super(ConfigDialog, self).__init__()
        self.model = ConfigModel(data)

    def setupUi(self, config_dialog):
        super(ConfigDialog, self).setupUi(config_dialog)

        #self.config_view = Tree()

        self.config_view = Tree()
        self.config_view.setObjectName("config_view")
        self.config_view.setModel(self.model)
        self.gridLayout.addWidget(self.config_view, 0, 0, 1, 1)

        self.config_view.expandAll()
        #self.config_view.setDragDropMode(True)
        #self.setDragDropMode(QAbstractItemView.InternalMove)
        #self.setDragEnabled(True)
        #self.setAcceptDrops(True)
        #self.setDropIndicatorShown(True)

        self.delete_button.pressed.connect(self.remove_selected)

    def remove_selected(self):
        index = self.config_view.selectedIndexes()[0]
        self.model.removeRow(index)\


class Tree(QTreeView):
    def __init__(self):
        QTreeView.__init__(self)
        data = {"section 1": {"opt1": "str", "opt2": 123, "opt3": 1.23, "opt4": False, "...": {'subopt': 'bal'}},
                "section 2": {"opt1": "str", "opt2": [1.1, 2.3, 34], "opt3": 1.23, "opt4": False, "...": ""}}
        #model = ConfigModel(data)

        #self.setModel(model)
        self.setContextMenuPolicy(Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self.open_menu)

        self.setSelectionMode(self.SingleSelection)
        self.setDragDropMode(QAbstractItemView.InternalMove)
        self.setDragEnabled(True)
        self.setAcceptDrops(True)
        self.setDropIndicatorShown(True)

    # def dropEvent(self, e):
    #     print(e.dropAction()==QtCore.Qt.MoveAction)
    #     if e.keyboardModifiers() & QtCore.Qt.AltModifier:
    #         e.setDropAction()
    #         print('copy')
    #     else:
    #         e.setDropAction(QtCore.Qt.MoveAction)
    #         print("drop")
    #     print(e)
    #     e.accept()

    def open_menu(self):
        menu = QMenu()
        menu.addAction("Create new folder")
        menu.exec_(QCursor.pos())

if __name__ == '__main__':
    import sys


    def except_hook(cls, exception, traceback):
        sys.__excepthook__(cls, exception, traceback)

    sys.excepthook = except_hook

    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()

    data = {"section 1": {"opt1": "str", "opt2": 123, "opt3": 1.23, "opt4": False, }}#"...": {'subopt': 'bal'}},}
            #"section 2": {"opt1": "str", "opt2": [1.1, 2.3, 34], "opt3": 1.23, "opt4": False, "...": ""}}

    ui = ConfigDialog(data)
    ui.setupUi(Dialog)

    print(Qt.DisplayRole)
    Dialog.show()
    print(app.exec_())

    print(Dialog.result())
    print(ui.model.to_dict())

    sys.exit()
