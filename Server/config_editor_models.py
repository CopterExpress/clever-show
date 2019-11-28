import config_editor
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt as Qt


class ConfigModelItem:
    def __init__(self, label, value="", is_section=False, parent=None):
        self.itemData = [label, value]

        self.is_section=is_section
        self.childItems = []
        self.parentItem = parent

        if self.parentItem is not None:
            self.parentItem.appendChild(self)

    def appendChild(self, item):
        self.childItems.append(item)
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

        child = self.childItems.pop(position)
        child.parentItem = None
        return True


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

        if not parent.isValid():
            parentItem = self.rootItem
        else:
            parentItem = parent.internalPointer()

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
            return Qt.NoItemFlags
        item = index.internalPointer()

        flags = Qt.ItemIsEnabled | Qt.ItemIsSelectable

        if index.column() == 1 and not item.is_section:
            flags |= Qt.ItemIsEditable

        return flags

    @QtCore.pyqtSlot()
    def removeRow(self, index):
        parent = index.parent()
        self.beginRemoveRows(parent, index.row(), index.row())

        if not parent.isValid():
            parentNode = self.rootItem
        else:
            parentNode = parent.internalPointer()

        parentNode.removeChild(index.row())

        self.endRemoveRows()
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

    def to_dict(self):  # TODO recursive
        d = {}
        for section in self.rootItem.childItems:
            section_d = {}
            section_name, _ = section.itemData

            for item in section.childItems:
                option, value = item.itemData
                section_d[option] = value

            d[section_name] = section_d

        return d


class ConfigDialog(config_editor.Ui_config_dialog):
    def __init__(self, data):
        super(ConfigDialog, self).__init__()
        self.model = ConfigModel(data)

    def setupUi(self, config_dialog):
        super(ConfigDialog, self).setupUi(config_dialog)

        self.config_view.setModel(self.model)
        self.config_view.expandAll()

        self.delete_button.pressed.connect(self.remove_selected)

    def remove_selected(self):
        index = self.config_view.selectedIndexes()[0]
        self.model.removeRow(index)

        #print(self.model.to_dict())


if __name__ == '__main__':
    import sys


    def except_hook(cls, exception, traceback):
        sys.__excepthook__(cls, exception, traceback)

    sys.excepthook = except_hook

    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()

    data = {"section 1": {"opt1": "str", "opt2": 123, "opt3": 1.23, "opt4": False, "...": {'subopt': 'bal'}},
            "section 2": {"opt1": "str", "opt2": [1.1, 2.3, 34], "opt3": 1.23, "opt4": False, "...": ""}}

    ui = ConfigDialog(data)
    ui.setupUi(Dialog)


    Dialog.show()
    print(app.exec_())

    print(Dialog.result())
    print(ui.model.to_dict())

    sys.exit()
