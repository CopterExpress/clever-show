import sys
import re
import collections

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt as Qt


class CopterData:
    class_attrs = collections.OrderedDict([('copter_id', None), ('anim_id', None), ('batt_v', None), ('batt_p', None),
                                           ('sys_status', None), ('cal_status', None), ('selfcheck', None), ("time_delta", None),
                                           ("client", None), ("checked", 0)], )

    def __init__(self, **kwargs):
        self.attrs = self.class_attrs.copy()
        self.attrs.update(kwargs)

        for attr, value in self.attrs.items():
            setattr(self, attr, value)

    def __getitem__(self, key):
        return getattr(self, list(self.attrs.keys())[key])

    def __setitem__(self, key, value):
        setattr(self, list(self.attrs.keys())[key], value)


class CopterDataModel(QtCore.QAbstractTableModel):
    checks = {}
    selected_ready_signal = QtCore.pyqtSignal(bool)
    selected_takeoff_ready_signal = QtCore.pyqtSignal(bool)
    selected_not_calibrating_signal = QtCore.pyqtSignal(bool)

    def __init__(self, parent=None):
        super(CopterDataModel, self).__init__(parent)
        self.headers = ('copter ID', 'animation ID', 'battery V', 'battery %', 'system status', 'calibration status', 'selfcheck', 'time delta')
        self.data_contents = []

    def insertRows(self, contents, position='last', parent=QtCore.QModelIndex()):
        rows = len(contents)
        position = len(self.data_contents) if position == 'last' else position

        self.beginInsertRows(parent, position, position + rows - 1)
        self.data_contents[position:position] = contents

        self.endInsertRows()

    def user_selected(self):
        return filter(lambda x: x.checked == Qt.Checked, self.data_contents)

    def selfchecked_ready(self, contents=()):
        contents = contents or self.data_contents
        return filter(lambda x: all_checks(x), contents)

    def takeoff_ready(self, contents=()):
        contents = contents or self.data_contents
        return filter(lambda x: takeoff_checks(x), contents)

    def not_calibrating(self, contents=()):
        contents = contents or self.data_contents
        return filter(lambda x: not_calibrating_check(x), contents)

    def rowCount(self, n=None):
        return len(self.data_contents)

    def columnCount(self, n=None):
        return len(self.headers)

    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                return self.headers[section]

    def data(self, index, role=Qt.DisplayRole):
        row = index.row()
        col = index.column()
        #print('row {}, col {}, role {}'.format(row, col, role))
        if role == Qt.DisplayRole:
            #print(self.data_contents[row][col])
            return self.data_contents[row][col] or ""

        elif role == Qt.BackgroundRole:
            if col in self.checks.keys():
                item = self.data_contents[row][col]
                result = self.checks[col](item)
                if result is None:
                    return QtGui.QBrush(Qt.yellow)
                if result:
                    return QtGui.QBrush(Qt.green)
                else:
                    return QtGui.QBrush(Qt.red)

        elif role == Qt.CheckStateRole and col == 0:
            return self.data_contents[row].checked

        if role == QtCore.Qt.TextAlignmentRole and col != 0:
                return QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter

    def update_model(self, index=QtCore.QModelIndex()):
        #self.modelReset.emit()
        self.selected_ready_signal.emit(set(self.user_selected()).issubset(self.selfchecked_ready()))
        self.selected_takeoff_ready_signal.emit(set(self.user_selected()).issubset(self.takeoff_ready()))
        self.selected_not_calibrating_signal.emit(set(self.user_selected()).issubset(self.not_calibrating()))
        self.dataChanged.emit(index, index, (QtCore.Qt.EditRole,))

    @QtCore.pyqtSlot()
    def setData(self, index, value, role=Qt.EditRole):
        if not index.isValid():
            return False

        if role == Qt.CheckStateRole:
            self.data_contents[index.row()].checked = value

        elif role == Qt.EditRole:
            self.data_contents[index.row()][index.column()] = value
            self.update_model(index)
        else:
            return False

        return True

    def flags(self, index):
        roles = Qt.ItemIsSelectable | Qt.ItemIsEnabled
        if index.column() == 0:
            roles |= Qt.ItemIsUserCheckable
        return roles

    @QtCore.pyqtSlot(int, int, QtCore.QVariant)
    def update_item(self, row, col, value):
        self.setData(self.index(row, col), value)

    @QtCore.pyqtSlot(object)
    def add_client(self, client):
        self.insertRows([client])


def col_check(col):
    def inner(f):
        CopterDataModel.checks[col] = f

        def wrapper(*args, **kwargs):
            return f(*args, **kwargs)

        return wrapper

    return inner


@col_check(1)
def check_anim(item):
    if not item:
        return None
    if str(item) == 'No animation':
        return False
    else:
        return True


@col_check(2)
def check_bat_v(item):
    if not item:
        return None
    if float(item) > 3.2:  # todo config
        return True
    else:
        return False


@col_check(3)
def check_bat_p(item):
    if not item:
        return None
    if float(item) > 30:  # todo config
        return True
    else:
        return False
        #return True #For testing

@col_check(4)
def check_sys_status(item):
    if not item:
        return None
    if item == "MAV_STATE_STANDBY":
        return True
    else:
        return False  

@col_check(5)
def check_cal_status(item):
    if not item:
        return None
    if item == "OK":
        return True
    else:
        return False

@col_check(6)
def check_selfcheck(item):
    if not item:
        return None
    if item == "OK":
        return True
    else:
        return False

@col_check(7)
def check_time_delta(item):
    if not item:
        return None
    if abs(float(item)) < 1:
        return True
    else:
        return False


def all_checks(copter_item):
    for col, check in CopterDataModel.checks.items():
        if not check(copter_item[col]):
            return False
    return True

def takeoff_checks(copter_item):
    for i in range(5):
        if not CopterDataModel.checks[2+i](copter_item[2+i]):
            return False
    return True

def not_calibrating_check(copter_item):
    if copter_item[5] == "CALIBRATING":
        return False
    return True

class CopterProxyModel(QtCore.QSortFilterProxyModel):
    def __init__(self, parent=None):
        super(CopterProxyModel, self).__init__(parent)

    @staticmethod
    def human_sort_prepare(item):
        if item:
            item = [int(x) if x.isdigit() else x.lower() for x in re.split('([0-9]+)', str(item))]
        else:
            item = []
        return item

    def lessThan(self, left, right):
        leftData = self.sourceModel().data(left)
        rightData = self.sourceModel().data(right)

        return self.human_sort_prepare(leftData) < self.human_sort_prepare(rightData)


class SignalManager(QtCore.QObject):
    update_data_signal = QtCore.pyqtSignal(int, int, QtCore.QVariant)
    add_client_signal = QtCore.pyqtSignal(object)


if __name__ == '__main__':
    import threading
    import time

    def timer():
        idc = 1001
        while True:
            myModel.setData(myModel.index(0, 0), idc)
            idc += 1
            time.sleep(1)

    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)
    tableView = QtWidgets.QTableView()
    myModel = CopterDataModel(None)
    proxyModel = CopterProxyModel()

    proxyModel.setDynamicSortFilter(True)
    proxyModel.setSourceModel(myModel)

    tableView.setModel(proxyModel)

    tableView.verticalHeader().hide()
    tableView.setSortingEnabled(True)

    tableView.show()
    myModel.add_client(CopterData(copter_id=1000, checked=0, time_utc=1))
    myModel.add_client(CopterData(checked=2, selfcheck="OK", time_utc=2))
    myModel.add_client(CopterData(checked=2, selfcheck="not ok", time_utc="no"))

    myModel.setData(myModel.index(0, 1), "test")

    t = threading.Thread(target=timer, daemon=True)
    t.start()
    print(QtCore.QT_VERSION_STR)

    app.exec_()
