import sys
import re
import collections
import indexed
from server import ConfigOption

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt as Qt


ModelDataRole = 998
ModelStateRole = 999


class CopterData:
    class_basic_attrs = indexed.IndexedOrderedDict([('copter_id', None), ('anim_id', None),
                                                    ('batt_v', None), ('batt_p', None),
                                                    ('sys_status', None), ('cal_status', None), ('selfcheck', None),
                                                    ('position', None), ("time_delta", None),
                                                    ("client", None), ])

    def __init__(self, **kwargs):
        self.attrs_dict = self.class_basic_attrs.copy()
        self.attrs_dict.update(kwargs)

        for attr, value in self.attrs_dict.items():
            setattr(self, attr, value)

    def __getitem__(self, key):
        return getattr(self, self.attrs_dict.keys()[key])

    def __setitem__(self, key, value):
        setattr(self, self.attrs_dict.keys()[key], value)


class StatedCopterData(CopterData):
    class_basic_states = indexed.IndexedOrderedDict([("checked", 0), ("selfchecked", None), ("takeoff_ready", None),
                                                    ("copter_id", True), ])

    def __init__(self, **kwargs):
        self.states = CopterData(**self.class_basic_states)

        super(StatedCopterData, self).__init__(**kwargs)

    def __setattr__(self, key, value):
        self.__dict__[key] = value

        if key in self.class_basic_attrs.keys():
            try:
                self.states.__dict__[key] = \
                    Checks.all_checks[self.attrs_dict.keys().index(key)](value)
            except KeyError:  # No check present for that col
                pass
            else:  # update selfchecked and takeoff_ready
                self.states.__dict__["selfchecked"] = all(
                    [self.states[i] for i in Checks.all_checks.keys()]
                )

                self.states.__dict__["takeoff_ready"] = all(
                    [self.states[i] for i in Checks.takeoff_checklist]
                )


class Checks:
    all_checks = {}
    takeoff_checklist = (2, 3, 4, 5, 6)


class CopterDataModel(QtCore.QAbstractTableModel):
    selected_ready_signal = QtCore.pyqtSignal(bool)
    selected_takeoff_ready_signal = QtCore.pyqtSignal(bool)
    selected_flip_ready_signal = QtCore.pyqtSignal(bool)
    selected_calibrating_signal = QtCore.pyqtSignal(bool)
    selected_calibration_ready_signal = QtCore.pyqtSignal(bool)

    def __init__(self, parent=None):
        super(CopterDataModel, self).__init__(parent)
        self.headers = ('copter ID', '  animation ID  ', 'batt V', 'batt %', '  system  ',
                        'calibration', 'selfcheck', 'current x y z yaw frame_id', 'time delta')
        self.data_contents = []

        self.on_id_changed = None

        self.first_col_is_checked = False

    def insertRows(self, contents, position='last', parent=QtCore.QModelIndex()):
        rows = len(contents)
        position = len(self.data_contents) if position == 'last' else position

        self.beginInsertRows(parent, position, position + rows - 1)
        self.data_contents[position:position] = contents

        self.endInsertRows()

    def removeRows(self, position, rows=1, index=QtCore.QModelIndex()):
        self.beginRemoveRows(QtCore.QModelIndex(), position, position + rows - 1)
        self.data_contents = self.data_contents[:position] + self.data_contents[position + rows:]
        self.endRemoveRows()

        return True

    def user_selected(self, contents=()):
        contents = contents or self.data_contents
        return filter(lambda x: x.states.checked == Qt.Checked, contents)

    def selfchecked_ready(self, contents=()):
        contents = contents or self.data_contents
        return filter(lambda x: x.states.selfchecked, contents)

    def takeoff_ready(self, contents=()):
        contents = contents or self.data_contents
        return filter(lambda x: x.states.takeoff_ready, contents)

    def flip_ready(self, contents=()):
        contents = contents or self.data_contents
        return filter(lambda x: flip_checks(x), contents)  # possibly change as takeoff checks

    def calibrating(self, contents=()):
        contents = contents or self.data_contents
        return filter(lambda x: calibrating_check(x), contents)

    def calibration_ready(self, contents=()):
        contents = contents or self.data_contents
        return filter(lambda x: calibration_ready_check(x), contents)

    def get_row_index(self, row_data):
        try:
            index = self.data_contents.index(row_data)
        except ValueError:
            return None
        else:
            return index

    def get_row_by_attr(self, attr, value):
        try:
            row_data = next(filter(lambda x: getattr(x, attr, None) == value, self.data_contents))
        except StopIteration:
            return None
        else:
            return row_data

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
        if role == Qt.DisplayRole or role == Qt.EditRole:  # Separate editRole in case of editing non-text
            item = self.data_contents[row][col]
            return str(item) if item is not None else ""
        elif role == ModelDataRole:
            return self.data_contents[row][col]

        elif role == Qt.BackgroundRole:
            try:
                item = self.data_contents[row]
                result = item.states[col]
            except KeyError:
                return QtGui.QBrush(Qt.white)
            else:
                if result is None:
                    return QtGui.QBrush(Qt.yellow)
                if result:
                    return QtGui.QBrush(Qt.green)
                else:
                    return QtGui.QBrush(Qt.red)

        elif role == Qt.CheckStateRole and col == 0:
            return self.data_contents[row].states.checked

        if role == QtCore.Qt.TextAlignmentRole and col != 0:
                return QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter

    def update_model(self, index=QtCore.QModelIndex(), role=QtCore.Qt.EditRole):
        selected = set(self.user_selected())

        self.selected_ready_signal.emit(selected.issubset(self.selfchecked_ready()))
        self.selected_takeoff_ready_signal.emit(selected.issubset(self.takeoff_ready()))

        self.selected_flip_ready_signal.emit(selected.issubset(self.flip_ready()))
        self.selected_calibrating_signal.emit(selected.issubset(self.calibrating()))
        self.selected_calibration_ready_signal.emit(selected.issubset(self.calibration_ready()))

        self.dataChanged.emit(index, index, (role,))

    @QtCore.pyqtSlot()
    def setData(self, index, value, role=Qt.EditRole):
        if not index.isValid():
            return False

        col = index.column()
        row = index.row()

        if role == Qt.CheckStateRole:
            self.data_contents[row].states.checked = value
        elif role == Qt.EditRole:  # For user actions with data
            if col == 0 and self.on_id_changed:
                self.data_contents[row][col] = "Awaiting for response"
                self.data_contents[row].states.copter_id = None

                self.data_contents[row].client.send_message("id", {"new_id": value})
            else:
                self.data_contents[row][col] = value

        elif role == ModelDataRole:  # For inner setting\editing of data
            self.data_contents[row][col] = value
        elif role == ModelStateRole:
            self.data_contents[row].states[col] = value
        else:
            return False

        self.update_model(index, role)
        return True

    def select_all(self):
        self.first_col_is_checked = not self.first_col_is_checked
        for row_num, copter in enumerate(self.data_contents):
            copter.states.checked = int(self.first_col_is_checked)*2
            self.update_model(self.index(row_num, 0), Qt.CheckStateRole)

    def flags(self, index):
        roles = Qt.ItemIsSelectable | Qt.ItemIsEnabled
        if index.column() == 0:
            roles |= Qt.ItemIsUserCheckable | Qt.ItemIsEditable
        return roles

    @QtCore.pyqtSlot(int, int, QtCore.QVariant, QtCore.QVariant)
    def update_item(self, row, col, value, role=Qt.EditRole):
        self.setData(self.index(row, col), value, role)

    @QtCore.pyqtSlot(object)
    def add_client(self, client):
        self.insertRows([client])

    @QtCore.pyqtSlot(int)
    def remove_client(self, row):
        self.removeRows(row)


def col_check(col):
    def inner(f):
        Checks.all_checks[col] = f

        def wrapper(*args, **kwargs):
            return f(*args, **kwargs)

        return wrapper

    return inner


@col_check(1)
def check_anim(item):
    if not item:
        return None
    return str(item) != 'No animation'


@col_check(2)
def check_bat_v(item):
    if not item:
        return None
    return float(item) > 3.2


@col_check(3)
def check_bat_p(item):
    if not item:
        return None
    return float(item) > 30


@col_check(4)
def check_sys_status(item):
    if not item:
        return None
    return item == "STANDBY"


@col_check(5)
def check_cal_status(item):
    if not item:
        return None
    return item == "OK"


@col_check(6)
def check_selfcheck(item):
    if not item:
        return None
    return item == "OK"


@col_check(7)
def check_cal_status(item):
    if not item:
        return None
    return True


@col_check(8)
def check_time_delta(item):
    if not item:
        return None
    return abs(float(item)) < 1


def all_checks(copter_item):
    for col, check in Checks.all_checks.items():
        if not check(copter_item[col]):
            return False
    return True


def takeoff_checks(copter_item):
    for col in Checks.takeoff_checklist:
        if not Checks.all_checks[col](copter_item[col]):
            return False
    return True


def flip_checks(copter_item):
    for col in Checks.takeoff_checklist:
        if col != 4:
            if not Checks.all_checks[col](copter_item[col]):
                return False
        else:
            if copter_item[4] != "ACTIVE":
                return False
    return True


def calibrating_check(copter_item):
    return copter_item[5] == "CALIBRATING"


def calibration_ready_check(copter_item):
    if not Checks.all_checks[4](copter_item[4]):
        return False
    return not calibrating_check(copter_item)


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
    update_data_signal = QtCore.pyqtSignal(int, int, QtCore.QVariant, QtCore.QVariant)
    add_client_signal = QtCore.pyqtSignal(object)
    remove_client_signal = QtCore.pyqtSignal(int)


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

    msgs = []
    msg = "[{}]: Failure: {}".format("FCU connection", "Angular velocities estimation is not available")
    msgs.append(msg)
    msg = "[{}]: Failure: {}".format("FCU connection1", "Angular velocities estimation is not available")
    msgs.append(msg)
    msg = "[{}]: Failure: {}".format("FCU connection2", "Angular velocities estimation is not available")
    msgs.append(msg)

    myModel.add_client(StatedCopterData(copter_id=1000, checked=0, selfcheck=msgs, time_utc=1))
    myModel.add_client(StatedCopterData(checked=2, selfcheck="OK", time_utc=2))
    myModel.add_client(StatedCopterData(checked=2, selfcheck="not ok", time_utc="no"))

    myModel.setData(myModel.index(0, 1), "test")

    t = threading.Thread(target=timer, daemon=True)
    t.start()
    print(QtCore.QT_VERSION_STR)

    app.exec_()
