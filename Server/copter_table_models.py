import re
import sys
import time
import math
import indexed

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt as Qt


ModelDataRole = 998
ModelStateRole = 999


class ModelChecks:
    checks_dict = {}
    takeoff_checklist = (3, 4, 6, 7, 8)

    battery_min = 50.0  # config.getfloat('CHECKS', 'battery_percentage_min')
    start_pos_delta_max = 1.0  # config.getfloat('CHECKS', 'start_pos_delta_max')
    time_delta_max = 1.0

    @classmethod
    def col_check(cls, col):
        def inner(f):
            def wrapper(item):
                if item is not None:
                    return f(item)
                return None

            cls.checks_dict[col] = wrapper
            return wrapper

        return inner

    @classmethod
    def all_checks(cls, copter_item):
        for col, check in cls.checks_dict.items():
            if not check(copter_item[col]):
                return False
        return True

    @classmethod
    def takeoff_checks(cls, copter_item):
        for col in cls.takeoff_checklist:
            if not cls.checks_dict[col](copter_item[col]):
                return False
        return True


@ModelChecks.col_check(1)
def check_ver(item):
    return True  # TODO git version!


@ModelChecks.col_check(2)
def check_anim(item):
    return str(item) != 'No animation'


@ModelChecks.col_check(3)
def check_bat(item):
    if item == "NO_INFO":
        return False
    return item[1]*100 > ModelChecks.battery_min


@ModelChecks.col_check(4)
def check_sys_status(item):
    return item == "STANDBY"


@ModelChecks.col_check(5)
def check_cal_status(item):
    return item == "OK"


@ModelChecks.col_check(6)
def check_mode(item):
    return (item != "NO_FCU") and not ("CMODE" in item)


@ModelChecks.col_check(7)
def check_selfcheck(item):
    return item == "OK"


@ModelChecks.col_check(8)
def check_pos_status(item):
    if item == 'NO_POS':
        return False
    return not math.isnan(item[0])


@ModelChecks.col_check(9)
def check_start_pos_status(item):
    return item != 'NO_POS'


@ModelChecks.col_check(10)
def check_time_delta(item):
    return abs(item) < ModelChecks.time_delta_max


class CopterData:
    class_basic_attrs = indexed.IndexedOrderedDict([('copter_id', None), ('git_ver', None), ('anim_id', None),
                                                    ('battery', None), ('sys_status', None), ('cal_status', None),
                                                    ('mode', None), ('selfcheck', None), ('position', None),
                                                    ('start_pos', None), ('time_delta', None), ('client', None)])

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

    def __init__(self, checks_class=ModelChecks, **kwargs):
        self.states = CopterData(**self.class_basic_states)
        self.checks = ModelChecks

        super(StatedCopterData, self).__init__(**kwargs)

    def __setattr__(self, key, value):
        self.__dict__[key] = value

        if key in self.class_basic_attrs.keys():
            try:
                self.states.__dict__[key] = \
                    ModelChecks.checks_dict[self.attrs_dict.keys().index(key)](value)
                if key == 'start_pos':
                    if (self.__dict__['position'] is not None) and (self.__dict__['start_pos'] is not None):
                        current_pos = get_position(self.__dict__['position'])                   
                        start_pos = get_position(self.__dict__['start_pos'])
                        delta = get_position_delta(current_pos, start_pos)
                        if delta != 'NO_POS':
                            self.states.__dict__[key] = (delta < ModelChecks.start_pos_delta_max)
            except KeyError:  # No check present for that col
                pass
            else:  # update selfchecked and takeoff_ready
                self.states.__dict__["selfchecked"] = all(
                    [self.states[i] for i in ModelChecks.checks_dict.keys()]
                )

                self.states.__dict__["takeoff_ready"] = all(
                    [self.states[i] for i in ModelChecks.takeoff_checklist]
                )

def get_position(pos_array):
    if pos_array[0] != 'nan' and pos_array != 'NO_POS':
        pos = []
        for i in range(3):
            pos.append(pos_array[i])
    else:
        pos = 'NO_POS'
    return pos


def get_position_delta(pos1, pos2):
    if pos1 != 'NO_POS' and pos2 != 'NO_POS':
        delta_squared = 0
        for i in range(3):
            delta_squared += (pos1[i]-pos2[i])**2
        return math.sqrt(delta_squared)
    return 'NO_POS'


class ModelFormatter:
    view_formatters = {}
    place_formatters = {}
    VIEW_FORMATTER = False
    PLACE_FORMATTER = True

    @classmethod
    def format_view(cls, col, value):
        if col in cls.view_formatters:
            return cls.view_formatters[col](value)
        return value

    @classmethod
    def format_place(cls, col, value):
        if col in cls.place_formatters:
            return cls.place_formatters[col](value)
        return value

    @classmethod
    def col_format(cls, col, format_type):
        def inner(f):
            if format_type:
                cls.place_formatters[col] = f
            else:
                cls.view_formatters[col] = f

            def wrapper(*args, **kwargs):
                return f(*args, **kwargs)

            return wrapper

        return inner


@ModelFormatter.col_format(0, ModelFormatter.PLACE_FORMATTER)
def place_id(value):
    value = value.stip()
    # check user hostname spelling http://man7.org/linux/man-pages/man7/hostname.7.html
    # '-' (hyphen) not first; latin letters/numbers/hyphens; length form 1 to 63
    # or matches command pattern
    if re.match("^(?!-)[A-Za-z0-9-]{1,63}$", value) or re.match("^/[A-Za-z0-9]*$", value):
        return value
    else:
        msgbox = QtWidgets.QMessageBox()
        msgbox.setWindowTitle("Wrong input for the copter name!")
        msgbox.setIcon(QtWidgets.QMessageBox.Critical)
        msgbox.setText(
            "Wrong input for the copter name!\n"
            "Please use only A-Z, a-z, 0-9, and '-' chars.\n"
            "Don't use '-' as first char.")
        msgbox.exec_()
        return None


@ModelFormatter.col_format(3, ModelFormatter.PLACE_FORMATTER)
def place_battery(value):
    if isinstance(value, list):
        battery_v, battery_p = value
        if math.isnan(battery_v) or math.isnan(battery_p):
            return "NO_INFO"
    return value


@ModelFormatter.col_format(3, ModelFormatter.VIEW_FORMATTER)
def view_battery(value):
    if isinstance(value, list):
        battery_v, battery_p = value
        return "{:.1f}V {:d}%".format(battery_v, int(battery_p*100))
    return value

@ModelFormatter.col_format(7, ModelFormatter.VIEW_FORMATTER)
def view_selfcheck(value):
    if isinstance(value, list):
        if len(value)==1:
            return value[0]
        return "ERROR"
    return value

@ModelFormatter.col_format(8, ModelFormatter.VIEW_FORMATTER)
def view_selfcheck(value):
    if isinstance(value, list):
        x, y, z, yaw, frame = value
        return "{:.2f} {:.2f} {:.2f} {:d} {}".format(x, y, z, int(yaw), frame)
    return value

@ModelFormatter.col_format(9, ModelFormatter.VIEW_FORMATTER)
def view_selfcheck(value):
    if isinstance(value, list):
        x, y, z = value
        return "{:.2f} {:.2f} {:.2f}".format(x, y, z)
    return value

@ModelFormatter.col_format(10, ModelFormatter.PLACE_FORMATTER)
def place_time_delta(value):
    return abs(value - time.time())


@ModelFormatter.col_format(10, ModelFormatter.VIEW_FORMATTER)
def view_time_delta(value):
    return "{:.3f}".format(value)  

class CopterDataModel(QtCore.QAbstractTableModel):
    selected_ready_signal = QtCore.pyqtSignal(bool)
    selected_takeoff_ready_signal = QtCore.pyqtSignal(bool)
    selected_flip_ready_signal = QtCore.pyqtSignal(bool)  # TODO fix this signals
    selected_calibrating_signal = QtCore.pyqtSignal(bool)
    selected_calibration_ready_signal = QtCore.pyqtSignal(bool)

    def __init__(self, checks=ModelChecks, formatter=ModelFormatter, parent=None):
        super(CopterDataModel, self).__init__(parent)
        self.headers = ('copter ID', 'version', ' animation ID ', '  battery  ', '  system  ', 'sensors', 
                        '  mode  ', ' checks ', 'current x y z yaw frame_id', '    start x y z    ', 'dt')
        self.data_contents = []

        self.checks = checks
        self.formatter = formatter

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
        return filter(flip_checks, contents)  # possibly change as takeoff checks

    def calibrating(self, contents=()):
        contents = contents or self.data_contents
        return filter(calibrating_check, contents)

    def calibration_ready(self, contents=()):
        contents = contents or self.data_contents
        return filter(calibration_ready_check, contents)

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
        if role == Qt.DisplayRole and orientation == Qt.Horizontal:
            return self.headers[section]

    def data(self, index, role=Qt.DisplayRole):
        row = index.row()
        col = index.column()
        if role == Qt.DisplayRole or role == Qt.EditRole:  # Separate editRole in case of editing non-text
            item = self.data_contents[row][col]
            return str(self.formatter.format_view(col, item)) if item is not None else ""
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
        elif role == Qt.EditRole:  # For user/outer actions with data, place modifiers applied
            formatted_value = self.formatter.format_place(col, value)
            if formatted_value is not None:  # todo use new := syntax
                self.data_contents[row][col] = formatted_value

                if col == 0:
                    self.data_contents[row].client.send_message("id", {"new_id": formatted_value})
                    self.data_contents[row].client.remove()

        elif role == ModelDataRole:  # For inner setting\editing of data
            self.data_contents[row][col] = value
        elif role == ModelStateRole:
            self.data_contents[row].states[col] = value
        else:
            return False

        self.update_model(index, role)
        return True

    def select_all(self):  # probably NOT thread-safe!
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

    @QtCore.pyqtSlot(int)  # Probably deprecated now
    def remove_row(self, row):
        self.removeRows(row)

    @QtCore.pyqtSlot(object)
    def remove_row_data(self, data):
        row = self.get_row_index(data)
        if row is not None:
            self.removeRows(row)

def flip_checks(copter_item):
    for col in ModelChecks.takeoff_checklist:
        if col != 4 or col != 7:
            if not ModelChecks.checks_dict[col](copter_item[col]):
                return False
        elif copter_item[4] != "ACTIVE":
            return False
    return True


def calibrating_check(copter_item):
    return copter_item[5] == "CALIBRATING"


def calibration_ready_check(copter_item):
    if not ModelChecks.checks_dict[4](copter_item[4]):
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
    remove_row_signal = QtCore.pyqtSignal(int)
    remove_client_signal = QtCore.pyqtSignal(object)

    def __init__(self, model):
        super().__init__()

        self.update_data_signal.connect(model.update_item)
        self.add_client_signal.connect(model.add_client)
        self.remove_row_signal.connect(model.remove_row)
        self.remove_client_signal.connect(model.remove_row_data)


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
