import math
import os
import re
import subprocess
import sys
from contextlib import suppress
from functools import partialmethod

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt as Qt, QUrl, QDir

# Additional custom roles to interact with various table data
ModelDataRole = 998
ModelStateRole = 999

def get_git_version():  # TODO import from animation
    return subprocess.check_output("git log --pretty=format:%h -n 1").decode('UTF-8')



class ModelChecks:
    checks_dict = {}

    battery_min = 50.0
    start_pos_delta_max = 1.0
    time_delta_max = 1.0

    @classmethod
    def column_check(cls, column, pass_context=False):
        def inner(f):
            def wrapper(item, context=None):
                if item is None:
                    return None
                if pass_context:
                    return f(item, context)
                return f(item)

            cls.checks_dict[column] = wrapper
            return wrapper

        return inner

    @classmethod
    def check(cls, column, context):
        if isinstance(column, int):
            column = context.columns[column]
        item = context[column]
        try:
            return cls.checks_dict[column](item, context)
        except KeyError:  # When there is no check
            return None if item is None else True  # item is not None

    @classmethod
    def all_checks(cls, copter_item):
        for col, check in cls.checks_dict.items():
            if not check(copter_item[col]):
                return False
        return True


@ModelChecks.column_check("git_version")
def check_ver(item):
    return get_git_version() == item


@ModelChecks.column_check("animation_id")
def check_anim(item):
    return str(item) != 'No animation'


@ModelChecks.column_check("battery")
def check_bat(item):
    if item == "NO_INFO":
        return False
    return item[1] * 100 > ModelChecks.battery_min


@ModelChecks.column_check("fcu_status")
def check_sys_status(item):
    return item == "STANDBY"


@ModelChecks.column_check("calibration_status")
def check_cal_status(item):
    return item == "OK"


@ModelChecks.column_check("mode")
def check_mode(item):
    return (item != "NO_FCU") and not ("CMODE" in item)


@ModelChecks.column_check("selfcheck")
def check_selfcheck(item):
    return item == "OK"


@ModelChecks.column_check("current_position")
def check_pos(item):
    if item == 'NO_POS':
        return False
    return not math.isnan(item[0])

# @ModelChecks.column_check("last_task")
# def check_task(item):
#     return True


@ModelChecks.column_check('time_delta')
def check_time_delta(item):
    return abs(item) < ModelChecks.time_delta_max


@ModelChecks.column_check("start_position", pass_context=True)
def check_start_pos(item, context):
    if context.current_position is None:
        return item != 'NO_POS'  # maybe should return true

    delta = get_distance(get_position(context.current_position),
                         get_position(context.start_position))
    if math.isnan(delta):
        return False

    return delta < ModelChecks.start_pos_delta_max


def get_position(position):
    if position != 'NO_POS' and position[0] != 'nan':
        return position
    return [float('nan')]*3


def get_distance(pos1, pos2):  # todo as common function
    if any(math.isnan(x) for x in pos1+pos2):
        return float('nan')
    return math.sqrt(sum(map(lambda p: p[0] - p[1], zip(pos1, pos2)))**2)  # point distance formula


class CopterData:
    def __init__(self, columns=(), **kwargs):
        self.columns = columns
        for column in columns:
            setattr(self, column, None)

        for attr, value in kwargs.items():
            setattr(self, attr, value)

    def __getitem__(self, key):
        if key in self.columns:
            return getattr(self, key)
        return getattr(self, self.columns[key])

    def __setitem__(self, key, value):
        if key in self.columns:
            setattr(self, key, value)
        else:
            setattr(self, self.columns[key], value)


class StatedCopterData(CopterData):
    def __init__(self, columns=(), checks_defaults=None, checks_class=ModelChecks, **kwargs):
        if checks_defaults is None:
            checks_defaults = {}

        self.__dict__['states'] = CopterData(columns, **checks_defaults)
        self.__dict__['checks'] = checks_class
        self.__dict__['all_checks'] = None

        super().__init__(columns, **kwargs)

    def __setattr__(self, key, value):
        self.__dict__[key] = value

        if key in self.columns:
            with suppress(KeyError):
                self.states.__dict__[key] = \
                    self.checks.check(key, self)  # {key: self.__dict__[key] for key in self.columns})
                self.states.__dict__["all_checks"] = all([self.states[i] for i in self.checks.checks_dict.keys()])


class ModelFormatter:
    view_formatters = {}
    place_formatters = {}
    VIEW_FORMATTER = 1
    PLACE_FORMATTER = 2

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
    def column_formatter(cls, col, formatter_type):
        def inner(f):
            if formatter_type == cls.PLACE_FORMATTER:
                cls.place_formatters[col] = f
            elif formatter_type == cls.VIEW_FORMATTER:
                cls.view_formatters[col] = f

            def wrapper(*args, **kwargs):
                return f(*args, **kwargs)

            return wrapper

        return inner

    place_formatter = partialmethod(column_formatter, formatter_type=PLACE_FORMATTER)
    view_formatter = partialmethod(column_formatter, formatter_type=VIEW_FORMATTER)

@ModelFormatter.place_formatter("copter_id")
def place_id(value):
    value = str(value).strip()
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


@ModelFormatter.place_formatter("battery")
def place_battery(value):
    if isinstance(value, list):
        battery_v, battery_p = value
        if math.isnan(battery_v) or math.isnan(battery_p):
            return "NO_INFO"
    return value

@ModelFormatter.view_formatter("battery")
def view_battery(value):
    if isinstance(value, list):
        battery_v, battery_p = value
        return f"{battery_v:4.1f}V {min(battery_p, 1):4.0%}"
    return value


@ModelFormatter.view_formatter("selfcheck")
def view_selfcheck(value):
    if isinstance(value, list):
        if len(value) == 1 and len(value[0]) <= 8:
            return value[0]
        return "ERROR"
    return value


@ModelFormatter.view_formatter("current_position")
def view_selfcheck(value):
    if isinstance(value, list):
        x, y, z, yaw, frame = value
        return f"{x: .2f} {y: .2f} {z: .2f} {int(yaw): d} {frame}"
    return value


@ModelFormatter.view_formatter("start_position")
def view_selfcheck(value):
    if isinstance(value, list):
        x, y, z = value
        return f"{x: .2f} {y: .2f} {z: .2f}"
    return value


@ModelFormatter.place_formatter("last_task")
def place_last_task(value):
    if value is None:  # TODO possible behaviour deviation
        return 'No task'
    return value


@ModelFormatter.place_formatter("time_delta")
def place_time_delta(value):
    return abs(value - time.time())


@ModelFormatter.view_formatter("time_delta")
def view_time_delta(value):
    return f"{value:.3f}"


class CopterDataModel(QtCore.QAbstractTableModel):
    columns_dict = {'copter_id': 'copter ID',
                    'git_version': 'version',
                    'animation_id': ' animation ID ',
                    'battery': '  battery  ',
                    'fcu_status': 'FCU status',
                    'calibration_status': 'sensors',
                    'mode': '  mode  ',
                    'selfcheck': ' checks ',
                    'current_position': 'current x y z yaw frame_id',
                    'start_position': '    start x y z     ',
                    'last_task': 'last task',
                    'time_delta': 'dt',
                    'config_version': 'configuration',
                    }

    columns = list(columns_dict.keys())

    selected_ready_signal = QtCore.pyqtSignal(bool)
    selected_takeoff_ready_signal = QtCore.pyqtSignal(bool)
    selected_flip_ready_signal = QtCore.pyqtSignal(bool)  # TODO fix this signals
    selected_calibrating_signal = QtCore.pyqtSignal(bool)
    selected_calibration_ready_signal = QtCore.pyqtSignal(bool)

    update_data_signal = QtCore.pyqtSignal(int, int, QtCore.QVariant, QtCore.QVariant)
    add_client_signal = QtCore.pyqtSignal(object)
    remove_row_signal = QtCore.pyqtSignal(int)
    remove_client_signal = QtCore.pyqtSignal(object)

    def __init__(self, checks=ModelChecks, formatter=ModelFormatter, data_model=StatedCopterData, parent=None):
        super(CopterDataModel, self).__init__(parent)
        # self.headers = ('   copter ID   ', '   version   ', '    animation ID    ', '     battery     ', '  fcu_status  ', ' sensors ',
        #                 '       mode       ', ' checks ', ' current x y z yaw frame_id ', '       start x y z       ', '           task           ',  'dt')
        self.headers = list(self.columns_dict.values())
        self.data_contents = []

        self.checks = checks
        self.formatter = formatter
        self.data_model = data_model

        self.first_col_is_checked = False

        self.update_data_signal.connect(self._update_item)
        self.add_client_signal.connect(self._add_client)
        self.remove_row_signal.connect(self._remove_row)
        self.remove_client_signal.connect(self._remove_row_data)

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

    @classmethod
    def is_column(cls, index, column_name):
        return index.column() == cls.columns.index(column_name)

    def user_selected(self, contents=()):
        return self.filter(lambda x: x.states.checked == Qt.Checked, contents)

    def filter(self, f, contents=()):
        contents = contents or self.data_contents
        return filter(f, contents)

    def selected_check(self, f, selected=()):
        selected = selected or set(self.user_selected())
        print(selected and all(f(item) for item in selected))
        return bool(selected) and all(f(item) for item in selected) #selected.issubset(self.filter(f))

    def get_row_data(self, index):
        row = index.row()
        if row == -1:
            return None
        try:
            return self.data_contents[row]
        except IndexError:
            return None

    def get_row_index(self, row_data):
        try:
            return self.data_contents.index(row_data)
        except ValueError:
            return None

    def get_row_by_attr(self, attr, value):
        try:
            return next(filter(lambda x: getattr(x, attr, None) == value, self.data_contents))
        except StopIteration:
            return None

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
            return str(self.formatter.format_view(self.columns[col], item)) if item is not None else ""
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

        self.selected_ready_signal.emit(self.selected_check(lambda x: x.states.all_checks, selected))
        self.selected_takeoff_ready_signal.emit(self.selected_check(takeoff_checks, selected))
        self.selected_flip_ready_signal.emit(self.selected_check(flip_checks, selected))
        self.selected_calibrating_signal.emit(self.selected_check(calibrating_check, selected))
        self.selected_calibration_ready_signal.emit(self.selected_check(calibration_ready_check, selected))

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
            formatted_value = self.formatter.format_place(self.columns[col], value)
            if formatted_value is None:  # todo use new := syntax
                return False

            self.data_contents[row][col] = formatted_value

            if col == 0:
                self.data_contents[row].client.send_message("id", {"new_id": formatted_value})
                self.data_contents[row].client.remove()  # TODO change
                self._remove_row(row)

        elif role == ModelDataRole:  # For inner setting\editing of data
            self.data_contents[row][col] = value
        elif role == ModelStateRole:
            self.data_contents[row].states[col] = value
        else:
            return False

        self.update_model(index, role)
        return True

    def select_all(self):  # probably NOT thread-safe! TODO remake
        self.first_col_is_checked = not self.first_col_is_checked
        for row_num, copter in enumerate(self.data_contents):
            copter.states.checked = int(self.first_col_is_checked) * 2
            self.update_model(self.index(row_num, 0), Qt.CheckStateRole)

    def flags(self, index):
        roles = Qt.ItemIsSelectable | Qt.ItemIsEnabled
        if index.column() == 0:
            roles |= Qt.ItemIsUserCheckable | Qt.ItemIsEditable
        if self.is_column(index, "config_version"):
            roles |= Qt.ItemIsDragEnabled  # | Qt.ItemIsDropEnabled

        return roles

    def supportedDropActions(self):
        return QtCore.Qt.CopyAction

    def mimeTypes(self):
        return ['text/plain']

    def mimeData(self, indexes):
        index = indexes[0]
        if self.is_column(index, "config_version"):
            return self._config_mime(index)

        return None

    def _config_mime(self, index):
        mimedata = QtCore.QMimeData()
        path = os.path.join(QDir.tempPath(), "config_{}.ini".format(
            self.data_contents[index.row()].copter_id))

        with suppress(OSError):  # remove if file exists
            os.remove(path)

        self.data_contents[index.row()].client.get_file("config/client.ini", path, )
        mimedata.setUrls([QUrl.fromLocalFile(path)])

        return mimedata

    # Thread-safe wrappers
    def add_client(self, **kwargs):
        default_states = {"checked": 0, "copter_id": True}
        self.add_client_signal.emit(self.data_model(self.columns, default_states, **kwargs))

    def remove_client_data(self, row_data):
        self.remove_client_signal.emit(row_data)

    def remove_row(self, row):
        self.remove_row_signal.emit(row)

    def update_data(self, row, col, data, role=ModelDataRole):
        self.update_data_signal.emit(row, col, data, role)

    @QtCore.pyqtSlot(int, int, QtCore.QVariant, QtCore.QVariant)
    def _update_item(self, row, col, value, role=Qt.EditRole):
        self.setData(self.index(row, col), value, role)

    @QtCore.pyqtSlot(object)
    def _add_client(self, client):
        self.insertRows([client])

    @QtCore.pyqtSlot(int)  # Probably deprecated now
    def _remove_row(self, row):
        self.removeRows(row)

    @QtCore.pyqtSlot(object)
    def _remove_row_data(self, data):
        row = self.get_row_index(data)
        if row is not None:
            self.removeRows(row)


def check_checklist(copter_item, checklist=()):
    return all(copter_item.states[col] for col in checklist)


def takeoff_checks(copter_item):
    checklist = ("battery", "fcu_status", "mode", "selfcheck", "current_position")
    return check_checklist(copter_item, checklist)

def flip_checks(copter_item):
    checklist = ("battery", "mode", "current_position")
    if not check_checklist(copter_item, checklist):
        return False
    if copter_item["fcu_status"] != "ACTIVE":
    return True

# for col in checklist:
    #     if not copter_item.state[col]:  # ModelChecks.check(col, copter_item):

def calibrating_check(copter_item):


def calibration_ready_check(copter_item):
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


if __name__ == '__main__':
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
    myModel = CopterDataModel()
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

    #myModel._add_client(StatedCopterData(copter_id=1000, checked=0, selfcheck=msgs, time_utc=1))
    #myModel._add_client(StatedCopterData(checked=2, selfcheck="OK", time_utc=2))
    #myModel._add_client(StatedCopterData(checked=2, selfcheck="not ok", time_utc="no"))
    #myModel.setData(myModel.index(0, 1), "test")

    #t.start()
    print(QtCore.QT_VERSION_STR)
    print(get_git_version())
    myModel.update_data(0, 3, [1, 2], role=Qt.EditRole)
