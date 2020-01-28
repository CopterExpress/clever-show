import os
import re
import sys
import time
import math
from contextlib import suppress

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt as Qt, QUrl, QDir

ModelDataRole = 998
ModelStateRole = 999


class ModelChecks:
    checks_dict = {}
    takeoff_checklist = (3, 4, 6, 7, 8)

    battery_min = 50.0
    start_pos_delta_max = 1.0
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
    return item[1] * 100 > ModelChecks.battery_min


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
def check_selfcheck(item):
    return True


@ModelChecks.col_check(11)
def check_time_delta(item):
    return abs(item) < ModelChecks.time_delta_max





class CopterData:
    def __init__(self, columns=(), **kwargs):
        self._columns = columns
        for column in columns:
            setattr(self, column, None)

        for attr, value in kwargs.items():
            setattr(self, attr, value)

    def __getitem__(self, key):
        if key in self._columns:
            return getattr(self, key)
        return getattr(self, self._columns[key])

    def __setitem__(self, key, value):
        if key in self._columns:
            setattr(self, key, value)
        else:
            setattr(self, self._columns[key], value)


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

        if key in self._columns:
            with suppress(KeyError):
                # print(self.__dict__)
                self.states.__dict__[key] = self.checks.checks_dict[self._columns.index(key)](value)
                self.states.__dict__["all_checks"] = all([self.states[i] for i in ModelChecks.checks_dict.keys()])

                # if key == 'start_position':
                #     if (self.__dict__['current_position'] is not None) and (
                #             self.__dict__['start_position'] is not None):
                #         current_pos = get_position(self.__dict__['current_position'])
                #         start_pos = get_position(self.__dict__['start_position'])
                #         delta = get_position_delta(current_pos, start_pos)
                #         if delta != 'NO_POS':
                #             self.states.__dict__[key] = (delta < ModelChecks.start_pos_delta_max)

                # update all_checks and takeoff_ready

                # self.states.__dict__["takeoff_ready"] = all(
                #     [self.states[i] for i in ModelChecks.takeoff_checklist]
                # )


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
            delta_squared += (pos1[i] - pos2[i]) ** 2
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
        return "{:.1f}V {:d}%".format(battery_v, int(battery_p * 100))
    return value


@ModelFormatter.col_format(7, ModelFormatter.VIEW_FORMATTER)
def view_selfcheck(value):
    if isinstance(value, list):
        if len(value) == 1:
            if len(value[0]) <= 8:
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
def view_last_task(value):
    if value is None:
        return 'No task'
    return value


@ModelFormatter.col_format(11, ModelFormatter.PLACE_FORMATTER)
def place_time_delta(value):
    return abs(value - time.time())


@ModelFormatter.col_format(11, ModelFormatter.VIEW_FORMATTER)
def view_time_delta(value):
    return "{:.3f}".format(value)


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

    def filter(self, f, contents=()):
        contents = contents or self.data_contents
        return filter(f, contents)

    def user_selected(self, contents=()):
        return self.filter(lambda x: x.states.checked == Qt.Checked, contents)

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

        self.selected_ready_signal.emit(selected.issubset(self.filter(lambda x: x.states.all_checks)))
        #self.selected_takeoff_ready_signal.emit(selected.issubset(self.filter(lambda x: x.states.takeoff_ready)))
        self.selected_flip_ready_signal.emit(selected.issubset(self.filter(flip_checks)))
        self.selected_calibrating_signal.emit(selected.issubset(self.filter(calibrating_check)))
        self.selected_calibration_ready_signal.emit(selected.issubset(self.filter(calibration_ready_check)))

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
        # class_basic_attrs = {'client': None}
        # class_basic_states = OrderedDict([("checked", 0), ("selfchecked", None), ("takeoff_ready", None)])
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
    myModel.add_client(copter_id=1000, client=None)
    #myModel.setData(myModel.index(0, 1), "test")

   # t = threading.Thread(target=timer, daemon=True)
    #t.start()
    print(QtCore.QT_VERSION_STR)

    app.exec_()
