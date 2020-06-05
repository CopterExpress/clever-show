from PyQt5.QtCore import pyqtSlot
from PyQt5.QtGui import QKeySequence
from PyQt5 import QtWidgets

import modules.ui.visual_land as visual_land
import math
import logging
import sys
from functools import partial

from lib import b_partial

#  TODO: previous step and reset
class VisualLandDialog(QtWidgets.QDialog):
    def __init__(self, model):
        super(VisualLandDialog, self).__init__()

        self.ui = visual_land.Ui_Dialog()
        self.setupUi()

        self.model = model
        self.row_min = 0
        self.row_max = self.model.rowCount() - 1
        self._finished = False

    def setupUi(self):
        self.ui.setupUi(self)
        self.ui.one_button.clicked.connect(partial(self.selection_choice, 1))
        self.ui.two_button.clicked.connect(partial(self.selection_choice, 2))
        self.ui.land_emergency_button.clicked.connect(b_partial(self.send_to_selected, "land"))
        self.ui.disarm_emergency_button.clicked.connect(b_partial(self.send_to_selected, "disarm"))

        self.ui.one_button.setShortcut(QKeySequence("1"))
        self.ui.two_button.setShortcut(QKeySequence("2"))
        self.ui.land_emergency_button.setShortcut(QKeySequence("L"))
        self.ui.disarm_emergency_button.setShortcut(QKeySequence("D"))

    @property
    def row_mid(self):
        return int(math.ceil((self.row_min + self.row_max) / 2.0))

    def send_to_row(self, row, message, args=(), kwargs=None):
        logging.debug(f"Send {message}: {args}, {kwargs} to {row}")
        self.model.data_contents[row].client.send_message(message, args=args, kwargs=kwargs)
        # test[row] = args, kwargs  # for testing
        # print(test)

    def clear_leds(self, rows):
        for row in rows:
            self.send_to_row(row, "led_fill")

    def start(self):
        self.show()
        self.send_led_indication()

        self.exec()

    def send_led_indication(self):
        for row in range(self.row_min, self.row_mid):
            self.send_to_row(row, "led_fill", kwargs={"green": 255})

        for row in range(self.row_mid, self.row_max + 1):
            self.send_to_row(row, "led_fill", kwargs={"red": 255})

    @pyqtSlot()
    def selection_choice(self, choice):
        if self.row_min == self.row_max:
            # self.ui.one_button.setDisabled(True)  # maybe?
            # self.ui.two_button.setDisabled(True)
            self.send_to_selected("land")
            return

        if choice == 1:
            to_clear = range(self.row_mid, self.row_max + 1)
            self.row_max = self.row_mid - 1

        elif choice == 2:
            to_clear = range(self.row_min, self.row_mid)
            self.row_min = self.row_mid

        else:
            return

        self.clear_leds(to_clear)
        self.send_led_indication()

    @pyqtSlot()
    def send_to_selected(self, message, args=(), kwargs=None):
        for row in range(self.row_min, self.row_max + 1):
            self.send_to_row(row, message, args, kwargs)

        self._finished = True
        self.close()

    def closeEvent(self, event):
        if not self._finished:
            self.clear_leds(range(self.row_min, self.row_max + 1))

        event.accept()


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    app = QtWidgets.QApplication(sys.argv)

    import copter_table_models
    model = copter_table_models.CopterDataModel()
    for i in range(10):
        model.add_client()

    dialog = VisualLandDialog(model)
    test = list(range(10))

    dialog.start()
