from PyQt5 import QtWidgets
from PyQt5.QtGui import QStandardItem
from PyQt5.QtGui import QStandardItemModel
from PyQt5.QtCore import QModelIndex
from PyQt5.QtCore import Qt
from PyQt5.QtCore import pyqtSlot
 
# Импортируем нашу форму.
from server_gui import Ui_MainWindow
import sys
 
 
class main_window(QtWidgets.QMainWindow):
    def __init__(self):
        super(main_window, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        model = QStandardItemModel()
        item = QStandardItem()
        model.setHorizontalHeaderLabels(
            ('copter ID', 'animation ID', 'battery V', 'battery %', 'selfcheck', 'time UTC')
        )
        model.setColumnCount(6)
        model.setRowCount(20)
        self.ui.tableView.setModel(model)
        self.ui.tableView.horizontalHeader().setStretchLastSection(True)
 
 
app = QtWidgets.QApplication([])
application = main_window()
application.show()
 
sys.exit(app.exec())