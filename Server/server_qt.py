from PyQt5 import QtWidgets
 
# Импортируем нашу форму.
from server_gui import Ui_MainWindow
import sys
 
 
class main_window(QtWidgets.QMainWindow):
    def __init__(self):
        super(main_window, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
 
        self.ui.tableWidget.setColumnCount(6)
        self.ui.tableWidget.setRowCount(20)
        self.ui.tableWidget.setHorizontalHeaderLabels(
            ('copter ID', 'animation ID', 'battery V', 'battery %', 'selfcheck', 'time UTC')
        )
        self.ui.tableWidget.horizontalHeader().setStretchLastSection(True)
 
 
app = QtWidgets.QApplication([])
application = main_window()
application.show()
 
sys.exit(app.exec())