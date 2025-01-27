import sys
from PyQt6 import QtWidgets

app = QtWidgets.QApplication(sys.argv)
widget = QtWidgets.QWidget()
widget.resize(300, 200)
widget.setWindowTitle("Hello, Elaine")
widget.show()
sys.exit(app.exec())
