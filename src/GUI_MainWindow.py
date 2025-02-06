from PyQt5.QtWidgets import QWidget, QLabel
from PyQt5.QtGui import QIcon

class MainWindow(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        self.initUI()

    def initUI(self):
        self.setGeometry(0, 0, 600, 400)
        self.setWindowTitle("ROV Electrical Debugging")
        self.setWindowIcon(QIcon('images\logo.jpg'))
        self.initLabels()

    def initLabels(self):
        label = QLabel(self)
        label.setText("I am here")
        
