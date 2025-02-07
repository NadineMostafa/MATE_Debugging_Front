import rclpy
from GUI_ROSNode import GUI_ROSNode
from PyQt6.QtWidgets import QWidget, QLabel,QHBoxLayout,QPushButton
from PyQt6.QtCore import QThread, pyqtSignal


class MainWindow(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        self.initUI()

    def initUI(self):
        self.setGeometry(0, 0, 600, 400)
        self.setWindowTitle("ROV Electrical Debugging")
        self.initLabels()

    def initLabels(self):
        self.label = QLabel(self)
        self.label.setText("I am here")
        self.reset_button=QPushButton("reset",self)
        self.reset_button.clicked.connect(lambda:self.reset_data(1))
        layout = QHBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.reset_button)
        self.setLayout(layout)


        self.ros_thread = ROS2Thread()
        self.ros_thread.signal_received.connect(self.update_label)  
        self.ros_thread.start()
    
    def reset_data(self,button_number:int):
        self.label.setText("data reseted")
        self.ros_thread.node.publish_message(button_number)
    
    def update_label(self, message):
        self.label.setText(f"{message}")


class ROS2Thread(QThread):    
    signal_received = pyqtSignal(str)  

    def run(self):
        """Run ROS 2 node in a separate thread."""
        rclpy.init()
        self.node = GUI_ROSNode()
        self.node.data_received.connect(self.signal_received.emit)  
        rclpy.spin(self.node)  
        self.node.destroy_node()
        rclpy.shutdown()
