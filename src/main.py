from PyQt5.QtWidgets import QApplication
from GUI_MainWindow import MainWindow
from threading import Thread
from GUI_ROSNode import GUINode
import rclpy


global ros_node

def ros_init():
    global ros_node
    rclpy.init()
    ros_node = GUINode()
    rclpy.spin(ros_node)

if __name__ == "__main__":
    ros_thread = Thread(target=ros_init)
    ros_thread.start()

    app = QApplication([])
    window = MainWindow()

    window.signal.signal_sender.connect(ros_node.clicked)

    window.show()
    app.exec_()