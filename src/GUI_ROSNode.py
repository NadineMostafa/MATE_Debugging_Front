import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtCore import QObject, pyqtSignal


class GUI_ROSNode(Node,QObject):

    data_received=pyqtSignal(str)
    def __init__(self):
        Node.__init__('GUI_ROSNode')
        QObject.__init__(self)
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
            self.data_received.emit(msg.data)














