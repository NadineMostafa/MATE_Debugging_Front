from rclpy.node import Node
from std_msgs.msg import String
from PyQt6.QtCore import QObject, pyqtSignal


class GUI_ROSNode(Node,QObject):

    data_received=pyqtSignal(str)
    def __init__(self):
        Node.__init__(self,'GUI_ROSNode')
        QObject.__init__(self)
        self.subscription = self.create_subscription(
            String,
            'topic_1',
            self.listener_callback,
            10)
        
        self.publisher=self.create_publisher(String,'topic_2',10)

    def publish_message(self,gui_data:int):
         msg=String()
         msg.data=str(gui_data)
         self.publisher.publish(msg)

    def listener_callback(self, msg):
            self.data_received.emit(msg.data)
