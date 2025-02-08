import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from rov_debug_interfaces.msg import DecodedData

from PyQt5.QtCore import pyqtSignal, QObject

class SignalSender(QObject):
    custom_signal = pyqtSignal(DecodedData)


class GUINode(Node):
    def __init__(self):
        super().__init__('GUINode')

        self.publisher_ = self.create_publisher(Int16, 'topic', 10)

        self.subscription = self.create_subscription(
            DecodedData,
            'recieved',
            self.listener_callback,
            10)
        self.subscription 
        
        self.signal_emmitter = SignalSender()



    def clicked(self, msg = 0):
        flag = Int16()
        flag.data = msg
        self.publisher_.publish(flag)


    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.id)
        self.signal_emmitter.custom_signal.emit(msg)


def main(args=None):
    rclpy.init(args=args)

    gui_node = GUINode()

    rclpy.spin(gui_node)

    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
