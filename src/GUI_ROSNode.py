
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16


class GUINode(Node):
    def __init__(self):
        super().__init__('GUINode')
        self.publisher_ = self.create_publisher(Int16, 'reset', 10)

    def clicked(self, msg = 0):
        flag = Int16()
        flag.data = msg
        self.publisher_.publish(flag)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
