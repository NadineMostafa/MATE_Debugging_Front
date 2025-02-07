import rclpy
from GUI_ROSNode import GUI_ROSNode
from rov_debug_msgs.msg import DecodedData 
from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QScrollArea, 
    QGridLayout, QPushButton , QHBoxLayout 
)
from PyQt5.QtCore import QThread, pyqtSignal, Qt

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.ros_node = None  
        self.labels = {}  
        self.initUI()
        self.initROS()

    def initUI(self):
        self.setWindowTitle("ROV Electrical Debugging")
        self.setGeometry(100, 100, 1000, 800)

        layout = QVBoxLayout()
        scroll = QScrollArea()
        content = QWidget()
        grid = QGridLayout(content)
        row = 0

        self.add_label(grid, row, 0, "ID")
        self.add_label(grid, row, 1, "Valid")
        row += 1

        self.add_label(grid, row, 0, "IMU X")
        self.add_label(grid, row, 1, "IMU Y")
        self.add_label(grid, row, 2, "IMU Z")
        row += 1

        self.add_label(grid, row, 0, "IMU Roll")
        self.add_label(grid, row, 1, "IMU Pitch")
        self.add_label(grid, row, 2, "IMU Yaw")
        row += 1

        for i in range(1, 7):
            self.add_label(grid, row, 0, f"Thruster Current {i}")
            row += 1

        for i in range(1, 7):
            self.add_label(grid, row, 0, f"Thruster PWM {i}")
            row += 1

        for i in range(1, 7):
            self.add_label(grid, row, 0, f"Indicator {i}")
            row += 1

        for i in range(1, 5):
            self.add_label(grid, row, 0, f"Heartbeat {i}")
            row += 1

        for i in range(1, 5):
            self.add_label(grid, row, 0, f"Connection Percentage {i}")
            row += 1

        for i in range(1, 5):
            self.add_label(grid, row, 0, f"Arm {i}")
            row += 1

        self.add_label(grid, row, 0, "ROV Depth")

        scroll.setWidget(content)
        layout.addWidget(scroll)
        reset_layout = QHBoxLayout()

        for i in range(1, 5):
            btn = QPushButton(f"Reset {i}")
            btn.clicked.connect(lambda _, x=i: self.ros_node.publish_message(x))
            reset_layout.addWidget(btn)

        layout.addLayout(reset_layout)
        self.setLayout(layout)

    def add_label(self, grid, row, col, name):
        label = QLabel(f"{name}: --")
        grid.addWidget(label, row, col)
        self.labels[name] = label

    def initROS(self):
        self.ros_thread = ROS2Thread()
        self.ros_thread.node_ready.connect(self.on_ros_node_ready)
        self.ros_thread.start()

    def on_ros_node_ready(self, node):
        self.ros_node = node
        node.data_received.connect(self.update_gui)

    def update_gui(self, msg: DecodedData):
        self.labels["ID"].setText(f"ID: {msg.id}")
        self.labels["Valid"].setText(f"Valid: {msg.valid}")
        self.labels["IMU X"].setText(f"IMU X: {msg.imu.acc_x:.2f}")
        self.labels["IMU Y"].setText(f"IMU Y: {msg.imu.acc_y:.2f}")
        self.labels["IMU Z"].setText(f"IMU Z: {msg.imu.acc_z:.2f}")
        self.labels["IMU Roll"].setText(f"IMU Roll: {msg.imu.roll:.2f}")
        self.labels["IMU Pitch"].setText(f"IMU Pitch: {msg.imu.pitch:.2f}")
        self.labels["IMU Yaw"].setText(f"IMU Yaw: {msg.imu.yaw:.2f}")

        for i in range(1, 7):
            current = getattr(msg, f"thruster_current_{i}")
            self.labels[f"Thruster Current {i}"].setText(
                f"Thruster Current {i}: {current:.2f}A"
            )

        for i in range(1, 7):
            pwm = getattr(msg, f"thruster_pwm_{i}")
            self.labels[f"Thruster PWM {i}"].setText(
                f"Thruster PWM {i}: {pwm:.2f}"
            )

        for i in range(1, 7):
            indicator = getattr(msg, f"indicator_{i}")
            self.labels[f"Indicator {i}"].setText(
                f"Indicator {i}: {indicator:.2f}"
            )

        for i in range(1, 5):
            heartbeat = getattr(msg, f"heartbeat_{i}")
            self.labels[f"Heartbeat {i}"].setText(
                f"Heartbeat {i}: {'ON' if heartbeat else 'OFF'}"
            )

        for i in range(1, 5):
            connection = getattr(msg, f"connection_percentage_{i}")
            self.labels[f"Connection Percentage {i}"].setText(
                f"Connection Percentage {i}: {connection:.2f}%"
            )

        for i in range(1, 5):
            arm = getattr(msg, f"arm_{i}")
            self.labels[f"Arm {i}"].setText(
                f"Arm {i}: {arm:.2f}"
            )

        self.labels["ROV Depth"].setText(f"ROV Depth: {msg.rov_depth:.2f}m")

class ROS2Thread(QThread):
    node_ready = pyqtSignal(object) 

    def run(self):
        rclpy.init()
        self.node = GUI_ROSNode()
        self.node_ready.emit(self.node) 
        rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()
