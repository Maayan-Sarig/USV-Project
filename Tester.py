import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QProgressBar, QLabel
from PyQt5.QtCore import QTimer

class ROSFloatPublisher(Node):
    def __init__(self):
        super().__init__('Submarine')
        self.publisher = self.create_publisher(Float32, 'subm', 10)
        self.speed = [0.0, 0.0, 0.0]
        self.running = False
        self.start_time = 0
        self.dep = 0.0

    def set_start_time(self, start):
        self.start_time = start
        self.running = True
        self.phase = 0

    # Defines the Experiment beheviour
    def sys_scr(self, now, phase):
        if not self.running:
            return None
        speeds = []
        interval = 1  # default fallback
        elapsed = now - self.start_time

        # Starts the experiment algirithm
        if elapsed <= 5 and phase == 0:
            speeds = [0.0, 0.0, 0.0]
            interval = 5

        elif elapsed <= 10 and phase == 1:
            speeds = [1.5, 0.0, -1.0]
            interval = 5  # (35 - 5)

        elif elapsed <= 70 and phase == 2:
            speeds = [1.5, 0.0, 0.0]
            interval = 60

        elif elapsed <= 75 and phase == 3:
            speeds = [0.8, 0.7, 1.0]
            interval = 5

        elif elapsed <= 120 and phase == 4:
            speeds = [0.0, 0.0, 0.0]
            interval = 45

        return (speeds, interval)

    def try_publish(self):
        if not self.running:
            return None
        now = time.time()
        progress = []

        speeds, interval = self.sys_scr(now, self.phase)

        # Calculate per-phase start offset
        phase_offsets = [0, 5, 10, 70, 75]
        phase_start = self.start_time + phase_offsets[self.phase]
        phase_elapsed = now - phase_start
        progress_val = min(phase_elapsed / interval, 1.0)
        progress.append(progress_val)

        if progress_val >= 1.0:
            self.phase += 1
            progress_val = 0

        # Prepare publishing
        msg = Float32()
        self.dep += speeds[2] * 0.1
        self.speed = speeds
        msg.data = self.dep
        self.publisher.publish(msg)
        self.get_logger().info(f'Phase {self.phase} | Published: {msg.data}')
        
        return progress


class GUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.setWindowTitle("ROS2 Experiment")
        self.dep = 0

        layout = QVBoxLayout()

        self.label = QLabel("Press start to begin publishing.")
        layout.addWidget(self.label)

        self.start_button = QPushButton("Start")
        self.start_button.clicked.connect(self.start_publishing)
        layout.addWidget(self.start_button)

        # Speed labels for x, y, z
        self.speed_labels = []
        for i in range(3):
            speed_label = QLabel(f"Speed {i}: 0.0")
            layout.addWidget(speed_label)
            self.speed_labels.append(speed_label)

        # One overall progress bar at the bottom
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        layout.addWidget(self.progress_bar)

        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(100)  # every 100ms

    def start_publishing(self):
        self.node.set_start_time(time.time())
        self.label.setText("Publishing started!")

    def update_display(self):
        progress = self.node.try_publish()
        if progress:
            self.dep += self.node.speed[2] * 0.1
            self.speed_labels[2].setText(f"depth: {self.dep:.2f}")
            
            self.progress_bar.setValue(int(progress[0] * 100))

def main():
    rclpy.init()
    ros_node = ROSFloatPublisher()

    app = QApplication(sys.argv)
    gui = GUI(ros_node)
    gui.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    timer.start(20)  # Spin ROS events every 20ms

    sys.exit(app.exec_())

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()