import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

class Follower(Node):
    def __init__(self):
        super().__init__('follower')
        self.dir = 0.0
        self.dep = None
        self.length = 0.0
        self.spins =  0.0

        self.subscription2 = self.create_subscription(Float32, 'stepper', self.set_length, 10)
        self.subscription3 = self.create_subscription(Float32, 'subm', self.depth, 10)

        # publishes data to t200
        self.boost = self.create_publisher(Float32, 'follow', 10)
        self.create_timer(0.2, self.boost_pub)

    def set_length(self, msg):
        self.dir = msg.data

    def depth(self,msg):
        self.dep = -msg.data

    def boost_pub(self):
        bst = 0.0

        # Update spin and length
        self.spins += (self.dir / 60.0) * 0.2
        if self.spins <= 0:
            self.spins = 0

        if abs(self.spins) > 23.125:
            n = int(self.spins / 23.125)
        else:
            n = 0

        partial_spins = self.spins % 23.125
        self.length = 2.0 * partial_spins * 3.14 * (0.095 + 0.0076 / 2.0 + (4 - n) * 0.0076)

        if n > 0:
            for i in range(n):
                self.length += 2.0 * 23.125 * 3.14 * (0.095 + 0.0076 / 2.0 + (4 - i) * 0.0076)
        dep_c = self.length/3 + self.length/3 * math.sin(45)
        # Adds the correct thrust to follow sub
        if self.dep != None and self.dep >= 0:
            if dep_c > self.dep * 1.05:
                bst = -50.0
            elif dep_c < self.dep * 0.95:
                bst = 50.0
            else:
                bst = 0.0
        else:
            bst = 0.0
        self.get_logger().info(f"Depth Calculated (m): {dep_c:.2f} Boost: {bst:.2f}")
        msg = Float32()
        msg.data = bst
        self.boost.publish(msg)