import rclpy
from rclpy.executors import MultiThreadedExecutor
from hx711 import TensionNode
from stepper import WinchMotorNode as wm
from ENCODER import encoder
from t200 import ThrusterNode
from imu import IMU
from GPS import GPS
from logger import Logger

def main(args=None):
    rclpy.init(args=args)
    hx = TensionNode()
    step = wm()
    t200 = ThrusterNode()
    imu = IMU()

    # runs multipole nodes in parallel
    executor = MultiThreadedExecutor()
    executor.add_node(hx)
    executor.add_node(step)
    executor.add_node(t200)
    executor.add_node(imu)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        hx.destroy_node()
        t200.destroy_node()
        imu.destroy_node()
        step.stop_motor()
        step.pi.stop()
        step.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()