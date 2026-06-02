import rclpy
from rclpy.executors import MultiThreadedExecutor
from ENCODER import encoder
from follower import Follower
from imu import IMU
from GPS import GPS
from logger import Logger

def main(args=None):
    rclpy.init(args=args)
    enc = encoder()
    gps = GPS()
    log = Logger()
    flw = Follower()

    # runs multipole nodes in parallel
    executor = MultiThreadedExecutor()
    executor.add_node(enc)
    executor.add_node(gps)
    executor.add_node(log)
    executor.add_node(flw)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        enc.destroy_node()
        gps.destroy_node()
        log.destroy_node()
        flw.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()