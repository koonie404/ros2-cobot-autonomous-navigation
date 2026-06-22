import array
import math
import time
import threading
import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from analysis_ex import AnalysisOne, Parse, GetScanData, GetScanBufferSize

PORT = '/dev/ttyACM0'
BAUD = 230400

STREAM_BUF_SIZE = 1024
PACKETS_PER_REV = 56   # 1회전당 패킷 수 (667 / 12 ≈ 56)
SENSOR_HZ = 6

class LidarPublisher(Node):

    def __init__(self):
        super().__init__('lidar_publisher')

        self._scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self._scan_msg = LaserScan()
        self.SCAN_BUFFER_SIZE = GetScanBufferSize()

        self._ser        = serial.Serial(PORT, BAUD, timeout=0)
        self._stream_buf = bytearray(STREAM_BUF_SIZE)
        self._parse_cnt  = 0
        self._running    = False

        self._initialize_scan()
        self.get_logger().info(f'LD14P LiDAR publisher started on {PORT}')

    # ── 초기화 (C++ initialize_scan 대응) ─────────────────────────────────────
    def _initialize_scan(self):
        self._scan_msg.header.frame_id = 'laser_frame'

        self._scan_msg.angle_min = -math.pi

        self._scan_msg.angle_increment = (
            2.0 * math.pi / self.SCAN_BUFFER_SIZE
        )

        self._scan_msg.angle_max = (
            self._scan_msg.angle_min
            + self._scan_msg.angle_increment
            * (self.SCAN_BUFFER_SIZE - 1)
        )

        self._scan_msg.scan_time      = 1.0 / SENSOR_HZ
        self._scan_msg.time_increment = self._scan_msg.scan_time / self.SCAN_BUFFER_SIZE

        self._scan_msg.range_min = 0.15
        self._scan_msg.range_max = 8.0

        self._scan_msg.ranges      = [0.0] * self.SCAN_BUFFER_SIZE
        self._scan_msg.intensities = [0.0] * self.SCAN_BUFFER_SIZE
        
        # 초기화용 zero 버퍼 (재사용)
        self._zero_buf = array.array('f', [0.0] * self.SCAN_BUFFER_SIZE)

    # ── 발행 (C++ publish_scan 대응) ──────────────────────────────────────────
    def _publish_scan(self):
        ranges, intensities = GetScanData()

        self._scan_msg.header.stamp = self.get_clock().now().to_msg()

        self._scan_msg.ranges[:]      = ranges
        self._scan_msg.intensities[:] = intensities

        self._scan_pub.publish(self._scan_msg)

        # 버퍼 초기화 (C++ std::fill 대응)
        self._scan_msg.ranges[:]      = self._zero_buf
        self._scan_msg.intensities[:] = self._zero_buf

    # ── 시리얼 루프 (C++ while(running_) 대응) ────────────────────────────────
    def _serial_loop(self):
        while self._running:
            avail = self._ser.in_waiting
            while avail > 0:
                to_read = min(avail, STREAM_BUF_SIZE)
                n = self._ser.readinto(memoryview(self._stream_buf)[:to_read])
                if n > 0:
                    for i in range(n):
                        if AnalysisOne(self._stream_buf[i]):
                            Parse()
                            self._parse_cnt += 1
                            if self._parse_cnt >= PACKETS_PER_REV:
                                self._parse_cnt = 0
                                self._publish_scan()
                avail = self._ser.in_waiting

            time.sleep(0.001)   # C++ usleep(1000) 대응

    def start(self):
        self._running = True
        self._thread  = threading.Thread(target=self._serial_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        self._thread.join()
        self._ser.close()


# ── 엔트리포인트 ──────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = LidarPublisher()
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()