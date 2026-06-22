import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
import tf_transformations
from tf2_ros import TransformBroadcaster

class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.sub = self.create_subscription(
            Int32MultiArray,
            'esp32s3/telemetry',
            self.telemetry_callback,
            qos
        )
        
        self.bat_pub = self.create_publisher(Float32, 'battery_voltage', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 로봇 파라미터
        self.TICKS_PER_REV = 7*4*298      # 엔코더 해상도
        self.ENCODER_WRAP = 2**31      # signed int32 wraparound range
        self.WHEEL_RADIUS = 0.0325      # 바퀴 반지름 (m), 0.066/2        
        self.WHEEL_CIRCUMFERENCE = (2 * math.pi * self.WHEEL_RADIUS) 
        self.DEG_TO_RAD = math.pi / 180.0
        self.GYRO_DEADZONE_RAD = 0.01 # rad/s (약 0.57 deg/s)       

        # 상태 변수
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # 이전 타임스탬프 저장용
        self.prev_time = None
        
        self.prev_left_pos = None
        self.prev_right_pos = None

        self.get_logger().info('Odometry Publisher Started')

        # 메시지 재사용
        self.bat_msg = Float32()
        self.joint_msg = JointState()
        self.odom_msg = Odometry()
        self.tf_msg = TransformStamped()

        # 고정 메시지 초기화
        self.joint_msg.name = [
            'left_wheel_joint',
            'right_wheel_joint'
        ]
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = (
            'base_footprint'
        )
         # 대각선에 적절한 값 설정 - Nav2가 odom을 신뢰하도록
        self.odom_msg.pose.covariance[0] = 0.1   # x
        self.odom_msg.pose.covariance[7] = 0.1   # y
        self.odom_msg.pose.covariance[35] = 0.2  # yaw
        self.odom_msg.twist.covariance[0] = 0.1  # linear
        self.odom_msg.twist.covariance[35] = 0.2 # angular

        self.tf_msg.header.frame_id = 'odom'
        self.tf_msg.child_frame_id = (
            'base_footprint'
        )

        self.get_logger().info(
            'Odometry Publisher Started'
        )

    def telemetry_callback(self, msg):
        # Expected:
        # [left_pos, right_pos,
        #  left_vel, right_vel,
        #  gyro_z, battery]
        if len(msg.data) != 6:
            self.get_logger().warn("Expected 6 telemetry values")
            return

        (
            left_pos,
            right_pos,
            left_vel,
            right_vel,
            gyro_deg_s_100,
            bat_voltage_100
        ) = msg.data     
        
        now = self.get_clock().now()
        stamp = now.to_msg()

        # self.get_logger().info(
        #         f'esp32s3/telemetry: {msg.data}',
        #         throttle_duration_sec=0.1
        # )

        # 1. 최초 시간, 위치 초기화
        if self.prev_time is None:
            self.prev_time = now            
            self.prev_left_pos = float(left_pos)
            self.prev_right_pos = float(right_pos)
            
            self.get_logger().info("First telemetry received")
            return

        # 2. 단위 변환
        left_rpm = float(left_vel) / 100.0   # rpm
        right_rpm = float(right_vel) / 100.0 # rpm
        gyro_deg_s = float(gyro_deg_s_100) / 100.0 # deg/s
        battery_voltage = float(bat_voltage_100) / 100.0 # voltage
        
        # 3. 배터리 전압 발행
        self.bat_msg.data = battery_voltage
        self.bat_pub.publish(self.bat_msg)
        
        # 4. 시간 차 계산
        dt = (
            now.nanoseconds - self.prev_time.nanoseconds
        ) / 1e9
        self.prev_time = now        
        # 비정상 dt 처리 (음수, 0, 1초 초과)
        if dt <= 0 or dt > 1.0:
            self.get_logger().warn(f"Invalid dt: {dt:.6f}")
            return
        
        # 5. 엔코더 차 계산(오버플로우 보정)
        delta_left = float(left_pos) - self.prev_left_pos        
        delta_right = float(right_pos) - self.prev_right_pos
        if delta_left > self.ENCODER_WRAP: delta_left -= 2*self.ENCODER_WRAP
        elif delta_left < -self.ENCODER_WRAP: delta_left += 2*self.ENCODER_WRAP
        if delta_right > self.ENCODER_WRAP: delta_right -= 2*self.ENCODER_WRAP
        elif delta_right < -self.ENCODER_WRAP: delta_right += 2*self.ENCODER_WRAP
        self.prev_left_pos = float(left_pos)
        self.prev_right_pos = float(right_pos)        
        
        # 6-1. 자이로 계산(rad/s)
        gyro_rad_s = gyro_deg_s * self.DEG_TO_RAD # rad        
        if abs(gyro_rad_s) < self.GYRO_DEADZONE_RAD: # Noise 제거
            gyro_rad_s = 0.0
        
        # 6-2. 바퀴 속도 계산(m/s)
        left_speed = (left_rpm / 60.0) * self.WHEEL_CIRCUMFERENCE
        right_speed = (right_rpm / 60.0) * self.WHEEL_CIRCUMFERENCE
        linear_velocity = (left_speed + right_speed) / 2.0
        
        # 7-1. 이동거리 계산
        left_dist = (delta_left / self.TICKS_PER_REV) * self.WHEEL_CIRCUMFERENCE
        right_dist = (delta_right / self.TICKS_PER_REV) * self.WHEEL_CIRCUMFERENCE       
        dist = (left_dist + right_dist) / 2.0
        delta_yaw = gyro_rad_s*dt
        
        # 7-2. 위치 갱신
        self.x += dist * math.cos(self.yaw + delta_yaw / 2.0)
        self.y += dist * math.sin(self.yaw + delta_yaw / 2.0)
        self.yaw += delta_yaw
        # 장시간 운용 시 yaw가 무한정 증가할 수 있음 -> 정규화 추가
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # Quaternion 계산
        half_yaw = self.yaw * 0.5
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)
        
        # 8. joint_states (odom 계산 후 발행)
        left_rad = (float(left_pos) / self.TICKS_PER_REV) * 2.0 * math.pi
        right_rad = (float(right_pos) / self.TICKS_PER_REV) * 2.0 * math.pi
        
        left_rad_s = left_rpm * 2.0 * math.pi / 60.0
        right_rad_s = right_rpm * 2.0 * math.pi / 60.0
        
        self.joint_msg.header.stamp = stamp
        self.joint_msg.position = [left_rad, right_rad]
        self.joint_msg.velocity = [left_rad_s, right_rad_s] # Nav2/SLAM 정확도 향상을 위해 추가 권장
        self.joint_pub.publish(self.joint_msg)
                
        # 9. Odometry
        self.odom_msg.header.stamp = stamp
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y 
        self.odom_msg.pose.pose.position.z = 0.0
        self.odom_msg.pose.pose.orientation = Quaternion(
                x=0.0,
                y=0.0,
                z=qz,
                w=qw
        )
        self.odom_msg.twist.twist.linear.x = linear_velocity
        self.odom_msg.twist.twist.angular.z = gyro_rad_s
        self.odom_pub.publish(self.odom_msg)

        # 10. TF 메시지 발행
        # TF(Transform) 트리는 로봇의 뼈대를 정의하는 계층 구조
        # map -> odom -> base_footprint -> base_link -> laser_frame
        # 최종적인 TF 데이터 흐름 (Chain)
        # 이렇게 설정하면 시스템 전체의 TF는 다음과 같이 흐름
        # Map → Odom: SLAM 노드(가장 상위)가 발행
        # Odom → base_footprint: 대표님의 노드가 엔코더/IMU로 계산해서 발행
        # base_footprint → base_link: URDF(또는 static_tf)가 바퀴 반지름만큼 띄워서 고정 발행
        # base_link → laser_frame: URDF가 라이다 위치에 고정 발행
        self.tf_msg.header.stamp = stamp
        self.tf_msg.transform.translation.x = self.x
        self.tf_msg.transform.translation.y = self.y
        self.tf_msg.transform.translation.z = 0.0
        self.tf_msg.transform.rotation = self.odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(
            self.tf_msg
        )

def main():
    rclpy.init()
    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# telemetry 수신
#     ↓
# 1. 최초 초기화 (시간 + 엔코더)
#     ↓
# 2. 단위 변환 (rpm, deg/s, voltage)
#     ↓
# 3. 배터리 발행
#     ↓
# 4. dt 계산 + 비정상 체크
#     ↓
# 5. 엔코더 차이 + 오버플로우 보정
#     ↓
# 6. 자이로(rad/s) + 속도(m/s) 계산
#     ↓
# 7. 위치 갱신 + yaw 정규화
#     ↓
# 8. joint_states 발행
#     ↓
# 9. odom 발행
#     ↓
# 10. TF 발행
# 
# ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
# ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.05 -p turn:=0.2
# ros2 run tf2_tools view_frames
# pdf 파일 받아서 확인
# rviz2 > Add > TF
# Fixed Frame > odom
# rviz2에서 base_footprint가 전방(+x 방향)으로 가는지,
# 왼쪽으로 돌렸을 때 반시계 방향(+yaw)으로 도는지 확인 
# 바닥에 놓고 teleop_twist_keyboard로 조종해 보기
# +X 방향 : 빨간 화살표 방향 
# rviz2의 격자는 1m
# map
#  └── odom
#       └── base_footprint
#             └── base_link

