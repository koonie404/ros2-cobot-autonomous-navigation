# ESP32-S3 firmware

`esp32s3_telemetry`는 실제 시연의 Serial micro-ROS 구성과 대응합니다.

- Transport: UART Serial, 115200 baud
- ROS Domain ID: 36
- Subscribe: `/cmd_vel`
- Publish: `/esp32s3/telemetry`
- Telemetry payload: left/right encoder position, left/right RPM x100, Gyro Z x100, battery voltage x100

`libraries`에는 프로젝트에서 작성·수정한 모터, 엔코더, IMU, 차동구동 및 micro-ROS 노드 래퍼만 포함합니다. 다음 외부 라이브러리는 포함하지 않습니다.

- micro_ros_arduino
- Dynamixel2Arduino
- TFT_eSPI

