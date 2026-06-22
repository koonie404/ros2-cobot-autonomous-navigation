import serial
import time

PORT = '/dev/ttyACM0'   # 환경에 맞게 수정
BAUD = 230400

ser = serial.Serial(PORT, BAUD, timeout=0)

time.sleep(1)

cnt_byte = 0
last_time = time.time()

try:
    while True:
        # 1. 수신된 데이터 읽기
        data = ser.read(1024)
        cnt_byte += len(data)

        # 2. 1초마다 출력
        current_time = time.time()
        if current_time - last_time >= 1.0:
            print(f"Bytes per second: {cnt_byte}")
            cnt_byte = 0
            last_time = current_time

except KeyboardInterrupt:
    ser.close()

# 초당 거리 데이터 수신 바이트 수 확인
# top 명령으로 cpu 점유율 확인 - 90% 이상 점유