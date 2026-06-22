import serial
import time

PORT = '/dev/ttyACM0'
BAUD = 230400

ser = serial.Serial(PORT, BAUD, timeout=0)

time.sleep(1)

STREAM_BUF_SIZE = 1024
stream_buf = bytearray(STREAM_BUF_SIZE)
cnt_byte = 0

last_time = time.time()

# =========================
# 메인 루프
# =========================
try:
    while True:
        avail = ser.in_waiting
        while avail > 0:
            to_read = min(avail, STREAM_BUF_SIZE)
            n = ser.readinto(memoryview(stream_buf)[:to_read])
#             print(avail, n)
            
            cnt_byte += n
            avail = ser.in_waiting
        else: time.sleep(0.001) # CPU 과부하 방지

        # 1초마다 출력
        now = time.time()
        if now - last_time >= 1:
            print(f"Bytes per second: {cnt_byte}")
            cnt_byte = 0
            last_time = now

except KeyboardInterrupt:
    ser.close()

# CPU 과부하 방지: time.sleep(0.001)
# top 명령으로 cpu 점유율 확인 - 10% 전후로 낮아짐