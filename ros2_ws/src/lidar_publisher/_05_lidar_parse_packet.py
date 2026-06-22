import serial
import time

from analysis_ex import AnalysisOne, Parse

PORT = '/dev/ttyACM0'
BAUD = 230400

ser = serial.Serial(PORT, BAUD, timeout=0)

STREAM_BUF_SIZE = 1024
stream_buf = bytearray(STREAM_BUF_SIZE)
cnt_byte = 0

parse_cnt = 0

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
            for i in range(n):
                if AnalysisOne(stream_buf[i]):
                    Parse();
                    parse_cnt+=1
            
            cnt_byte += n
            avail = ser.in_waiting
        else: time.sleep(0.001) # CPU 과부하 방지

        # 1초마다 출력
        now = time.time()
        if now - last_time >= 1:
            print(f"cnt_byte={cnt_byte}, parse_cnt={parse_cnt}")
            cnt_byte = 0
            parse_cnt = 0
            last_time = now

except KeyboardInterrupt:    
    ser.close()
    
# 초당 6개 - 6hz 회전
# 패킷 크기: 47 바이트 - 0x54|0x2C|LSL|LSH|SAL|SAH|MD(12*3B)|EAL|EAH|TSL|TSH|CRC
# 샘플 개수: 1회전당 667개 (0.54도)
