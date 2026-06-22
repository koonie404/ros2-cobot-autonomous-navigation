import serial
import time

ser = serial.Serial('/dev/ttyACM0', 230400)

time.sleep(1)

try:
    while True:
        data = ser.read(1024)
        print(data)

except KeyboardInterrupt:
    ser.close()
    
# 라이다 데이터 읽어보기
# dtoverlay -a | grep uart
# dtoverlay -h uart3-pi5
#  Info:   Enable uart 3 on GPIOs 8-9. Pi 5 only.
# sudo nano /boot/firmware/config.txt
# dtoverlay=uart3-pi5 맨 아래 추가(rpi5)
# 재부팅 후, /dev/ttyAMA3(rpi5) 확인
# pinctrl get 8,9
#  8: a2    pn | hi // GPIO8 = TXD3
#  9: a2    pu | hi // GPIO9 = RXD3
# LD14P TX -> GPIO9(Pin 24, UART3 RX)
# LD14P RX -> GPIO8(Pin 21, UART3 TX)
# sudo stty -F /dev/ttyAMA3 230400 raw -echo
# apt install bsdmainutils -y
# sudo cat /dev/ttyAMA3 | hexdump -C

