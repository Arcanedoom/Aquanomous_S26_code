import serial
import time
import sys

serialConnected = 0

# connect ser depending on which system the program is currently running on
if sys.platform == 'linux':
    ser = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=5)
    serialConnected = 1
elif sys.platform == 'windows':
    ser = serial.Serial('COM6', baudrate=9600, timeout=5)
    serialConnected = 1


while serialConnected:
    # tbyte = '2'
    # ser.write(tbyte.encode('utf-8'))
    # print('write: ', ord(tbyte))
    b = ser.read(4)
    if((b[0] > 50) & (b[0] < 100)):
        print("Throttle: ", b[0], ", Forward")
    if(b[0] < 50):
        print("Throttle: ", b[0], ", Backward")
    if(b[0] > 100):
        print("Throttle: ", b[0], ", KILLSWITCH")
    if((b[2] > 50) & (b[2] < 100)):
        print("Steering: ", b[2], ", Right")
    if(b[2] < 50):
        print("Steering: ", b[2], ", Left")
    if(b[2] > 100):
        print("Throttle: ", b[0], ", KILLSWITCH")
    print('read: ', b)
    # time.sleep(0.01)

if(serialConnected == 0):
    print('Serial failed to connect')

print('exiting')