import serial
import time

ser = serial.Serial('COM6', baudrate=9600, timeout=5)


while True:
    # tbyte = '2'
    # ser.write(tbyte.encode('utf-8'))
    # print('write: ', ord(tbyte))
    b = ser.read(4)
    if(b[0] > 50):
        print("Throttle: ", b[0], ", Forward")
    if(b[0] < 50):
        print("Throttle: ", b[0], ", Backward")
    if(b[2] > 50):
        print("Steering: ", b[2], ", Right")
    if(b[2] < 50):
        print("Steering: ", b[2], ", Left")
    print('read: ', b)
    # time.sleep(0.01)