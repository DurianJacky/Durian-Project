from math import cos, sin, pi
import struct
import serial
import re
import time

controller_arduino_port = '/dev/tty.usbserial-143340'  # Adjust these paths as needed
# motor_RS485_Left_port = '/dev/tty.usbmodem141201'
motor_RS485_Right_port = '/dev/tty.usbserial-143330'
motor_PWM_port = '/dev/tty.usbserial-143320'


ser_controller = serial.Serial(controller_arduino_port, baudrate = 115200, timeout = 1) 
# ser_485_left = serial.Serial(motor_RS485_Left_port, baudrate = 38400, timeout = 1) 
ser_485_right = serial.Serial(motor_RS485_Right_port, baudrate = 38400, timeout = 1) 
ser_PWM = serial.Serial(motor_PWM_port, baudrate = 115200, timeout = 1) 

# time.sleep(3)

w = 243.8 * 10 # unit: mm
l = 485.1 * 10 # unit: mm
theta = pi/4

def getControllerValues():
    if ser_controller.is_open > 0:
        arduinoData = ser_controller.readline().decode('ascii')

    # 使用正则表达式提取数据
    rx_match = re.search(r'Rx:\s*([-+]?\d*\.\d+|\d+)', arduinoData)
    ry_match = re.search(r'Ry:\s*([-+]?\d*\.\d+|\d+)', arduinoData)
    e_stop_match = re.search(r'E-stop:\s*(\d+)', arduinoData)
    e_x_match = re.search(r'e_x:\s*([-+]?\d*\.\d+|\d+)', arduinoData)
    e_y_match = re.search(r'e_y:\s*([-+]?\d*\.\d+|\d+)', arduinoData)
    e_psi_match = re.search(r'e_psi:\s*([-+]?\d*\.\d+|\d+)', arduinoData)

    # 将匹配的结果赋值给变量，如果没有匹配到则赋值为None
    rx = float(rx_match.group(1)) if rx_match else None
    ry = float(ry_match.group(1)) if ry_match else None
    e_stop = bool(int(e_stop_match.group(1))) if e_stop_match else None
    e_x = float(e_x_match.group(1)) if e_x_match else None
    e_y = float(e_y_match.group(1)) if e_y_match else None
    e_psi = float(e_psi_match.group(1)) if e_psi_match else None
    data = [rx, ry, e_stop, e_x, e_y, e_psi]
    return data

def getMotion(controllerData):
    e_x = controllerData[3]
    e_y = controllerData[4]
    e_psi = controllerData[5]
    if e_x != 0 or e_y != 0:
        if e_x != 0 and e_y != 0:
            return "xy"
        elif e_x != 0:
            return "x"
        else:
            return "y"
    elif e_psi != 0:
        return "psi"  

def xMotionK(Kx, l, w, theta):
    K1 = -(l*cos(theta) - w*sin(theta)/2) / (w*cos(theta)) * Kx
    K2 = -K1
    K3 = -1 / (2*cos(theta)) * Kx
    K4 = -K3
    K = [K1, K2, K3, K4]
    return K

def yMotionK(Ky, l, w, theta):
    K1 = -Ky/(cos(2*theta) - 3)
    K2 = K1
    K3 = (Ky*sin(theta))/(2*sin(theta)**2 + 2)
    K4 = K3
    # K1 = Ky/2
    # K2 = K1
    # K3 = 0
    # K4 = 0
    K = [K1, K2, K3, K4]
    return K

def mMotionK(Km, l, w, theta):
    K1 = -1/w * Km
    K2 = -K1
    K3 = 0
    K4 = 0
    K = [K1, K2, K3, K4]
    return K

def getKx(proportion, e_x_max, l, w, theta):
    
    Kx1 = w*cos(theta) / (l*cos(theta) - w*sin(theta)/2)  / e_x_max * proportion
    Kx2 = (2*cos(theta)) / e_x_max * proportion * 133/80

    return min([Kx1, Kx2])

def getKy(proportion, e_y_max, l, w, theta):

    Ky1 = -(cos(2*theta) - 3) / e_y_max * proportion
    Ky2 = (2*sin(theta)**2 + 2) / (sin(theta)) / e_y_max * proportion * 133/80

    return min([Ky1, Ky2])

def getKm(proportion, e_m_max, l, w, theta):
    Km = w / e_m_max * proportion
    return Km

def thrustAllocation(controllerData, l, w, theta):
    e_stop = controllerData[2]
    e_x = controllerData[3]
    e_y = controllerData[4]
    e_psi = controllerData[5]

    if e_stop == 1:
        T_rear_left = 0 # P1
        T_rear_right = 0 # P2
        T_front_left = 0 # P3
        T_front_right = 0 # P4
    else:
        motion = getMotion(controllerData)
        if motion == 'x':
            Kx = getKx(80, 100, l, w, theta) # 80% 
            Kx_n = xMotionK(Kx, l, w, theta)

            T_rear_left = e_x * Kx_n[0] # P1
            T_rear_right = e_x * Kx_n[1] # P2
            T_front_left = e_x * Kx_n[2] # P3
            T_front_right = e_x * Kx_n[3] # P4

        elif motion == 'y':
            Ky = getKy(80, 100, l, w, theta) # 80% 
            Ky_n = yMotionK(Ky, l, w, theta)

            T_rear_left = e_y * Ky_n[0] # P1
            T_rear_right = e_y * Ky_n[1] # P2
            T_front_left = e_y * Ky_n[2] # P3
            T_front_right = e_y * Ky_n[3] # P4

        elif motion == 'psi':
            Km = getKm(80, 100, l, w, theta) # 80% 
            Km_n = mMotionK(Km, l, w, theta)

            T_rear_left = e_psi * Km_n[0] # P1
            T_rear_right = e_psi * Km_n[1] # P2
            T_front_left = e_psi * Km_n[2] # P3
            T_front_right = e_psi * Km_n[3] # P4

        else:
            Kx = getKx(30, 100, l, w, theta) # 30% 
            Kx_n = xMotionK(Kx, l, w, theta)
            Ky = getKy(30, 100, l, w, theta) # 30% 
            Ky_n = yMotionK(Ky, l, w, theta)
            Km = getKm(30, 100, l, w, theta) # 30% 
            Km_n = mMotionK(Km, l, w, theta)

            T_rear_left = e_x * Kx_n[0] + e_y * Ky_n[0] + e_psi * Km_n[0]
            T_rear_right = e_x * Kx_n[1] + e_y * Ky_n[1] + e_psi * Km_n[1]
            T_front_left = e_x * Kx_n[2] + e_y * Ky_n[2] + e_psi * Km_n[2]
            T_front_right = e_x * Kx_n[3] + e_y * Ky_n[3] + e_psi * Km_n[3]
    
    thrust_percent = [T_rear_left, T_rear_right, T_front_left * 133/80, T_front_right * 133/80]
    return thrust_percent

def pct2analog(input):
    for i in range(len(input)):
        input[i] = input[i] / 100 * 127
    return input

# def sendData(thrusts, ser_485_left, ser_485_right, ser_PWM):
def sendData(thrusts, ser_485_left, ser_PWM):
    data = []
    for thrust in thrusts:
        if thrust > 0:
            direction = True
        else:
            direction = False
        
        

        velocity = abs(thrust)
        # 将布尔值转换为字节（True -> 1, False -> 0）
        direction_byte = struct.pack('B', direction)

        # 将浮点数转换为4字节的IEEE 754格式
        velocity_bytes = struct.pack('B', velocity)
    
        # 合并两个字节数组
        data.append(direction_byte + velocity_bytes)

    # 发送数据
    ser_485_left.write(data[0])
    # ser_485_right.write(data[1])
    ser_PWM.write(data[2] + data[3])
    
    



while 1:
    
    # 打印
    controllerData = getControllerValues()
    
    if controllerData[0] != None:

        print(controllerData)

        thrusts_percent = thrustAllocation(controllerData, l, w, theta)
        print(thrusts_percent)

        thrusts_percent = pct2analog(thrusts_percent)

        thrusts_percent_round = [round(thrust) for thrust in thrusts_percent]
        print(thrusts_percent_round)

        sendData(thrusts_percent_round, ser_485_right, ser_PWM)









    



