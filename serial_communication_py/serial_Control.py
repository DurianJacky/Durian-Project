import serial
import struct
import re
import numpy as np
import time
import serial.tools.list_ports
from math import cos, sin

class SerialCommunication:
    """A class used for serial communication.

    Attributes:
        port (str): serial port.
        baudrate (int): baudrate for serial communication. (default: 38400)
        timeout (int): timeout for serial communication. (default: 1)

    Methods:
        read_data(): Returns the data received
        send_data(): send data through serial communication
        close(): close the serial port
    """
    def __init__(self, port, baudrate=38400, timeout=1):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)

    def read_data(self):
        if self.ser.is_open:
            return self.ser.readline().decode('ascii')
        return None

    def send_data(self, data):
        if self.ser.is_open:
            self.ser.write(data)
    
    def close(self):
        self.ser.close()

class ControllerDataParser:
    """ A utility class for parsing controller data.

    This class provides static methods for parsing controller data.
    All methods are static and can be called without instantiating the class.

    Methods:
        parse_controller_data(): Return a dictionary about controller data.
    """
    @staticmethod
    def parse_controller_data(arduino_data):
        rx_match = re.search(r'Rx:\s*([-+]?\d*\.\d+|\d+)', arduino_data)
        ry_match = re.search(r'Ry:\s*([-+]?\d*\.\d+|\d+)', arduino_data)
        e_stop_match = re.search(r'E-stop:\s*(\d+)', arduino_data)
        e_x_match = re.search(r'e_x:\s*([-+]?\d*\.\d+|\d+)', arduino_data)
        e_y_match = re.search(r'e_y:\s*([-+]?\d*\.\d+|\d+)', arduino_data)
        e_psi_match = re.search(r'e_psi:\s*([-+]?\d*\.\d+|\d+)', arduino_data)

        data = {
            "rx": float(rx_match.group(1)) if rx_match else None,
            "ry": float(ry_match.group(1)) if ry_match else None,
            "e_stop": bool(int(e_stop_match.group(1))) if e_stop_match else None,
            "e_x": float(e_x_match.group(1)) if e_x_match else None,
            "e_y": float(e_y_match.group(1)) if e_y_match else None,
            "e_psi": float(e_psi_match.group(1)) if e_psi_match else None,
        }
        return data


class MotionCalculator:
    """ A utility class for performing motion parameters calculating operation.

    This class provides static methods for calculating various motion parameters,
    including motion direction, Ki for certain motion, scaler of motion parameters.
    All methods are static and can be called without instantiating the class.
    """
    @staticmethod
    def get_motion(controller_data):
        e_x = controller_data["e_x"]
        e_y = controller_data["e_y"]
        e_psi = controller_data["e_psi"]

        if e_x != 0 or e_y != 0:
            if e_x != 0 and e_y != 0:
                return "xy"
            elif e_x != 0:
                return "x"
            else:
                return "y"
        elif e_psi != 0:
            return "psi"

    @staticmethod
    def x_motion_k(Kx, l, w, theta):
        K1 = -(l * np.cos(theta) - w * np.sin(theta) / 2) / (w * np.cos(theta)) * Kx
        K2 = -K1
        K3 = -1 / (2 * np.cos(theta)) * Kx
        K4 = -K3
        return [K1, K2, K3, K4]

    @staticmethod
    def y_motion_k(Ky, l, w, theta):
        K1 = -Ky / (np.cos(2 * theta) - 3)
        K2 = K1
        K3 = (Ky * np.sin(theta)) / (2 * np.sin(theta)**2 + 2)
        K4 = K3
        return [K1, K2, K3, K4]

    @staticmethod
    def m_motion_k(Km, w):
        K1 = -1 / w * Km
        K2 = -K1
        return [K1, K2, 0, 0]
    
    @staticmethod
    def get_kx(proportion, e_x_max, l, w, theta):
        Kx1 = w*cos(theta) / (l*cos(theta) - w*sin(theta)/2)  / e_x_max * proportion *133
        Kx2 = (2*cos(theta)) / e_x_max * proportion * 133/80  *80
        return min([Kx1, Kx2])
    
    @staticmethod
    def get_ky(proportion, e_y_max, l, w, theta):
        Ky1 = -(cos(2*theta) - 3) / e_y_max * proportion *133
        Ky2 = (2*sin(theta)**2 + 2) / (sin(theta)) / e_y_max * proportion * 133/80 *80
        return min([Ky1, Ky2])
    
    @staticmethod
    def get_km(proportion, e_psi_max, l, w, theta):
        Km = w / e_psi_max * proportion *133
        return Km
    

class ThrustAllocator:
    """A class used for thrusts allocation.

    Attributes:
        l (float): Distance in the y direction from the center of mass to the forward propeller.
        w (float): Distance in the x direction from the center of mass to the rare propeller.
        theta (float): Angle between the x direction and propeller thrust direction.

    Methods:
        allocate_thrust(): Return a list of thrusts in lbs.
        lbs_to_pct(): Return a list of thrusts which converted from lbs to percentage.
        pct_to_lbs(): Return a list of thrusts which converted from percentage to lbs.
        pct_to_analog(): Return a list of thrusts which converted from percentage to analog.
        analog_to_pct(): Return a list of thrusts which converted from analog to percentage.
        compensate(): Return a list of compensated thrusts with less motion error.
    """
    def __init__(self, l, w, theta):
        self.l = l
        self.w = w
        self.theta = theta

    def allocate_thrust(self, controller_data):
        e_stop = controller_data["e_stop"]
        e_x = controller_data["e_x"]
        e_y = controller_data["e_y"]
        e_psi = controller_data["e_psi"]

        if e_stop:
            T_rear_left = 0 # P1
            T_rear_right = 0 # P2
            T_front_left = 0 # P3
            T_front_right = 0 # P4

        motion = MotionCalculator.get_motion(controller_data)
        if motion == 'x':
            Kx = MotionCalculator.get_kx(80, 100, self.l, self.w, self.theta) # Use 80% throttle
            Kx_n = MotionCalculator.x_motion_k(Kx, self.l, self.w, self.theta)

            T_rear_left = e_x * Kx_n[0] # P1
            T_rear_right = e_x * Kx_n[1] # P2
            T_front_left = e_x * Kx_n[2] # P3
            T_front_right = e_x * Kx_n[3] # P4

        elif motion == 'y':
            Ky = MotionCalculator.get_ky(80, 100, self.l, self.w, self.theta) # Use 80% throttle
            Ky_n = MotionCalculator.y_motion_k(Ky, self.l, self.w, self.theta)

            T_rear_left = e_y * Ky_n[0] # P1
            T_rear_right = e_y * Ky_n[1] # P2
            T_front_left = e_y * Ky_n[2] # P3
            T_front_right = e_y * Ky_n[3] # P4

        elif motion == 'psi':
            Km = MotionCalculator.get_km(80, 100, self.l, self.w, self.theta) # Use 80% throttle
            Km_n = MotionCalculator.m_motion_k(Km, self.w)

            T_rear_left = e_psi * Km_n[0] # P1
            T_rear_right = e_psi * Km_n[1] # P2
            T_front_left = e_psi * Km_n[2] # P3
            T_front_right = e_psi * Km_n[3] # P4

        else:
            Kx = MotionCalculator.get_kx(30, 100, self.l, self.w, self.theta) # Use 30% throttle
            Kx_n = MotionCalculator.x_motion_k(Kx, self.l, self.w, self.theta)
            Ky = MotionCalculator.get_ky(30, 100, self.l, self.w, self.theta) # Use 30% throttle
            Ky_n = MotionCalculator.y_motion_k(Ky, self.l, self.w, self.theta)
            Km = MotionCalculator.get_km(10, 100, self.l, self.w, self.theta) # Use 10% throttle
            Km_n = MotionCalculator.m_motion_k(Km, self.w)

            T_rear_left = e_x * Kx_n[0] + e_y * Ky_n[0] + e_psi * Km_n[0]
            T_rear_right = e_x * Kx_n[1] + e_y * Ky_n[1] + e_psi * Km_n[1]
            T_front_left = e_x * Kx_n[2] + e_y * Ky_n[2] + e_psi * Km_n[2]
            T_front_right = e_x * Kx_n[3] + e_y * Ky_n[3] + e_psi * Km_n[3]

        thrust_lbs = [T_rear_left, T_rear_right, T_front_left, T_front_right]

        return thrust_lbs

    @staticmethod
    def lbs_to_pct(thrusts):
        return [thrusts[1]/133, thrusts[1]/133, thrusts[2]/80, thrusts[3]/80]
    
    @staticmethod
    def pct_to_lbs(thrusts):
        return [thrusts[1]*133, thrusts[1]*133, thrusts[2] * 80, thrusts[3] * 80]

    @staticmethod
    def pct_to_analog(thrusts):
        return [round(thrust / 100 * 127) for thrust in thrusts]
    
    @staticmethod
    def analog_to_pct(thrusts):
        return [(thrust / 127 * 100) for thrust in thrusts]

    @staticmethod
    def compensate(thrusts_lbs, thrusts_analog, l, w, theta):
        # Define the matrix C
        C = np.array([
            [0, 0, -np.cos(theta), np.cos(theta)],
            [1, 1, np.sin(theta), np.sin(theta)],
            [-w / 2, w / 2, l * np.cos(theta) - (w * np.sin(theta)) / 2, (w * np.sin(theta)) / 2 - l * np.cos(theta)]
        ])

        def calculate_error(T1, T2):
            result1 = np.dot(C, T1)
            result2 = np.dot(C, T2)
            return np.linalg.norm(result1 - result2)
        
        T_lbs = np.array(thrusts_lbs)

        T_alg = np.array(thrusts_analog)
        thrusts_alg2lbs = ThrustAllocator.analog_to_pct(ThrustAllocator.pct_to_lbs(thrusts_analog))
        T_analog2lbs = np.array(thrusts_alg2lbs)
        
        optimal_analog = T_alg.copy()
        min_error = calculate_error(T_lbs, T_analog2lbs)

        # Possible deltas for adjustments
        deltas = [-1, 1]

        # Iterate through possible adjustments for the first two and last two elements
        for delta_first_two in deltas:
            for delta_last_two in deltas:
                modified_analog = T_alg.copy()
                # sign * value
                modified_analog[0] = (1 if modified_analog[0] > 0 else -1) * (abs(modified_analog[0])+delta_first_two)
                modified_analog[1] = (1 if modified_analog[1] > 0 else -1) * (abs(modified_analog[1])+delta_first_two)
                modified_analog[2] = (1 if modified_analog[2] > 0 else -1) * (abs(modified_analog[2])+delta_last_two)
                modified_analog[3] = (1 if modified_analog[3] > 0 else -1) * (abs(modified_analog[3])+delta_last_two)

                # Ensure modified_analog values are within range
                if all(0 <= modified_analog[i] <= 127 for i in range(4)):
                    modified_alg2lbs = ThrustAllocator.analog_to_pct(ThrustAllocator.pct_to_lbs(modified_analog))
                    error = calculate_error(T_lbs, modified_alg2lbs)
                    if error < min_error:
                        min_error = error
                        optimal_analog = modified_analog.copy()

        return optimal_analog.tolist()


class PID:
    def __init__(self, kp, ki, kd):
        """
        initialize incremental PID controller
        :param kp: proportional gain
        :param ki: integral gain
        :param kd: differential gain
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0  # prior error
        self.prev_prev_error = 0  # previous error
        self.delta_output = 0  # increment output
        self.integral_term = 0  # Integral term accumulation for optional anti-integral saturation
    
    def incre(self, error):
        """
        calculate inicremental PID output
        :param error: current error
        :return: increment of control output
        """
        

        # increment calculation：Δu = kp * (e_k - e_k-1) + ki * e_k + kd * (e_k - 2*e_k-1 + e_k-2)
        self.delta_output = (
            self.kp * (error - self.prev_error) +
            self.ki * error +
            self.kd * (error - 2 * self.prev_error + self.prev_prev_error)
        )
        
        # update previous error
        self.prev_prev_error = self.prev_error
        self.prev_error = error
        
        return self.delta_output

def calculate_relative_position(X_B, Y_B, psi_B, X_P, Y_P, psi_P):
    """
    calculate relative pose error e = [e_x, e_y, e_M]^T.
    
    :param X_B: the X coordinate of Body frame in the World frame
    :param Y_B: the Y coordinate of Body frame in the World frame
    :param psi_B:  Angle of Body frame in the World frame (in radians)
    :param X_P: the X coordinate of the target P in the World frame
    :param Y_P: the Y coordinate of the target P in the World frame
    :param psi_P:  Angle of target P in the World frame (in radians)
    :return: relative pose error vector [e_x, e_y, e_M]
    """
  
    e_x = (
        X_P * np.cos(psi_B)
        - X_B * np.cos(psi_B)
        - Y_B * np.sin(psi_B)
        + Y_P * np.sin(psi_B)
    )
    e_y = (
        Y_P * np.cos(psi_B)
        - Y_B * np.cos(psi_B)
        + X_B * np.sin(psi_B)
        - X_P * np.sin(psi_B)
    )
    e_M = psi_P - psi_B

    # return error vector
    return np.array([e_x, e_y, e_M])

class DeviceDetector:
    """A class used for detecting device.

    This class provides static methods for detecting device.
    All methods are static and can be called without instantiating the class.

    Methods:
        detect_devices(): Return a dictionary about detected serial port.
    """
    @staticmethod
    def detect_devices():
        arduino_ports = {
            "navyLeft": None,
            "navyRight": None,
            "MK": None,
            "controller": None
        }

        ports = serial.tools.list_ports.comports()
        for port in ports:
            try:
                if 'usb' in port.device:
                    ser = serial.Serial(port.device, 38400, timeout=1)
                    time.sleep(2)
                    command = bytes([0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
                    ser.write(command)
                    response = ser.readline().decode('utf-8').strip()
                    if response in arduino_ports:
                        arduino_ports[response] = port.device
                    ser.close()
            except Exception as e:
                print(f"Failed to connect to {port.device}: {e}")

        return arduino_ports


def set_control_data(thrusts, eStop, manual):
    data = []
    # thrust is int
    for thrust in thrusts:
        if thrust > 0:
            direction = True
        else:
            direction = False

        velocity = abs(thrust)
        # Converts bool to bytes (True -> 1, False -> 0)
        direction_byte = struct.pack('B', direction)

        # Converts int to bytes
        velocity_bytes = struct.pack('B', velocity)
    
        # Merge two byte lists
        data.append(direction_byte + velocity_bytes)
    
    eStop_bytes = struct.pack('B', eStop)
    data.append(eStop_bytes)

    Manual_bytes = struct.pack('B', manual)
    data.append(Manual_bytes)

    return data


if __name__ == "__main__":
    # detect device
    detector = DeviceDetector()
    arduino_ports = detector.detect_devices()
    
    # Check to see if all devices are found
    if None in arduino_ports.values():
        print("Some devices were not detected.")
        exit()

    # Create a serial communication instance.
    controller_port = arduino_ports["controller"]
    controller_comm = SerialCommunication(controller_port)

    NavyLeft_port = arduino_ports["navyLeft"]
    NavyLeft_comm = SerialCommunication(NavyLeft_port)

    NavyRight_port = arduino_ports["navyRight"]
    NavyRight_comm = SerialCommunication(NavyRight_port)

    MK_port = arduino_ports["MK"]
    MK_comm = SerialCommunication(MK_port)

    # Initialization of thrusts allocator
    w = 243.8 * 10 # unit: mm 
    l = 485.1 * 10 # unit: mm 
    theta = np.pi / 4  # unit: rad
    allocator = ThrustAllocator(l, w, theta)
    
    # operation status info
    eStop = True
    manual = True
    autonomous = False

    try:
        while True:
            # read data
            raw_data = controller_comm.read_data()
            if raw_data:
                print(f"Received: {raw_data}")
                
                # Parse controller data
                parsed_data = ControllerDataParser.parse_controller_data(raw_data)
                
                # Thrusts allocation
                thrusts_lbs = allocator.allocate_thrust(parsed_data)

                # Convert lbs thrust to percentage
                thrusts_percent = allocator.lbs_to_pct(thrusts_lbs)
                
                # Convert percentage thrust to analog 
                thrusts_analog = allocator.pct_to_analog(thrusts_percent)
                print(f"Analog Thrust Values: {thrusts_analog}")
                
                # Thrust compensation to ensure optimal analog adjustment
                thrusts_opt_alg = allocator.compensate(thrusts_lbs, thrusts_analog, l, w, theta)
                print(f"Compensated Thrusts: {thrusts_opt_alg}")

                eStop = parsed_data["e_stop"]
                control_data = set_control_data(thrusts_opt_alg, eStop, manual)

                # send data to arduino
                NavyLeft_comm.send_data(control_data)
                NavyRight_comm.send_data(control_data)
                MK_comm.send_data(control_data)

            # reading delay，avoiding occupying CPU too much
            # time.sleep(0.1)

    except KeyboardInterrupt:
        print("Terminating program...")
    finally:
        controller_comm.close()