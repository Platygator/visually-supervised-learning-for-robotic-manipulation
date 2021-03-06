import serial
import time


class SerialConnection:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyACM0', baudrate=57600, timeout=1)  # open serial port
        self.ser.close()
        self.ser.open()
        time.sleep(2)

    def wait_for_execution(self):
        message = self.ser.readline().decode('ascii')
        while message != "done\r\n":
            message = self.ser.readline().decode('ascii')
            time.sleep(0.5)
        time.sleep(1)

    def get_values(self):
        arduino_data = self.ser.readline().decode('ascii')
        try:
            x, y, z = [float(k) for k in arduino_data.split(" ")]
        except ValueError:
            return 0
        return x, y, z

    def run_serial(self, body, shoulder, elbow, gripper_close):
        conversion_factor = 180/3.1415
        # Type Cast to INT is required to avoid issues in Serial Communication!!
        body = int(body * conversion_factor)
        shoulder = int(144 - (shoulder * conversion_factor))
        elbow = int(88 + (elbow * conversion_factor))
        # body = - int(90 + (body * conversion_factor))
        # shoulder = int(144 - (shoulder * conversion_factor))
        # elbow = int(88 + (elbow * conversion_factor))
        if gripper_close:
            gripper = 60
        else:
            gripper = 0
        message = str(body).zfill(3) + str(shoulder).zfill(3) + str(elbow).zfill(3) + str(gripper).zfill(3)
        self.ser.write(bytes(message, encoding='ascii'))

    def move_to_home(self):
        message = "090020001000"
        self.ser.write(bytes(message, encoding='ascii'))

    def close_connection(self):
        self.ser.close()


if __name__ == '__main__':
    s = SerialConnection()
    time.sleep(5)
    # s.move_to_home()
    # time.sleep(2)
    # print(s.get_values())
    # # s.runSerial(70, 70, 10, 60)
    from math import pi
    conversion = pi/180
    print("move 1")
    s.run_serial(0, 90 * conversion, - 90 * conversion, True)
    s.wait_for_execution()
    print("move 2")
    s.run_serial(45 * conversion, 90 * conversion, - 90 * conversion, True)
    s.wait_for_execution()
    print("move 3")
    s.run_serial(90 * conversion, 90 * conversion, - 90 * conversion, True)
    s.wait_for_execution()
    s.move_to_home()
    s.wait_for_execution()
    s.close_connection()
    # s.run_serial(pi/4, pi/3, pi/6, 60)
    # time.sleep(10)
    # s.run_serial(90 + 90, 20 + 144, 1 - 88, 60)
