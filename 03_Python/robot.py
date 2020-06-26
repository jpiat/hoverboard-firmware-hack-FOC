import time
import traitlets
from traitlets.config.configurable import SingletonConfigurable
import serial
import time
import struct
import math
import threading

def clamp(n, smallest, largest): return max(smallest, min(n, largest))


class Robot(SingletonConfigurable, threading.Thread):

    SPEED_COEFF = 1.0
    STEER_COEFF = 0.01

    def __init__(self, wheel_diameter_m=(10*0.0254), wheel_spacing_m=1.0,  serial_port='/dev/ttyTHS1'):
        super(Robot, self).__init__()
        threading.Thread.__init__(self)
        self.control_lock = threading.Lock()
        self.ser = serial.Serial(serial_port, 38400)  # open serial port
        self.speed = 0
        self.steer = 0
        self.right_wheel_rpm = 0
        self.left_wheel_rpm = 0
        self.vbat = 0
        self.temp = 0
        self.wheel_diameter = wheel_diameter_m
        self.wheel_spacing = wheel_spacing_m
        self.target_speed = 0
        self.target_steer = 0
        self.start()

    def sync(self):
        if self.ser.inWaiting() > 18 : #There is one full frame available
           ins = self.ser.read(1)
           if ins == b'\xcd' :
              ins = self.ser.read(1)
              if ins == b'\xab' :
                 return True
        return False

    def get_steer(self):
        diff_speed = -(((self.right_wheel_rpm + self.left_wheel_rpm)/2)/60)*math.pi
        angle = math.atan2(diff_speed, self.wheel_spacing/2)
        return angle #in radians/s

    def get_speed(self):
        return (((self.right_wheel_rpm - self.left_wheel_rpm)/2)/60)*math.pi #in m/s

        #m/s
    def set_speed(self, speed):
        speed = clamp(speed, -1.5, 1.5)
        self.target_speed = speed
        self.control_lock.acquire()
        self.speed = (self.target_speed/(math.pi * self.wheel_diameter)*60)/Robot.SPEED_COEFF  #convert to rpm
        #print("New speed in RPM : {}".format(self.speed))
        self.control_lock.release()
        #rad/s
    def set_steer(self, steer):
        steer = clamp(steer, -math.pi/3, math.pi/3)
        self.target_steer = steer
        self.control_lock.acquire()
        self.steer = (math.tan(steer)*(self.wheel_spacing/2))/Robot.STEER_COEFF
        #print("New speed in differential RPM : {}".format(self.steer))
        self.control_lock.release()

    def forward(self, speed=0.25, duration=None):
        self.set_speed(speed)

    def backward(self, speed=0.25):
        self.set_speed(-speed)

    def left(self, speed=1.0):
        self.set_speed(0)
        self.set_steer(50.0*math.pi/180)

    def right(self, speed=1.0):
        self.set_speed(0)
        self.set_steer(-50.0*math.pi/180)

    def stop(self):
        self.set_speed(0)
        self.set_steer(0)

    def spinonce(self):
        if self.sync():
           full = self.ser.read(16)
           data = struct.unpack('hhhhhhHH', full)
           checksum = 0xABCD
           for a in data[0:-2] :
               checksum = checksum ^ a
               checksum = checksum & 0xFFFF
               if data[-1] == checksum :
                  self.right_wheel_rpm = data[2]
                  self.left_wheel_rpm = data[3]
                  self.vbat = data[4]/100.
                  self.temp = data[5]/100.
                  self.control_lock.acquire()
                  output_checksum = int(0xABCD)
                  output_checksum = (output_checksum & 0xFFFF) ^ int(self.steer)
                  output_checksum = (output_checksum & 0xFFFF) ^ int(self.speed)
                  control_bytes  = struct.pack('HhhH', 0xABCD, int(self.steer), int(self.speed), output_checksum)
                  self.ser.write(control_bytes)
                  self.control_lock.release()
                  return True
        return False

    def run(self):
       while True :
             if not self.spinonce():
                time.sleep(0.001)



if __name__ == "__main__":
	robot = Robot(serial_port='/dev/ttyUSB0')
	while True:
		robot.set_steer(-50.0*math.pi/180)
		time.sleep(0.03)
