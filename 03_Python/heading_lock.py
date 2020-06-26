import robot
import ak8975
import os
import yaml
import time
import math

if __name__ == "__main__":
        robot = robot.Robot(serial_port='/dev/ttyUSB0')
        sensor = ak8975.AK8975(1, 0x0c)
        if not os.path.isfile('./calib.yaml') :
                print("Calibrate compass first")
                exit(0)
        else:
                with open('./calib.yaml', 'r') as stream:
                        calib_data = yaml.load(stream, Loader=yaml.Loader)
        if 'offsets' in calib_data:
                sensor.setOffset(calib_data['offsets'])
        if 'scale' in calib_data:
                sensor.setScale(calib_data['scale'])
        first_heading = None
        while first_heading == None or int(first_heading) != 0:
              first_heading = sensor.read_heading()
              if not first_heading is None :
                 print("{:06.2f}° : {} volts ".format(first_heading, robot.vbat))
              time.sleep(0.01)
        while True :
                time.sleep(0.01)
                current_heading = sensor.read_heading()
                if current_heading > 180.0 :
                     current_heading = current_heading - 360
                print("{}".format(current_heading))
                error_heading = first_heading - current_heading
                ctrl = -(error_heading * 2.5)
                robot.set_steer(ctrl*math.pi/180)
        sensor.close()
