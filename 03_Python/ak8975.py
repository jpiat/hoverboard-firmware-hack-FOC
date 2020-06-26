from smbus import SMBus
import time
import struct
import math
import yaml
import numpy as np
import sys
import select
import os
import math

REG_WIA                   = 0x00
REG_INFO                  = 0x01
REG_ST1                   = 0x02
REG_HXL                   = 0x03
REG_HXH                   = 0x04
REG_HYL                   = 0x05
REG_HYH                   = 0x06
REG_HZL                   = 0x07
REG_HZH                   = 0x08
REG_ST2                   = 0x09
REG_CNTL                  = 0x0a
REG_ASTC                  = 0x0c
REG_ASAX                  = 0x10
REG_ASAY                  = 0x11
REG_ASAZ                  = 0x12


CNTL_PWRDWN               = 0x00
CNTL_MEASURE              = 0x01
CNTL_SELFTEST             = 0x08
CNTL_FUSE_ACCESS          = 0x0f

ST1_DRDY                  = 0x01

ALPHA = 0.80

class AK8975 :

	def __init__(self, i2c_bus, i2c_addr):
		self._bus = SMBus(i2c_bus)
		self._dev_addr = i2c_addr
		self.xCoeff = self._bus.read_byte_data(self._dev_addr, REG_ASAX)
		self.yCoeff = self._bus.read_byte_data(self._dev_addr, REG_ASAY)
		self.zCoeff = self._bus.read_byte_data(self._dev_addr, REG_ASAZ)
		self._bus.write_byte_data(self._dev_addr, REG_CNTL, CNTL_PWRDWN)
		self.offsets = [0.]*3
		self.scale = [1.]*3
		self.last_sin = 0.
		self.last_cos = 0.0
		self.lock_counter = 100

	def setOffset(self, offsets):
		self.offsets = offsets

	def setScale(self, scale):
                self.scale = scale

	def close(self):
		self._bus.close()

	def read_gauss(self):
		self._bus.write_byte_data(self._dev_addr, REG_CNTL, CNTL_MEASURE)
		while (self._bus.read_byte_data(self._dev_addr, REG_ST1) & ST1_DRDY) ==  0:
			time.sleep(0.001)
		data = self._bus.read_i2c_block_data(self._dev_addr, REG_HXL, 6)
		data = bytes(data)
		fields = struct.unpack('hhh', data)
		x = self.adjustValue(fields[0], self.xCoeff)
		y = self.adjustValue(fields[1], self.yCoeff)
		z = self.adjustValue(fields[2], self.zCoeff)
		return (x, y, z)

	def read_heading(self):
		x, y, z = self.read_gauss()
		x = (x - self.offsets[0])*self.scale[0]
		y = (y - self.offsets[1])*self.scale[1]
		z = (z - self.offsets[2])*self.scale[2]
		#heading = 90+ (math.atan2(y, x) * 180./math.pi)
		#heading  = heading + 90.0 #robot compass not north aligned
		heading = math.atan2(y, x)
		self.last_sin = ALPHA*self.last_sin + (1- ALPHA) * math.sin(heading)
		self.last_cos = ALPHA*self.last_cos + (1- ALPHA) * math.cos(heading)
		heading = math.atan2(self.last_sin, self.last_cos) * 180./math.pi
		heading = heading + 180
		if self.lock_counter == 0:
			return heading
		else:
			self.lock_counter = self.lock_counter - 1
			return None

	def adjustValue(self, value, adj):
		# apply the proper compensation to value.  This equation is taken
  		# from the AK8975 datasheet, section 8.3.11
		return ( value * ((((adj - 128.0) * 0.5) / 128.0) + 1.0) )



def something(line):
  print('read input:', line, end='')

def something_else():
  print('no input')


def read_keyboard():
	if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
		return  sys.stdin.readline()
	return None

if __name__ == "__main__":
	sensor = AK8975(1, 0x0c)
	if not os.path.isfile('./calib.yaml') :
		calib_data = {}
		mins = [0.]*3
		maxs = [0.]*3
		print("Gathering data, do the 8")
		values = []
		while read_keyboard() == None:
			time.sleep(0.25)
			vals = sensor.read_gauss()
			values.append(vals)
		val_array = np.matrix(values)
		min = np.amin(val_array, axis=0)
		max = np.amax(val_array, axis=0)
		offsets = (max + min)/2
		val_array = val_array - offsets
		min = np.amin(val_array, axis=0)
		max = np.amax(val_array, axis=0)
		avgs = (max - min)/2
		intensity = np.sum(avgs)/3
		scale = intensity/avgs
		#	for i in range(3):
		#		if vals[i] > maxs[i]:
		#			maxs[i] = vals[i]
		#		if vals[i] < mins[i]:
                #                       mins[i] = vals[i]
		#offset_x = (maxs[0] + mins[0])/2
		#offset_y = (maxs[1] + mins[1])/2
		#offset_z = (maxs[2] + mins[2])/2
		calib_data['offsets'] = np.asarray(offsets).ravel().tolist()
		calib_data['scale'] = np.asarray(scale).ravel().tolist()
		print(calib_data)
		with open('./calib.yaml', 'w') as file:
			yaml.dump(calib_data, file)
		exit(0)
	else:
		with open('./calib.yaml', 'r') as stream:
			calib_data = yaml.load(stream, Loader=yaml.Loader)
	if 'offsets' in calib_data:
		sensor.setOffset(calib_data['offsets'])
	if 'scale' in calib_data:
		sensor.setScale(calib_data['scale'])

	while True :
		time.sleep(0.1)
		print("{}".format(sensor.read_heading()))
	sensor.close()

