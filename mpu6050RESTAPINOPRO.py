from flask import request
from flask_api import FlaskAPI
import smbus
import time

class mpu6050:

    # Global Variables
    GRAVITIY_MS2 = 9.80665
    address = None
    bus = None

    # MPU-6050 Registers

    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F

    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47

    def __init__(self, address, bus=1):
        self.address = address
        self.bus = smbus.SMBus(bus)
        # Wake up the MPU-6050 since it starts in sleep mode
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

    # I2C communication methods

    def read_i2c_word(self, register):
        # Read the data from the registers
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)

        value = (high << 8) + low

        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value

    def get_accel_data(self, g = False):
        x = self.read_i2c_word(self.ACCEL_XOUT0)
        y = self.read_i2c_word(self.ACCEL_YOUT0)
        z = self.read_i2c_word(self.ACCEL_ZOUT0)

        return {'x': x, 'y': y, 'z': z}

    def get_gyro_data(self):
        x = self.read_i2c_word(self.GYRO_XOUT0)
        y = self.read_i2c_word(self.GYRO_YOUT0)
        z = self.read_i2c_word(self.GYRO_ZOUT0)

        return {'x': x, 'y': y, 'z': z}

    def get_all_data(self):
        accel = self.get_accel_data()
        gyro = self.get_gyro_data()

        return [accel, gyro]

mpu1 = mpu6050(0x68)
mpu2 = mpu6050(0x69)

app = FlaskAPI(__name__)

@app.route('/getData',methods=["GET"])
def api_root():
	accel_data1 = mpu1.get_accel_data()
	gyro_data1 = mpu1.get_gyro_data()
	accel_data2 = mpu2.get_accel_data()
	gyro_data2 = mpu2.get_gyro_data()
	return{
		"mpu1": {
			"accel": {
				"x" : accel_data1['x'],
				"y" : accel_data1['y'],
				"z" : accel_data1['z']
			},
			"gyro" : {
				"x" : gyro_data1['x'],
				"y" : gyro_data1['y'],
				"z" : gyro_data1['z']
			
			}
		},
		"mpu2": {
			"accel": {
				"x" : accel_data2['x'],
				"y" : accel_data2['y'],
				"z" : accel_data2['z']
			},
			"gyro" : {
				"x" : gyro_data2['x'],
				"y" : gyro_data2['y'],
				"z" : gyro_data2['z']
			
			}
		}
	}

@app.route('/test',methods=["GET"])
def api_test1():
	return{
		"Text":"Test Works"
	}

if __name__ == "__main__":
	app.run()
