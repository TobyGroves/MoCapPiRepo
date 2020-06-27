from flask import request
from flask_api import FlaskAPI
from threading import Thread
import smbus
import time
#import threading

class mpu6050:
    GRAVITIY_MS2 = 9.80665
    address = None
    bus = None

    # MPU-6050 Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F

    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47

    xAccelOffset = 0.0
    yAccelOffset = 0.0
    zAccelOffset = 0.0

    xGyroOffset = 0.0
    yGyroOffset = 0.0
    zGyroOffset = 0.0

    mean_ax = 0
    mean_ay = 0
    mean_az = 0
    mean_gx = 0
    mean_gy = 0
    mean_gz = 0

    min_ax = 0
    min_ay = 0
    min_az = 0
    min_gx = 0
    min_gy = 0
    min_gz = 0
    max_ax = 0
    max_ay = 0
    max_az = 0
    max_gx = 0
    max_gy = 0
    max_gz = 0

    buffersize = 1000

    acel_deadzone = 8
    gyro_deadzone = 1

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

    def get_accel_data(self):
        x = self.read_i2c_word(self.ACCEL_XOUT0) + self.xAccelOffset
        y = self.read_i2c_word(self.ACCEL_YOUT0) + self.yAccelOffset
        z = self.read_i2c_word(self.ACCEL_ZOUT0) + self.zAccelOffset

        return {'x': x, 'y': y, 'z': z}

    def get_gyro_data(self):
        x = self.read_i2c_word(self.GYRO_XOUT0) + self.xGyroOffset
        y = self.read_i2c_word(self.GYRO_YOUT0) + self.yGyroOffset
        z = self.read_i2c_word(self.GYRO_ZOUT0) + self.zGyroOffset

        return {'x': x, 'y': y, 'z': z}

    def get_all_data(self):
        accel = self.get_accel_data()
        gyro = self.get_gyro_data()

        return [accel, gyro]
    def setXAccelOffset(self,val):
        self.xAccelOffset = val

    def setYAccelOffset(self,val):
        self.yAccelOffset = val

    def setZAccelOffset(self,val):
        self.zAccelOffset = val

    def setXGyroOffset(self,val):
        self.xGyroOffset = val

    def setYGyroOffset(self,val):
        self.yGyroOffset = val

    def setZGyroOffset(self,val):
        self.zGyroOffset = val

    def calibrate(self):
        self.setXAccelOffset(0)
        self.setYAccelOffset(0)
        self.setZAccelOffset(0)
        self.setXGyroOffset(0)
        self.setYGyroOffset(0)
        self.setZGyroOffset(0)
        print("set offsets to 0")
        self.meansensors()
        self.setXAccelOffset(-self.mean_ax)
        self.setYAccelOffset(-self.mean_ay)
        self.setZAccelOffset(16384-self.mean_az)
        self.setXGyroOffset(-self.mean_gx)
        self.setYGyroOffset(-self.mean_gy)
        self.setZGyroOffset(-self.mean_gz)

    def meansensors(self):
        i = 0
        buff_ax = 0
        buff_ay = 0
        buff_az = 0
        buff_gx = 0
        buff_gy = 0
        buff_gz = 0
        print("calculating mean sensors")
        while(i<(self.buffersize+101)):
            # read raw accel and gyro measurements
            ax = self.read_i2c_word(self.ACCEL_XOUT0) + self.xAccelOffset
            ay = self.read_i2c_word(self.ACCEL_YOUT0) + self.yAccelOffset
            az = self.read_i2c_word(self.ACCEL_ZOUT0) + self.zAccelOffset
            gx = self.read_i2c_word(self.GYRO_XOUT0) + self.xGyroOffset
            gy = self.read_i2c_word(self.GYRO_YOUT0) + self.yGyroOffset
            gz = self.read_i2c_word(self.GYRO_ZOUT0) + self.zGyroOffset
            if(i>100 and i<=(self.buffersize+100)): # first 100 measures are discarded
                buff_ax = buff_ax+ax
                buff_ay = buff_ay+ay
                buff_az = buff_az+az
                buff_gx = buff_gx+gx
                buff_gy = buff_gy+gy
                buff_gz = buff_gz+gz

            if(i==(self.buffersize+100)):
                self.mean_ax=buff_ax/self.buffersize
                self.mean_ay=buff_ay/self.buffersize
                self.mean_az=buff_az/self.buffersize
                self.mean_gx=buff_gx/self.buffersize
                self.mean_gy=buff_gy/self.buffersize
                self.mean_gz=buff_gz/self.buffersize
            i+= 1
            #time.sleep(0.002) # so we dont get repeated measurements
        print("mean sensors calculated")

dataLoopCount = 0
accel_dataconst1 = 0
gyro_dataconst1 = 0
mpu1 = mpu6050(0x68)
mpu2 = mpu6050(0x69)
isList1 = True
recording = False
bob = False
list1 = []
list2 = []
thread = None
dataHandellerThread = None

#testThread = threading.Thread()

#app = FlaskAPI(__name__)
def create_app():
	_app = FlaskAPI(__name__)

	return _app


app = create_app()


@app.route('/calibrate',methods=["GET"])
def api_calibrate():
    print("recived calibration request")
    mpu1.calibrate()
    return {
        "status" : "Calibration Complete",
        "mean_ax" : mpu1.mean_ax,
        "mean_ay" : mpu1.mean_ay,
        "mean_az" : mpu1.mean_az,
        "mean_gx" : mpu1.mean_gx,
        "mean_gy" : mpu1.mean_gy,
        "mean_gz" : mpu1.mean_gz,
        "ax_offset" : mpu1.xAccelOffset,
        "ay_offset" : mpu1.yAccelOffset,
        "az_offset" : mpu1.zAccelOffset,
        "gx_offset" : mpu1.xGyroOffset,
        "gy_offset" : mpu1.yGyroOffset,
        "gz_offset" : mpu1.zGyroOffset
    }


@app.route('/getData',methods=["GET"])
def api_getData():
    accel_data1 = mpu1.get_accel_data()
    gyro_data1 = mpu1.get_gyro_data()
    accel_data2 = mpu2.get_accel_data()
    gyro_data2 = mpu2.get_gyro_data()

    return {
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

@app.route('/getDataMaxfps',methods=["GET"])
def api_getDataMaxfps():
    global isList1
    global list1
    global list2
    if recording:
        if isList1:
            isList1 = False
            tempList = list1
            list1 = []
            return {
                    "mpuMoveList":tempList
                    }
        else:
            isList1 = True
            tempList = list2
            list2 = []
            return {
                    "mpuMoveList":tempList
                    }
    else:
        return {
                "Error":"NotRecording"
                }

#fix the threading issue
def testThred():
    while True:
        time.sleep(1)
        print("ThreadTest")
        #if this works do an update every so oftern

@app.route('/threadTest',methods=["GET"])
def api_threadtest():
	global thread
	if not thread:
		thread = Thread(target = testThred)
		thread.setDaemon(True)
		thread.start()
	return{
		"Text":"Thread started"
	}

def dataHandeller():
    global list1
    global list2
    #print("called datahandeller")
    lastPollTime = time.time()
    print(recording)
    while recording:
        #print("in Loop Data handeller")
        timeSinceLastPoll = time.time() - lastPollTime;
        data = {"accel_data" : mpu1.get_accel_data(), "gyro_data" : mpu1.get_gyro_data(), "timeSinceLastPoll" : timeSinceLastPoll}
        if isList1:
            list1.append(data)
        else:
            list2.append(data)
        lastPollTime = time.time()
        time.sleep(0.01)
        #print(list1)
        #print(list2)


@app.route('/startRecording',methods=["GET"])
def api_startRecording():
    global dataHandellerThread
    global recording
    global isList1
    global list1
    global list2
    if not dataHandellerThread:
        dataHandellerThread = Thread(target = dataHandeller)
        dataHandellerThread.setDaemon(True)
        #print("before making true")
        #print(recording)
        recording = True
        #print("after making true")
        #print(recording)
        isList1 = True
        list1 = []
        list2 = []
        dataHandellerThread.start()
    return{
        "Text":"Recording started"
    }
@app.route('/stopRecording',methods=["GET"])
def api_stopRecording():
    global recording
    recording = False
    return{
        "Text":"Recording stopped"
    }





if __name__ == "__main__":
	app.run()
