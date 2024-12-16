from fusion import Fusion
import digital_filters as filt
from scipy import signal
import sys
from collections import deque
import time
import warnings
import qwiic_icm20948
import busio
import board
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

class Imu():
    def __init__(self,name,FS):
        self.address='QwiicIcm20948'
        self.ssf_acc=8192       #scale factors to convert readings to m/s^2
        self.ssf_gyr=65.5       #scale factors to convert readings to rad/s
        self.fs = FS
        self.dt = 1.0 / FS 
        
        self.fuse= Fusion()
    
    def initialize(self, Qwiicfunction):                                   #connects to Imu_1
        self.IMU = Qwiicfunction
        if self.IMU.connected == False:
            print(f"{Qwiicfunction} device isn't connected to the system. Please check your connection", file=sys.stderr)
        else:
            self.IMU.begin()
            print(f'Done {Qwiicfunction}')
        self.start_time=time.time()
        
        lp_butter_coeffs = signal.butter(2, 10, 'lowpass', fs = self.fs, output = 'ba')
        self.sh_filter = filt.LiveLFilter(b = lp_butter_coeffs[0], a = lp_butter_coeffs[1])
        self.sh_filtered_vals = deque([0.0] * 4, maxlen = 4)

    def compute_sh_dot_back(self, sh_cur):                          #computes derivative of gyroscope z axis to estimate angular velocity, then applies LP filter
        sh_filt_cur = self.sh_filter.update_filter(sh_cur)
        self.sh_filtered_vals.appendleft(sh_filt_cur)

        # Source: https://web.media.mit.edu/~crtaylor/calculator.html. Location of sampled points -3,-2,-1,0
        f = self.sh_filtered_vals
        h = self.dt
        deriv = (-2*f[3]+9*f[2]-18*f[1]+11*f[0]) / (6*1.0*h**1)
        return sh_filt_cur, deriv

    def read_imu(self):                          #reads and converts Imu_raw acc/gyr to standard scale. Computers derivative of gyr z, applies LP, and fusion algorithm 
                                                    # to calculate heading, pitch, roll
        #self.test.enable_channels([self.channel])
        if self.IMU.dataReady():
            self.IMU.getAgmt()
            self.IMU.axRaw=self.IMU.axRaw/self.ssf_acc
            self.IMU.ayRaw=self.IMU.ayRaw/self.ssf_acc
            self.IMU.azRaw=self.IMU.azRaw/self.ssf_acc
            
            self.IMU.gxRaw=self.IMU.gzRaw/self.ssf_gyr
            self.IMU.gyRaw=self.IMU.gyRaw/self.ssf_gyr
            self.IMU.gzRaw=self.IMU.gzRaw/self.ssf_gyr
            
            self.IMU.gzfilt,self.IMU.gzdot=self.compute_sh_dot_back(self.IMU.gzRaw)
                 
            self.fuse.update_nomag([-self.IMU.axRaw,self.IMU.azRaw,self.IMU.ayRaw],[self.IMU.gxRaw,self.IMU.gzRaw,self.IMU.gyRaw],self.dt)
            self.IMU.heading=self.fuse.heading
            self.IMU.pitch=self.fuse.pitch
            self.IMU.roll=self.fuse.roll
        
        else:
            print(f'{Qwiicfunction} not ready')
            self.IMU.gzfilt = 0
            self.IMU.pitch = 0
            self.IMU.heading = 0
            self.IMU.roll = 0
        
        return self.IMU
  
class loadCell:
    def __init__(self, offset, gain):
        self.offset = offset
        self.gain = gain
        self.chan = self.ADC_initialize()

    def ADC_initialize(self):
        # Initialize ADS1115, configure it for reading from a load cell.
        print(board.SCL)
        print(board.SDA)
        i2c = busio.I2C(board.SCL, board.SDA)
        
        ads = ADS.ADS1115(i2c)
        ads.gain = 1
        chan = AnalogIn(ads, ADS.P0, ADS.P1)
        return chan

    def ADC_read_volt(self):
        # Read voltage from ADC, proportional to force measured by load cell
        return self.chan.voltage

    def read_force(self):
        # Read current force applied to load cell
        force = (self.ADC_read_volt() * self.gain + self.offset)
        return force

    def read_volt(self):
        # Read current voltage
        volt = self.ADC_read_volt()
        return volt