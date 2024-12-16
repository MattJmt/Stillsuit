import board
import pyCandle
import signal
import sys
import threading
import time
import warnings
import numpy as np
from tdu import Imu
from tdu import loadCell
import csv
import datetime
import qwiic_icm20948

filename = "DYNAMICS_experiments/Reference/Reference_Steel_fast_2.csv"

FS = 100

LC_OFFSET = 35.6    #0.406 / - 0.061
LC_GAIN = 190       #50.814 / 92.155

STATE_OK = 0
STATE_EXITING = 1

global CUR_STATE
CUR_STATE = STATE_OK

LOCK = threading.Lock()

default_handler = None
def handler(num, frame):
    global CUR_STATE
    CUR_STATE = STATE_EXITING
    return default_handler(num,frame)

class motorState():
    def __init__(self): 
        self.p_des = 0.0
        self.p_cur = 0.0
        self.v_des = 0.0
        self.v_cur = 0.0
        self.t_des = 0.0
        self.t_cur = 0.0
        self.pretension = False

class IMUState():
    def __init__(self):
        self.th_cur = 0.0
        self.thd_cur = 0.0
        self.gzfilt = 0.0
        self.pitch = 0.0
        self.stance = 0.0
        self.HS = 0.0
        self.TO = 0.0

class loadcellState():
    def __init__(self):
        self.force = 0.0

class systemState():
    def __init__(self, n_motors = 2, n_imus = 2, n_loadcell = 1):
        self.motor_states = [motorState() for _ in range(n_motors)]
        self.imu_states = [IMUState() for _ in range(n_imus)]
        self.loadcell_states = [loadcellState() for _ in range(n_loadcell)]

class motorControlLoop(threading.Thread):
    def __init__(self, name, sys_state, md, motor_ind = 0, imu_ind = 0):
        threading.Thread.__init__(self)
        self.name = name
        self.sys_state = sys_state
        self.fs = FS
        self.md = md
        self.motor_ind = motor_ind
        self.last_torque_update_time = time.perf_counter()
        self.imu_ind = imu_ind


    def run(self):
        t = 0.0
        t_desdes = 0.0
        global CUR_STATE
        pretension = False
        p_des = 0.0
        v_des = 0.0
        t_des = 0.0
        p_prev = 10.0
        stance_bool = -1
        motor_multiplier = -1 if self.motor_ind == 1 else 1     # to account for motors spinning in different directions
        not_function = lambda x: 0 if x == 1 else 1             # to call other motor index
        
        while CUR_STATE != STATE_EXITING:
            next_time_instant = time.perf_counter() + (1.0 / self.fs)

            with LOCK:
                th_cur = self.sys_state.imu_states[self.motor_ind].th_cur

            p_cur = self.md.getPosition()
            v_cur = self.md.getVelocity()
            t_cur = self.md.getTorque()

            with LOCK:
                self.sys_state.motor_states[self.motor_ind].p_cur = p_cur
                self.sys_state.motor_states[self.motor_ind].v_cur = v_cur
                self.sys_state.motor_states[self.motor_ind].t_cur = t_cur
                self.sys_state.motor_states[self.motor_ind].p_des = p_des
                self.sys_state.motor_states[self.motor_ind].v_des = v_des
                self.sys_state.motor_states[self.motor_ind].t_des = t_des
                self.sys_state.motor_states[self.motor_ind].pretension = pretension

            
            """ 
            if self.motor_ind == 0:
                p_des = 0.0
                # v_des = 0.0 * motor_multiplier
                t_des = t_desdes * motor_multiplier
                self.md.setImpedanceControllerParams(0.0, 0.0) #control in: pos = Kp, Kd | vel = 0, Kd | t_ff = 0,0
                self.md.setTargetTorque(t_des)
            
            # ramp increase every 10s
            if time.perf_counter() - self.last_torque_update_time >= 10:
                t_desdes += 0.5  
                self.last_torque_update_time = time.perf_counter()
            """
            # simulate stance phase happening
            if self.motor_ind == 0:
                if stance_bool == 1:
                    t_des = 2.0
                else:
                    t_des = 0.0
                self.md.setImpedanceControllerParams(0.0,0.0)
                self.md.setTargetTorque(t_des)

            if time.perf_counter() - self.last_torque_update_time >= 0.63: # 0.90
                stance_bool *= -1
                self.last_torque_update_time = time.perf_counter()
            

            """
                
            if pretension == False:
                if abs(p_cur - p_prev) > 0.001:
                    print("\033[K", end='')
                    print(f"Motor {self.motor_ind} Pretensioning: TENSIONING", end='\r', flush=True)
                        
                    v_des = 1.0 * motor_multiplier
                    self.md.setImpedanceControllerParams(0.0, 1.0) #control in: pos = Kp, Kd | vel = 0, Kd | t_ff = 0,0
                    self.md.setTargetVelocity(v_des)
                    p_prev = p_cur
                else:
                    pretension = True
                    v_des = 0.0
                    self.md.setTargetVelocity(v_des)
                    print(f"Motor {self.motor_ind} Pretensioning: DONE ")

            
            # step impulse during stance phase
            if pretension == True: #and self.sys_state.motor_states[not_function(self.motor_ind)].pretension == True:
                self.md.setImpedanceControllerParams(0.0, 0.0)

                if self.sys_state.imu_states[self.imu_ind].stance == 1:
                    t_des = 2.0 * motor_multiplier
                    self.md.setTargetTorque(t_des)
                
                else:
                    t_des = 0.0
                    self.md.setTargetTorque(t_des)
            
            """
            
            # p_des = 0.0
            # t_des = -0.5
            # self.md.setImpedanceControllerParams(0.0, 0.0) #control in: pos = Kp, Kd | vel = 0, Kd | t_ff = 0,0
            # self.md.setTargetVelocity(v_des)
            # self.md.setTargetTorque(t_des)
            #  Uncomment the line below to change the *.cfg file defaults
            # candle.md80s[0].setPositionControllerParams(20.0, 0.2, 0.0, 15.0)
            # candle.md80s[0].setVelocityControllerParams(0.05, 0.25, 0.0, 1.0)
            # candle.md80s[0].setMaxTorque(0.5)
            # torque parameters Kd, Kp are not configurable
            # self.md.setMaxTorque(10)
            # self.md.setVelocityControllerParams(5, 4, 0.0, 1.0)   #kd !=0 leads to worrying noise from motor
            
            # self.md.setImpedanceControllerParams(0.0, 1.0) #control in: pos = Kp, Kd | vel = 0, Kd | t_ff = 0,0
            # self.md.setTargetPosition(p_des)
            # self.md.setTargetVelocity(v_des)
            # self.md.setTargetTorque(t_des)
           


            if (np.abs(v_cur) > 30.0):
                warnings.warn("Cutoff velocity exceeded, exiting...")
                CUR_STATE = STATE_EXITING

            t = t + 1.0 / self.fs
            time.sleep(max(next_time_instant - time.perf_counter(), 0))

        print("Disabling motor!")
        self.md.setTargetTorque(0)

class readImuLoop(threading.Thread):
    def __init__(self,name, sys_state, Qwiicfunction, imu_ind):#,sys_state):
        threading.Thread.__init__(self)
        self.sys_state = sys_state
        self.name = name
        self.imu_fs = 100 # IMU updates at 100Hz, no point in reading any faster
        self.running = 	True
        self.imu_ind = imu_ind
        self.Qwiicfunction = Qwiicfunction
        self.stance = 0.0
        self.HS = 0.0

    def stop(self):
        self.running = 	False

    def run(self):
        global CUR_STATE
        print(f"start IMU {self.imu_ind}")
        imu = Imu(self.name,self.imu_fs)
        imu.initialize(self.Qwiicfunction)

        buffer = np.zeros(10)
        heel_strike_bool = False
        toe_off_bool = True
        heel_strike = 0.0
        toe_off = 0.0
        imu_multiplier = -1 if self.imu_ind == 1 else 1     # to account for imu measuring in different directions
        not_function = lambda x: 0 if x == 1 else 1
        while CUR_STATE != STATE_EXITING:
                        
            next_time_instant = time.perf_counter() + (1.0 / self.imu_fs)
            
            IMU = imu.read_imu()            

            buffer = np.roll(buffer, -1)
            buffer[-1] = -IMU.gzfilt * imu_multiplier
            
            # if np.all(buffer[:-1] <= 0) and (buffer[-1] > 0) and (buffer[0]-buffer[-1] <= -30) and (toe_off_bool == True):
            #     heel_strike = time.perf_counter()
            #     heel_strike_bool = True
            #     toe_off_bool = False
            
            # # "30" is difference in rise, "0.5" is minimum time between HS and TO.
            # if np.all(buffer[:-1] >= 0) and buffer[-1] and (buffer[0]-buffer[-1] >= 30) and (heel_strike_bool == True) and (time.perf_counter() - heel_strike > 0.5):
            #     toe_off = time.perf_counter()
            #     heel_strike_bool = False
            #     toe_off_bool = True
            
            # # opposite leg heel-strike acts as a toe-off
            # # if self.sys_state.imu_states[not(self.imu_ind)].HS == True:
            # #     heel_strike_bool = False
            # #     toe_off_bool = True
            
            # if (heel_strike_bool == True) and (toe_off_bool == False): 
            #     self.stance = 1
            # else:
            #     self.stance = 0
            
            if np.all(buffer[:-1] <= 0) and (buffer[-1] > 0) and (buffer[0]-buffer[-1] <= -30):
                self.HS = True

            if self.HS == True:
                self.stance = True
                
            if (self.stance == True) and (self.sys_state.imu_states[not(self.imu_ind)].HS == True):
                self.stance = False
                
            with LOCK:
                self.sys_state.imu_states[self.imu_ind].gzfilt = -IMU.gzfilt
                self.sys_state.imu_states[self.imu_ind].pitch = IMU.pitch
                self.sys_state.imu_states[self.imu_ind].stance = self.stance
                self.sys_state.imu_states[self.imu_ind].HS = self.HS
                self.sys_state.imu_states[self.imu_ind].TO = toe_off_bool

            if self.HS == True:     # this statement is placed after the lock to ensure the other leg "sees" HS = True
                self.HS = False


            time.sleep(max(next_time_instant-time.perf_counter(),0))
        print("Stopping imu thread.")

class readLoadCellLoop(threading.Thread):
    def __init__(self, name, sys_state, lc_ind):
        threading.Thread.__init__(self)
        self.name = name
        self.sys_state = sys_state
        self.load_cell_fs = 100
        self.lc_ind = lc_ind
        self.running = 	True
    
    def stop(self):
        self.running = 	False

    def run(self):
        global CUR_STATE
        print("Starting load cell reading...")
        load_cell = loadCell(LC_OFFSET, LC_GAIN)
        load_cell.ADC_initialize()
        count = 0
        while CUR_STATE != STATE_EXITING:
            next_time_instant = time.perf_counter() + (1.0 / self.load_cell_fs)
            force = load_cell.read_force()

            # save force
            with LOCK:
                self.sys_state.loadcell_states[self.lc_ind].force = force
            
            time.sleep(max(next_time_instant-time.perf_counter(),0))
        print("Stopping load cell thread.")

class saveDataLoop(threading.Thread):
    def __init__(self, name, sys_state):
        threading.Thread.__init__(self)
        self.name = name
        self.sys_state = sys_state
        self.fs = FS

    def run(self):
        global CUR_STATE

        t0 = time.perf_counter()
        f_out = open(filename, "w")

        file_header = "t,"
        file_header = file_header + "p_des_m1,p_cur_m1,"
        file_header = file_header + "v_des_m1,v_cur_m1,"
        file_header = file_header + "t_des_m1,t_cur_m1,"
        file_header = file_header + "th1,th1_d,"
        file_header = file_header + "gzfilt_S0,pitch_S0,"
        file_header = file_header + "pretension_m1,stance_S0,"
        file_header = file_header + "HS_S0,TO_S0,"
        file_header = file_header + "p_des_m2,p_cur_m2,"
        file_header = file_header + "v_des_m2,v_cur_m2,"
        file_header = file_header + "t_des_m2,t_cur_m2,"
        file_header = file_header + "th2,th2_d,"
        file_header = file_header + "gzfilt_S1,pitch_S1,"
        file_header = file_header + "pretension_m2,stance_S1,"
        file_header = file_header + "HS_S1,TO_S1,"
        file_header = file_header + "force_S1\n"
        f_out.write(file_header)

        while CUR_STATE != STATE_EXITING:
            next_time_instant = time.perf_counter() + (1.0 / self.fs)
            t = time.perf_counter() - t0

            with LOCK:
                data = "{:.5f},".format(t)
                data = data + "{:.3f},{:.3f},".format(self.sys_state.motor_states[0].p_des, 
                                                      self.sys_state.motor_states[0].p_cur)
                data = data + "{:.3f},{:.3f},".format(self.sys_state.motor_states[0].v_des, 
                                                      self.sys_state.motor_states[0].v_cur)
                data = data + "{:.3f},{:.3f},".format(self.sys_state.motor_states[0].t_des, 
                                                      self.sys_state.motor_states[0].t_cur)
                data = data + "{:.3f},{:.3f},".format(self.sys_state.imu_states[0].th_cur,
                                                      self.sys_state.imu_states[0].thd_cur)
                data = data + "{:.3f},{:.3f},".format(self.sys_state.imu_states[0].gzfilt,
                                                      self.sys_state.imu_states[0].pitch)
                data = data + "{:.3f},{:.3f},".format(self.sys_state.motor_states[0].pretension,
                                                      self.sys_state.imu_states[0].stance)
                data = data + "{:.3f},{:.3f},".format(self.sys_state.imu_states[0].HS,
                                                      self.sys_state.imu_states[0].TO)
                data = data + "{:.3f},{:.3f},".format(self.sys_state.motor_states[1].p_des, 
                                                      self.sys_state.motor_states[1].p_cur)
                data = data + "{:.3f},{:.3f},".format(self.sys_state.motor_states[1].v_des, 
                                                      self.sys_state.motor_states[1].v_cur)
                data = data + "{:.3f},{:.3f},".format(self.sys_state.motor_states[1].t_des, 
                                                      self.sys_state.motor_states[1].t_cur)
                data = data + "{:.3f},{:.3f},".format(self.sys_state.imu_states[1].th_cur,
                                                       self.sys_state.imu_states[1].thd_cur)
                data = data + "{:.3f},{:.3f},".format(self.sys_state.imu_states[1].gzfilt,
                                                      self.sys_state.imu_states[1].pitch)
                data = data + "{:.3f},{:.3f},".format(self.sys_state.motor_states[1].pretension,
                                                      self.sys_state.imu_states[1].stance)
                data = data + "{:.3f},{:.3f},".format(self.sys_state.imu_states[1].HS,
                                                      self.sys_state.imu_states[1].TO)
                data = data + "{:.3f}\n".format(self.sys_state.loadcell_states[0].force)
                
            f_out.write(data)
            
            
            print("\033[K", end='')
            print(
                "P0: {:+5.2f}".format(self.sys_state.motor_states[0].p_cur) + 
                "|V0: {:+6.2f}".format(self.sys_state.motor_states[0].v_cur) +
                "|T0: {:+5.2f}".format(self.sys_state.motor_states[0].t_cur) +
                "|I0: {:+5.2f}".format(self.sys_state.imu_states[0].gzfilt) +
                "|S0: {:+5.2f}".format(self.sys_state.imu_states[0].stance) +
                "||P2: {:+5.2f}".format(self.sys_state.motor_states[1].p_cur) +
                "|V1: {:+6.2f}".format(self.sys_state.motor_states[1].v_cur) +
                "|T1: {:+5.2f}".format(self.sys_state.motor_states[1].t_cur) +
                "|I1: {:+5.2f}".format(self.sys_state.imu_states[1].gzfilt) +
                "|S1: {:+5.2f}".format(self.sys_state.imu_states[1].stance) +
                "|L0: {:+5.2f}".format(self.sys_state.loadcell_states[0].force),
                end='\r', flush=True)
            
            time.sleep(max(next_time_instant - time.perf_counter(), 0))
        
        print("Saving Log File!")
        f_out.close()

def main():
    
    system_state = systemState(n_motors = 2, n_imus = 2, n_loadcell = 1)

    # Candle Motor Setup
    candle = pyCandle.Candle(pyCandle.CAN_BAUD_8M, True, pyCandle.SPI)
    ids = candle.ping(pyCandle.CAN_BAUD_8M)
    
    if len(ids) == 0:
        sys.exit("NO DRIVES FOUND")

    for id in ids:
        candle.addMd80(id)
        time.sleep(1) # The motors don't like being added too quickly...


    for md in candle.md80s:
        candle.controlMd80SetEncoderZero(md)
    # select mode:
        # candle.controlMd80Mode(md, pyCandle.RAW_TORQUE)
        # candle.controlMd80Mode(md, pyCandle.VELOCITY_PID)
        candle.controlMd80Mode(md, pyCandle.IMPEDANCE)    # Set mode to impedance control
        # candle.controlMd80Mode(ids[0], pyCandle.POSITION_PID)
        
        candle.controlMd80Enable(md, True)
        
    candle.begin()

    # Thread Setup
    saveDataThread = saveDataLoop("Save data", system_state)
    leftMotorControlThread = motorControlLoop("Motor control left", system_state, candle.md80s[0], 0, 0)
    rightMotorControlThread = motorControlLoop("Motor control right", system_state, candle.md80s[1], 1, 1) 
    right_readImuThread = readImuLoop("Read IMU Right Shank", system_state, qwiic_icm20948.QwiicIcm20948_S1(), 0)
    left_readImuThread = readImuLoop("Read IMU Left Shank", system_state, qwiic_icm20948.QwiicIcm20948_S2(),1)
    readLoadCellThread = readLoadCellLoop("Read load cell", system_state, 0)
    
    threads = []
    threads.append(saveDataThread)
    threads.append(leftMotorControlThread)
    threads.append(rightMotorControlThread)
    threads.append(right_readImuThread)
    threads.append(left_readImuThread)  
    threads.append(readLoadCellThread)


    for t in threads:
        t.start()

    for t in threads:
        t.join()
    
if __name__ == "__main__":
    default_handler = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, handler)

    try:
        main()
    except:
        sys.exit("MAIN DEAD")
        

