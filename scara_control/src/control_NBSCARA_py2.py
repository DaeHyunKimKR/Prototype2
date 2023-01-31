import os, sys
import serial
import time
import math
import numpy as np
from control_JSD_B7_py2 import JSD_B7 as module_JSD_B7
from control_KISTtray_py2 import KISTtray as module_KISTtray

class NBSCARA:
    def __init__(self, port, home_pulse = [], joint_limit = []):
        self.__port = port
        self.__baudrate=115200
        self.__timeout=0.1
        self.__ser = serial.Serial(port = self.__port, baudrate = self.__baudrate, timeout = self.__timeout, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize = serial.EIGHTBITS) 
        print("connecting to serial port")
        
        self.__home_pulse = home_pulse
        self.__joint_limit = joint_limit
        
        self.__list_j_id = [ 1, 2, 3, 4]
        self.__list_c_id = [ 4, 1, 2, 3]
        
        self.__joint = {}
        self.__joint[self.__list_j_id[0]] = module_JSD_B7(device_id=self.__list_c_id[0], pulse_per_rotation=2**17, gear_ratio=40, pitch=1, home_pulse=home_pulse[0], direction =  1)
        self.__joint[self.__list_j_id[1]] = module_JSD_B7(device_id=self.__list_c_id[1], pulse_per_rotation=2**17, gear_ratio=40, pitch=1, home_pulse=home_pulse[1], direction =  1)
        self.__joint[self.__list_j_id[2]] = module_JSD_B7(device_id=self.__list_c_id[2], pulse_per_rotation=2**17, gear_ratio=40, pitch=1, home_pulse=home_pulse[2], direction = -1)
        self.__joint[self.__list_j_id[3]] = module_JSD_B7(device_id=self.__list_c_id[3], pulse_per_rotation=2**17, gear_ratio=40, pitch=1, home_pulse=home_pulse[3], direction = -1)
        
        self.__device = {}
        self.__device[0]=module_KISTtray()
                
        self.__r_pos_U = [0 for i in range(len(self.__list_j_id)+1)]
        self.__r_pos_R = [0 for i in range(len(self.__list_j_id)+1)]
        self.__r_pos_D = [0 for i in range(len(self.__list_j_id)+1)]
        self.__r_pos_M = [0 for i in range(len(self.__list_j_id)+1)]
        
        self.__r_vel_U = [0 for i in range(len(self.__list_j_id)+1)]
        self.__r_vel_R = [0 for i in range(len(self.__list_j_id)+1)]
        self.__r_vel_D = [0 for i in range(len(self.__list_j_id)+1)]
        self.__r_vel_M = [0 for i in range(len(self.__list_j_id)+1)]
        
    def connect(self, port='', baudrate='', timeout=''):
        if port     == '':    port        = self.__port
        if baudrate == '':    baudrate    = self.__baudrate
        if timeout  == '':    timeout     = self.__timeout
        self.__ser = serial.serial_for_url(port = port, baudrat = baudrate, timeout = timeout, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize = serial.EIGHTBITS) 
        self.__port = PORT
        self.__baudrate = baudrate
        self.__timeout = timeout
        print("connecting to serial port")
        
    def disconnect(self):
        self.__ser.close()
        print("diconnecting to serial port")
    
    def CMD_ALARM_RESET(self, j_id = [0]):
        if j_id == [0]:
            j_id = self.__list_j_id

        if len(j_id) > len(self.__list_j_id):
            print("error: number of input values")
        else:
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                for j_idx in j_id:
                    self.__ser.write(self.__joint[j_idx].CMD_ALARM_RESET())
                    time.sleep(0.01)
            else:
                print("input node({node}) is wrong node")    
    
    def CMD_SV_ON(self, j_id = [0]):
        if j_id == [0]:
            j_id = self.__list_j_id

        if len(j_id) > len(self.__list_j_id):
            print("error: number of input values")
        else:
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                for j_idx in j_id:
                    self.__ser.write(self.__joint[j_idx].CMD_SV_ON())
                    time.sleep(0.01)
            else:
                print("input node({node}) is wrong node")
            
    def CMD_SV_OFF(self, j_id = [0]):
        if j_id == [0]:
            j_id = self.__list_j_id

        if len(j_id) > len(self.__list_j_id):
            print("error: number of input values")
        else:
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                for j_idx in j_id:
                    self.__ser.write(self.__joint[j_idx].CMD_SV_OFF())
                    time.sleep(0.01)
            else:
                print("input node({node}) is wrong node")
    
    def CMD_STOP(self, j_id = [0]):
        if j_id == [0]:
            j_id = self.__list_j_id

        if len(j_id) > len(self.__list_j_id):
            print("error: number of input values")
        else:
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                for j_idx in j_id:
                    self.__ser.write(self.__joint[j_idx].CMD_STOP())
                    time.sleep(0.01)
            else:
                print("input node({node}) is wrong node")
    
    def CMD_STOP_EMG(self, j_id = [0]):
        if j_id == [0]:
            j_id = self.__list_j_id

        if len(j_id) > len(self.__list_j_id):
            print("error: number of input values")
        else:
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                for j_idx in j_id:
                    self.__ser.write(self.__joint[j_idx].CMD_STOP_EMG())
                    time.sleep(0.01)
            else:
                print("input node({node}) is wrong node")
               
    def CMD_read_cur_pos(self, j_id = [0]):
        # log = []
        if j_id == [0]:
            j_id = self.__list_j_id

        if len(j_id) > len(self.__list_j_id):
            print("error: number of input values")
        else:
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                for j_idx in j_id:
                    # print(j_idx)
                    self.__ser.flush()
                    start_time = time.time()
                    self.__ser.write(self.__joint[j_idx].CMD_cur_pos_pul())
                    time.sleep(0.001)
                    mid_time = time.time()
                    packet = ''
                    for i in range(9):
                        for c in self.__ser.read():
                            packet = packet + c

                    end_time = time.time()
                    print("time1: ",mid_time - start_time)
                    print("time2: ",end_time - mid_time)
                    self.__joint[j_idx].dec_packet(packet)
                    self.__r_pos_U[j_idx] = self.__joint[j_idx].get_cur_pos_pul()
                    self.__r_pos_R[j_idx] = self.__joint[j_idx].get_cur_pos_rad()
                    self.__r_pos_D[j_idx] = self.__joint[j_idx].get_cur_pos_deg()
                    self.__r_pos_M[j_idx] = self.__joint[j_idx].get_cur_pos_mm()
                    # log.append(self.__r_pos_D)
                         
            else:
                print("input node({node}) is wrong node")
                
    def get_cur_pos_pul(self, j_id = [0]):
        if j_id == [0]:
            j_id = self.__list_j_id

        if len(j_id) > len(self.__list_j_id):
            print("error: number of input values")
        else:
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                __list_ret = [0 for i in range(len(j_id))]
                for idx in range(len(j_id)):
                    j_idx = j_id[idx]
                    __list_ret[idx] = self.__r_pos_U[j_idx]
                return __list_ret
                
    def get_cur_pos_rad(self, j_id = [0]):
        if j_id == [0]:
            j_id = self.__list_j_id

        if len(j_id) > len(self.__list_j_id):
            print("error: number of input values")
        else:
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                __list_ret = [0 for i in range(len(j_id))]
                for idx in range(len(j_id)):
                    j_idx = j_id[idx]
                    __list_ret[idx] = self.__r_pos_R[j_idx]
                return __list_ret
            
    def get_cur_pos_deg(self, j_id = [0]):
        if j_id == [0]:
            j_id = self.__list_j_id

        if len(j_id) > len(self.__list_j_id):
            print("error: number of input values")
        else:
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                __list_ret = [0 for i in range(len(j_id))]
                for idx in range(len(j_id)):
                    j_idx = j_id[idx]
                    __list_ret[idx] = self.__r_pos_D[j_idx]
                return __list_ret
                
    def get_cur_pos_mm(self, j_id = [0]):
        if j_id == [0]:
            j_id = self.__list_j_id

        if len(j_id) > len(self.__list_j_id):
            print("error: number of input values")
        else:
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                __list_ret = [0 for i in range(len(j_id))]
                for idx in range(len(j_id)):
                    j_idx = j_id[idx]
                    __list_ret[idx] = self.__r_pos_M[j_idx]
                return __list_ret
            
    def CMD_read_cur_vel(self, j_id = [0]):
        if j_id == [0]:
            j_id = self.__list_j_id

        if len(j_id) > len(self.__list_j_id):
            print("error: number of input values")
        else:
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                for idx in range(len(j_id)):
                    j_idx = j_id[idx]
                    print(j_idx)
                    self.__ser.flushInput()
                    self.__ser.write(self.__joint[j_idx].CMD_cur_vel_rpm())
                    time.sleep(0.001)
                    if self.__ser.readable():
                        packet = self.__ser.readall()
                        self.__joint[j_idx].dec_packet(packet)
                        self.__r_vel_U[j_idx] = self.__joint[j_idx].get_cur_vel_rpm()
                        self.__r_vel_R[j_idx] = self.__joint[j_idx].get_cur_vel_rad()
                        self.__r_vel_D[j_idx] = self.__joint[j_idx].get_cur_vel_deg()
                        self.__r_vel_M[j_idx] = self.__joint[j_idx].get_cur_vel_mm()
            else:
                print("input node({node}) is wrong node")

    def get_cur_vel_rpm(self, j_id = [0]):
        if j_id == [0]:
            j_id = self.__list_j_id

        if len(j_id) > len(self.__list_j_id):
            print("error: number of input values")
        else:
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                __list_ret = [0 for i in range(len(j_id))]
                for idx in range(len(j_id)):
                    j_idx = j_id[idx]
                    __list_ret[idx] = self.__r_vel_U[j_idx]
                return __list_ret
            
    def get_cur_vel_rad(self, j_id = [0]):
        if j_id == [0]:
            j_id = self.__list_j_id

        if len(j_id) > len(self.__list_j_id):
            print("error: number of input values")
        else:
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                __list_ret = [0 for i in range(len(j_id))]
                for idx in range(len(j_id)):
                    j_idx = j_id[idx]
                    __list_ret[idx] = self.__r_vel_R[j_idx]
                return __list_ret
            
    def get_cur_vel_deg(self, j_id = [0]):
        if j_id == [0]:
            j_id = self.__list_j_id

        if len(j_id) > len(self.__list_j_id):
            print("error: number of input values")
        else:
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                __list_ret = [0 for i in range(len(j_id))]
                for idx in range(len(j_id)):
                    j_idx = j_id[idx]
                    __list_ret[idx] = self.__r_vel_D[j_idx]
                return __list_ret
            
    def get_cur_vel_mm(self, j_id = [0]):
        if j_id == [0]:
            j_id = self.__list_j_id

        if len(j_id) > len(self.__list_j_id):
            print("error: number of input values")
        else:
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                __list_ret = [0 for i in range(len(j_id))]
                for idx in range(len(j_id)):
                    j_idx = j_id[idx]
                    __list_ret[idx] = self.__r_vel_M[j_idx]
                return __list_ret
            
    def set_tar_vel_rpm(self, j_id = [0], velocity_U = [0]):
        if j_id == [0]:
            j_id = self.__list_j_id
            
        if len(j_id) != len(velocity_U):
            print("error: number of input values(1=2)")
        elif len(j_id) > len(self.__list_j_id):
            print("error: number of input values(1)")
        else:    
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                for idx in range(len(j_id)):
                    self.__ser.write(self.__joint[j_id[idx]].set_tar_vel_rpm(velocity_U[idx]))
                    time.sleep(0.01)
            else:
                print("input node({node}) is wrong node")
                
    def set_tar_vel_rad(self, j_id = [0], velocity_R = [0]):
        if j_id == [0]:
            j_id = self.__list_j_id
            
        if len(j_id) != len(velocity_R):
            print("error: number of input values(1=2)")
        elif len(j_id) > len(self.__list_j_id):
            print("error: number of input values(1)")
        else:    
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                for idx in range(len(j_id)):
                    self.__ser.write(self.__joint[j_id[idx]].set_tar_vel_rad(velocity_R[idx]))
                    time.sleep(0.01)
            else:
                print("input node({node}) is wrong node")
                
    def set_tar_vel_deg(self, j_id = [0], velocity_D = [0]):
        if j_id == [0]:
            j_id = self.__list_j_id
            
        if len(j_id) != len(velocity_D):
            print("error: number of input values(1=2)")
        elif len(j_id) > len(self.__list_j_id):
            print("error: number of input values(1)")
        else:    
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                for idx in range(len(j_id)):
                    self.__ser.write(self.__joint[j_id[idx]].set_tar_vel_deg(velocity_D[idx]))
                    time.sleep(0.01)
            else:
                print("input node({node}) is wrong node")
                
    def set_tar_vel_mm(self, j_id = [0], velocity_M = [0]):
        if j_id == [0]:
            j_id = self.__list_j_id
            
        if len(j_id) != len(velocity_M):
            print("error: number of input values(1=2)")
        elif len(j_id) > len(self.__list_j_id):
            print("error: number of input values(1)")
        else:    
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                for idx in range(len(j_id)):
                    self.__ser.write(self.__joint[j_id[idx]].set_tar_vel_mm(velocity_M[idx]))
            else:
                print("input node({node}) is wrong node")
    
    def set_tar_pos_pul(self, j_id = [0], position_U = [0]):
        if j_id == [0]:
            j_id = self.__list_j_id
            
        if len(j_id) != len(position_U):
            print("error: number of input values(1=2)")
        elif len(j_id) > len(self.__list_j_id):
            print("error: number of input values(1)")
        else:    
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                for idx in range(len(j_id)):
                    self.__ser.write(self.__joint[j_id[idx]].set_tar_pos_pul(position_U[idx]))
            else:
                print("input node({node}) is wrong node")
                
    def set_tar_pos_rad(self, j_id = [0], position_R = [0]):
        if j_id == [0]:
            j_id = self.__list_j_id
            
        if len(j_id) != len(position_R):
            print("error: number of input values(1=2)")
        elif len(j_id) > len(self.__list_j_id):
            print("error: number of input values(1)")
        else:    
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                for idx in range(len(j_id)):
                    self.__ser.write(self.__joint[j_id[idx]].set_tar_pos_rad(position_R[idx]))
            else:
                print("input node({node}) is wrong node")
                
    def set_tar_pos_deg(self, j_id = [0], position_D = [0]):
        if j_id == [0]:
            j_id = self.__list_j_id
            
        if len(j_id) != len(position_D):
            print("error: number of input values(1=2)")
        elif len(j_id) > len(self.__list_j_id):
            print("error: number of input values(1)")
        else:    
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                for idx in range(len(j_id)):
                    self.__ser.write(self.__joint[j_id[idx]].set_tar_pos_deg(position_D[idx]))
            else:
                print("input node({node}) is wrong node")
                
    def set_tar_pos_mm(self, j_id = [0], position_M = [0]):
        if j_id == [0]:
            j_id = self.__list_j_id
            
        if len(j_id) != len(position_M):
            print("error: number of input values(1=2)")
        elif len(j_id) > len(self.__list_j_id):
            print("error: number of input values(1)")
        else:    
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                for idx in range(len(j_id)):
                    self.__ser.write(self.__joint[j_id[idx]].set_tar_pos_mm(position_M[idx]))
            else:
                print("input node({node}) is wrong node")
                
    def set_tar_pos_deg_time(self, j_id = [0], position_D = [0], time = [10]):
        if j_id == [0]:
            j_id = self.__list_j_id
            
        if len(j_id) != len(position_D):
            print("error: number of input values(1=2)")
        elif len(j_id) > len(self.__list_j_id):
            print("error: number of input values(1)")
        else:    
            if len(set(j_id) & set(self.__list_j_id)) > 0:
                velocity_D = {}
                for idx in range(len(j_id)):
                    if time[idx] < 0.1:
                        time[idx] = 0.1
                    velocity_D = abs(position_D[idx] - self.__r_pos_D[j_id[idx]])/time[idx]
                    if velocity_D > 180:
                        velocity_D = 180
                    print(velocity_D)
                    self.__ser.write(self.__joint[j_id[idx]].set_tar_vel_deg(velocity_D))
                    self.__ser.write(self.__joint[j_id[idx]].set_tar_pos_deg(position_D[idx]))
            else:
                print("input node({node}) is wrong node")
                
    def CMD_KISTtray(self, cmd):
        cmd_packet = self.__device[0].CMD_packet(cmd)
        if cmd_packet !='':
            self.__ser.write(cmd_packet)



# def main():

# if __name__=="__main__":
#     main()
    
    
    

