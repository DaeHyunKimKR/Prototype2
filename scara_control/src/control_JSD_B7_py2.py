import os, sys
import serial
import time
import math
import struct

#%%
class JSD_B7:
# Initialization and Parameter setting ----------------------------------------
    def __init__(self, device_id=0, pulse_per_rotation=1, gear_ratio=1, pitch = 1, home_pulse = 0, direction = 1):
        self.__ID = device_id
        self.__UPR = pulse_per_rotation
        self.__GR = gear_ratio
        self.__PT = pitch
        self.__R2U = self.__R2U__()
        self.__D2R = math.pi/180
        self.__M2D = 360/self.__PT
        self.__HOME = home_pulse
        self.__DIR = direction
        
        self.__FLAG_SV = 0
        
        self.__w_pos_U = 0
        self.__w_pos_R = 0
        self.__w_pos_D = 0
        self.__w_pos_M = 0
        
        self.__w_vel_U = 0
        self.__w_vel_R = 0
        self.__w_vel_D = 0
        self.__w_vel_M = 0
        
        self.__r_pos_U = 0
        self.__r_pos_R = 0
        self.__r_pos_D = 0
        self.__r_pos_M = 0
        
        self.__r_vel_U = 0
        self.__r_vel_R = 0
        self.__r_vel_D = 0
        self.__r_vel_M = 0

        #####
        self.check = 0


    def __R2U__(self):
        return round(self.__UPR*self.__GR / (2*math.pi) *20)/20
    
    def set_device_ID(self, input_id):
        self.__ID = input_id
        
    def get_device_ID(self):
        return self.__ID
        
    def set_pulse_per_rotation(self, pulse_per_rotation=1):
        self.__UPR = pulse_per_rotation
        self.__R2U = self.__R2U__()
        
    def get_pulse_per_rotation(self):
        return self.__UPR
    
    def set_gear_ratio(self, gear_ratio=1):
        self.__GR = gear_ratio
        self.__R2U = self.__R2U__()
        
    def get_gear_ratio(self):
        return self.__GR
        
    def set_pitch(self, pitch=1):
        self.__PT = pitch
        self.__M2D = 360/self.__PT
        
    def get_pitch(self):
        return self.__PT
    
    def set_home_pulse(self, home_pulse = 0):
        self.__HOME = home_pulse
        
    def get_home_pulse(self):
        return self.__HOME
    
    def set_direction(self, direction = 1):
        self.__DIR = direction
        
    def get_direction(self):
        return self.__DIR
    
#%% Device reading --------------------------------------------------------------
    def dec_packet(self, packet):
        packet_struct = '<'
        for i in range(len(packet)):
            packet_struct = packet_struct + 'B'
        packet = struct.unpack(packet_struct, packet)
        # header = packet[0]
        sum = 0
        for b in packet[1:-1]:
            sum = sum + b
        if packet[-1] == sum % 256:
            Add = packet[2]
            Val = 0
            Val_len = packet[3]
            for i in range(Val_len):
                Val = Val + packet[4+i]*(256**i)
            if Val > (256**Val_len)/2:
               Val = 256**Val_len - Val
            if Add == 16:
                self.set_cur_pos(Val)
            elif Add == 12:
                self.set_cur_vel(Val)

                
        # return [Add, Val]

#%% Write device
# Device control --------------------------------------------------------------
    def enc_packet(self, Add, Val):
        packet = bytearray(5+len(Val))
        packet[0] = 150
        packet[1] = self.__ID 
        packet[2] = Add
        packet[3] = len(Val)
        Val_idx = 0
        for i in Val:
            packet[4+Val_idx] = i
            Val_idx +=1
        sum = 0
        for b in packet[1:]:
            sum = sum + b
        packet[4+len(Val)] = sum % 256
        return packet

    def CMD_SV_ON(self):
        self.__FLAG_SV = 1
        return self.enc_packet(80, [1,0])
    
    def CMD_SV_OFF(self):
        self.__FLAG_SV = 0
        return self.enc_packet(80, [0,0])
    
    def CMD_STOP(self):
        return self.enc_packet(80, [8,0])
    
    def CMD_STOP_EMG(self):
        return self.enc_packet(80, [16,0])
    
    def CMD_ALARM_RESET(self):
        return self.enc_packet(84, [0,1])
    
    def CMD_cur_sv_stat(self):
        return self.enc_packet(10, [])
    
    def CMD_cur_pos_pul(self):
        packet = self.enc_packet(16, [])
        packet[3] = 1
        packet[4] = packet[4]+1
        return packet
    
    def CMD_cur_vel_rpm(self):
        return self.enc_packet(12, [])
    
# Motion control --------------------------------------------------------------
    def set_tar_pos_pul(self, position_U=0):
        self.__w_pos_R = position_U/self.__R2U
        self.__w_pos_D = self.__w_pos_R/self.__D2R
        self.__w_pos_M = self.__w_pos_D/self.__M2D
        self.__w_pos_U = round(position_U)
        p_val_buf = self.__HOME + self.__w_pos_U*self.__DIR
        if p_val_buf < 0:
            p_val_buf = 256**4 + p_val_buf
        p_val = [0,0,0,0]
        for i in range(len(p_val)):
            p_val[i] = int(p_val_buf % (256**(i+1)))>>(8*i)
        return self.enc_packet(34, p_val)

    def set_tar_pos_rad(self, position_R=0):
        return self.set_tar_pos_pul(position_R*self.__R2U)

    def set_tar_pos_deg(self, position_D=0):
        return self.set_tar_pos_rad(position_D*self.__D2R)
    
    def set_tar_pos_mm(self, position_M=0):
        return self.set_tar_pos_deg(position_M*self.__M2D)
    
    def get_tar_pos_pul(self):
        return self.__w_pos_U
    
    def get_tar_pos_rad(self):
        return self.__w_pos_R

    def get_tar_pos_deg(self):
        return self.__w_pos_D
    
    def get_tar_pos_mm(self):
        return self.__w_pos_M
    
    def set_tar_vel_rpm(self, velocity_U=0):
        self.__w_vel_D = velocity_U*6/self.__GR
        self.__w_vel_R = self.__w_vel_D*self.__D2R
        self.__w_vel_M = self.__w_vel_D/self.__M2D
        self.__w_vel_U = round(velocity_U)
        p_val_buf = self.__w_vel_U        
        if p_val_buf < 0:
            p_val_buf = 256**2 + p_val_buf
        p_val = [0,0]
        for i in range(len(p_val)):
            p_val[i] = int(p_val_buf % (256**(i+1)))>>(8*i)
        return self.enc_packet(30, p_val)

    def set_tar_vel_deg(self, velocity_D):
        return self.set_tar_vel_rpm(velocity_D*self.__GR/6)
    
    def set_tar_vel_rad(self, velocity_R):
        return self.set_tar_vel_deg(velocity_R/self.__D2R)
    
    def set_tar_vel_mm(self, velocity_M):
        return self.set_tar_vel_deg(velocity_M*self.__M2D)

    def get_tar_vel_rpm(self):
        return self.__w_vel_U
    
    def get_tar_vel_deg(self):
        return self.__w_vel_D

    def get_tar_vel_rad(self):
        return self.__w_vel_R

    def get_tar_vel_mm(self):
        return self.__w_vel_M
    
    def set_cur_pos(self, position_U):
        self.__r_pos_U = (position_U - self.__HOME)*self.__DIR 
        self.__r_pos_R = self.__r_pos_U/self.__R2U
        self.__r_pos_D = self.__r_pos_R/self.__D2R
        self.__r_pos_M = self.__r_pos_D/self.__M2D
        
    def set_cur_vel(self, velocity_U):
        self.__r_vel_U = velocity_U
        self.__r_vel_D = self.__r_vel_U*6/self.__GR
        self.__r_vel_R = self.__r_vel_D*self.__D2R
        self.__r_vel_M = self.__r_vel_D/self.__M2D
        
    def get_cur_pos_pul(self):
        return self.__r_pos_U
    
    def get_cur_pos_rad(self):
        return self.__r_pos_R
    
    def get_cur_pos_deg(self):
        return self.__r_pos_D
    
    def get_cur_pos_mm(self):
        return self.__r_pos_M
    
    def get_cur_vel_rpm(self):
        return self.__r_vel_U
    
    def get_cur_vel_rad(self):
        return self.__r_vel_R
    
    def get_cur_vel_deg(self):
        return self.__r_vel_D
    
    def get_cur_vel_mm(self):
        return self.__r_vel_M


# 
#    def main(self):
#     
#     self.check = self.check + 1
# 
#     print(self.check)
# 
#     
#     if self.check == 100:
#         print(time)
#         break; 
#     
# 
# if __name__=="__main__":
#     main()
#     
    
    
    
