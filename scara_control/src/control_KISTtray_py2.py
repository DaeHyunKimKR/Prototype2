import os, sys

class KISTtray():
    def __init__(self):
        self.__node_id = 5
        
        self._bps_Set      = self.enc_packet([0x07, 0x0D])

        self._Nom_Vel_Set  = self.enc_packet([0x09, 0x19, 0x96])
        
        self._Vel_Ctrl_Set = self.enc_packet([0x05, 0xFE, 0xFE, 0xFE, 0x20])

        self._set_dir      = self.enc_packet([0xED, 0x0E, 0x01])

        self._vel_cmd_dir0 = self.enc_packet([0x03, 0x00, 0xF6, 0x05, 0x0A])

        self._vel_cmd_dir1 = self.enc_packet([0x03, 0x01, 0xF6, 0x05, 0x0A])

        self._vel_cmd_zero = self.enc_packet([0x03, 0x00, 0x00, 0x00, 0x14])

        self._Resol_Set    = self.enc_packet([0x0A, 0x04, 0x00])

        self._Reset        = self.enc_packet([0x0D])    
        
        self._control_off      = self.enc_packet([0x0C, 0x01])

        self._control_on       = self.enc_packet([0x0C, 0x00])
        
        self._set_id       = [0xFF, 0xFE, 0x00, 0x03, 0xF1, 0x06, 0x05]

    def enc_packet(self, Val):
        packet = bytearray(5+len(Val))
        packet[0] = 0xFF
        packet[1] = 0xFE 
        packet[2] = self.__node_id
        packet[3] = len(Val)+1
        packet[4] = 0
        Val_idx = 0
        for i in Val:
            packet[5+Val_idx] = i
            Val_idx +=1
        sum = 0
        for b in packet[2:]:
            sum = sum + b
        packet[4] = 0xFF - (sum % 256)
        return packet

    def CMD_packet(self, cmd):
        if cmd == 'set_bps':
            ret_packet = self._bps_Set
        elif cmd == 'set_rated_vel':
            ret_packet = self._Nom_Vel_Set
        elif cmd == 'set_vel_controller':
            ret_packet = self._Vel_Ctrl_Set
        elif cmd == 'set_direction':
            ret_packet = self._set_dir
        elif cmd == 'cmd_vel_dir0':
            ret_packet = self._vel_cmd_dir0
        elif cmd == 'cmd_vel_dir1':
            ret_packet = self._vel_cmd_dir1
        elif cmd == 'cmd_vel_zero':
            ret_packet = self._vel_cmd_zero
        elif cmd == 'set_resolution':        
            ret_packet = self._Resol_Set
        elif cmd == 'reset':
            ret_packet = self._Reset
        elif cmd == 'control_off':
            ret_packet = self._control_off
        elif cmd == 'control_on':
            ret_packet = self._control_on
        elif cmd == 'set_id':
            ret_packet = self._set_id
        else:
            print('error (kist tray) wrong command')
            ret_packet = ''

        return ret_packet

