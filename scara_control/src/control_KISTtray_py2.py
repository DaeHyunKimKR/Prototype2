import os, sys

class KISTtray():
    def __init__(self):
        self.__node_id = 0
        
        self._bps_Set = {}
        self._bps_Set[0] = 0xFF
        self._bps_Set[1] = 0xFE
        self._bps_Set[2] = 0x00
        self._bps_Set[3] = 0x03
        self._bps_Set[4] = 0xE8
        self._bps_Set[5] = 0x07
        self._bps_Set[6] = 0x0D

        self._Nom_Vel_Set = {}
        self._Nom_Vel_Set[0] = 0xFF 
        self._Nom_Vel_Set[1] = 0xFE
        self._Nom_Vel_Set[2] = 0x00
        self._Nom_Vel_Set[3] = 0x04
        self._Nom_Vel_Set[4] = 0x43
        self._Nom_Vel_Set[5] = 0x09
        self._Nom_Vel_Set[6] = 0x19
        self._Nom_Vel_Set[7] = 0x96
        
        # self._Vel_Ctrl_Set = {}
        # self._Vel_Ctrl_Set[0] = 0xFF 
        # self._Vel_Ctrl_Set[1] = 0xFE
        # self._Vel_Ctrl_Set[2] = 0x00
        # self._Vel_Ctrl_Set[3] = 0x06
        # self._Vel_Ctrl_Set[4] = 0x96
        # self._Vel_Ctrl_Set[5] = 0x05
        # self._Vel_Ctrl_Set[6] = 0xFE
        # self._Vel_Ctrl_Set[7] = 0xFE
        # self._Vel_Ctrl_Set[8] = 0x4E
        # self._Vel_Ctrl_Set[9] = 0x04

        self._Vel_Ctrl_Set = {}
        self._Vel_Ctrl_Set[0] = 0xFF 
        self._Vel_Ctrl_Set[1] = 0xFE
        self._Vel_Ctrl_Set[2] = 0x00
        self._Vel_Ctrl_Set[3] = 0x06
        self._Vel_Ctrl_Set[4] = 0xDA
        self._Vel_Ctrl_Set[5] = 0x05
        self._Vel_Ctrl_Set[6] = 0xFE
        self._Vel_Ctrl_Set[7] = 0xFE
        self._Vel_Ctrl_Set[8] = 0xFE
        self._Vel_Ctrl_Set[9] = 0x20

        self._set_dir = {}
        self._set_dir[0] = 0xFF 
        self._set_dir[1] = 0xFE
        self._set_dir[2] = 0x00
        self._set_dir[3] = 0x03
        self._set_dir[4] = 0xED
        self._set_dir[5] = 0x0E
        self._set_dir[6] = 0x01

        self._vel_cmd_dir0 = {}
        self._vel_cmd_dir0[0] = 0xFF
        self._vel_cmd_dir0[1] = 0xFE
        self._vel_cmd_dir0[2] = 0x00
        self._vel_cmd_dir0[3] = 0x06
        self._vel_cmd_dir0[4] = 0xF1
        self._vel_cmd_dir0[5] = 0x03
        self._vel_cmd_dir0[6] = 0x00
        self._vel_cmd_dir0[7] = 0xF6
        self._vel_cmd_dir0[8] = 0x05
        self._vel_cmd_dir0[9] = 0x0A

        self._vel_cmd_dir1 = {}
        self._vel_cmd_dir1[0] = 0xFF
        self._vel_cmd_dir1[1] = 0xFE
        self._vel_cmd_dir1[2] = 0x00
        self._vel_cmd_dir1[3] = 0x06
        self._vel_cmd_dir1[4] = 0xF0
        self._vel_cmd_dir1[5] = 0x03
        self._vel_cmd_dir1[6] = 0x01
        self._vel_cmd_dir1[7] = 0xF6
        self._vel_cmd_dir1[8] = 0x05
        self._vel_cmd_dir1[9] = 0x0A

        self._vel_cmd_zero = {}
        self._vel_cmd_zero[0] = 0xFF 
        self._vel_cmd_zero[1] = 0xFE
        self._vel_cmd_zero[2] = 0x00
        self._vel_cmd_zero[3] = 0x06
        self._vel_cmd_zero[4] = 0xE2
        self._vel_cmd_zero[5] = 0x03
        self._vel_cmd_zero[6] = 0x00
        self._vel_cmd_zero[7] = 0x00
        self._vel_cmd_zero[8] = 0x00
        self._vel_cmd_zero[9] = 0x14

        self._Resol_Set = {}
        self._Resol_Set[0] = 0xFF
        self._Resol_Set[1] = 0xFE
        self._Resol_Set[2] = 0x00
        self._Resol_Set[3] = 0x04
        self._Resol_Set[4] = 0xEF
        self._Resol_Set[5] = 0x0A
        self._Resol_Set[6] = 0x04
        self._Resol_Set[7] = 0x00

        self._Reset = {}
        self._Reset[0] = 0xFF 
        self._Reset[1] = 0xFE
        self._Reset[2] = 0xFF
        self._Reset[3] = 0x02
        self._Reset[4] = 0xF1
        self._Reset[5] = 0x0D    
        
        self._con_off={}
        self._con_off[0] = 0xFF
        self._con_off[1] = 0xFE
        self._con_off[2] = 0x00
        self._con_off[3] = 0x03
        self._con_off[4] = 0xEF
        self._con_off[5] = 0x0C
        self._con_off[6] = 0x01

        self._con_on = {}
        self._con_on[0] = 0xFF
        self._con_on[1] = 0xFE
        self._con_on[2] = 0x00
        self._con_on[3] = 0x03
        self._con_on[4] = 0xF0
        self._con_on[5] = 0x0C
        self._con_on[6] = 0x00

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
            ret_packet = self._con_off
        elif cmd == 'control_on':
            ret_packet = self._con_on
        else:
            print('error (kist tray) wrong command')
            ret_packet = ''

        return ret_packet


#def main(self):



#if __name__=='__main__':
#    main()