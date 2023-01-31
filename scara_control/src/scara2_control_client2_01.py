 # -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import String, Bool
from scara_control.msg import Scara_cmd, Scara_status
import time

from control_NBSCARA_py2 import NBSCARA as module_SCARA


# Scara arm
class Scara_Client():
    def __init__(self):
        rospy.init_node('Scara_Client2')
        self.scara_status_pub = rospy.Publisher('/scara_status2', Scara_status, queue_size=100)
        self.scara_cmd = None
        self.scara_status = Scara_status()

        rospy.loginfo(":: Scara Client2 Started ::")

        self.__list_node  = [6,7,8,9,10]
        self.__list_joint = [1,2,3,4]
        self.__list_joint_P = [1]
        self.__list_joint_R = [2,3,4]
        self.__list_device = [5]

        self.__PORT = '/dev/SCARA1'
        self.__home_pulse = [0, -1060824, 96804, 5355262]
        self.scara = module_SCARA(port = self.__PORT, home_pulse = self.__home_pulse)
        self.scara.CMD_ALARM_RESET()
        
        self.__cur_pos_deg = [0 for i in range(len(self.__list_joint))]
        self.__cur_pos_pul = [0 for i in range(len(self.__list_joint))]
        
        self.__cur_vel = [0 for i in range(len(self.__list_joint))]
        
        self.__tar_pos_deg  = [0 for i in range(len(self.__list_joint))]
        self.__tar_pos_pul  = [0 for i in range(len(self.__list_joint))]
        
        self.__tar_vel  = [0 for i in range(len(self.__list_joint))]
        
        self.__tar_time = [0 for i in range(len(self.__list_joint))]
        
        self.list_dist1 = []
        self.list_ang1 = []
        self.list_ang2 = []
        self.list_ang3 = []

        self.j_time_list = []
        self.j_motion_time_list = []

        self.pub_scara_status()

    def msg_callback(self, data):
        self.scara_cmd = data

    def get_scara_cmd(self):
        ret_cmd = rospy.wait_for_message('/scara_control', Scara_cmd)
        return ret_cmd

    def get_joint_id(self, node):
        ret_list = [0 for i in range(len(node))]       
        for idx in range(len(node)):
            ret_list[idx] = node[idx] - self.__list_node[0] + 1
        return ret_list

    def updata_current_pos(self, node):
        self.scara.CMD_read_cur_pos(node)
        ret_pos_deg = self.scara.get_cur_pos_deg(node)
        ret_pos_pul = self.scara.get_cur_pos_pul(node)
        for idx in range(len(node)):
            self.__cur_pos_deg[node[idx]-1] = ret_pos_deg[idx]
            self.__cur_pos_pul[node[idx]-1] = ret_pos_pul[idx]
            
    def pub_scara_status(self):
        self.pub_status = self.scara_status
        # self.pub_status.servo = [status[0]['servo'],status[1]['servo'], status[2]['servo']]
        # self.pub_status.alarm = [status[0]['alarm'],status[1]['alarm'], status[2]['alarm']]
        # self.pub_status.warn = [status[0]['warn'],status[1]['warn'], status[2]['warn']]
        # self.pub_status.ready = [status[0]['ready'],status[1]['ready'], status[2]['ready']]
        # self.pub_status.inpos1 = [status[0]['inpos1'],status[1]['inpos1'], status[2]['inpos1']]
        # self.pub_status.zspd = [status[0]['zspd'],status[1]['zspd'], status[2]['zspd']]
        self.pub_status.current_joint = self.__cur_pos_deg
        self.pub_status.current_pulse = self.__cur_pos_pul
        self.scara_status_pub.publish(self.pub_status)

    def check_ready(self, node):
        # check_ready_list = []
        result = True
        return result

    def control(self):
        while True:
            recv_cmd = self.get_scara_cmd()
            recv_j_id = self.get_joint_id(recv_cmd.node) 
            if len(set(recv_cmd.node) & set(self.__list_node)) == 0:
                continue

            if recv_cmd == None:
                print("[SCARA::WARN] Command is None")
            else:
                if recv_cmd.cmd == "on":
                    self.scara.CMD_SV_ON(recv_j_id)
                    print("[SCARA::SV_ON]")
                    print(recv_cmd.node)
                    
                elif recv_cmd.cmd == "off":
                    self.scara.CMD_SV_OFF(recv_j_id)
                    print("[SCARA::SV_OFF]")
                    print(recv_cmd.node)
                    
                elif recv_cmd.cmd == "stop":
                    self.scara.CMD_STOP(recv_j_id)
                    print("[SCARA::STOP]")
                    print(recv_cmd.node)

                elif recv_cmd.cmd == "stop_emg":
                    self.scara.CMD_STOP_EMG(recv_j_id)
                    print("[SCARA::STOP_EMG]")
                    print(recv_cmd.node)

                elif recv_cmd.cmd == "reset_alarm":
                    self.scara.CMD_ALARM_RESET(recv_j_id)
                    print("[SCARA::RESET_ALARM]")
                    print(recv_cmd.node)

                elif recv_cmd.cmd == "status":
                    self.updata_current_pos(recv_j_id)
                    self.pub_scara_status()
                    print("[SCARA::UPDATA_STATUS]")
                    print(recv_cmd.node)
            
            #==============================================

                elif recv_cmd.cmd[:4] == "TRAY":
                    self.scara.CMD_KISTtray(recv_cmd.cmd[5:])
                    print("[SCARA::TRAY] " + recv_cmd.cmd[5:])
                    print(recv_cmd.node) 


            #==============================================
                elif recv_cmd.cmd == "joint":
                    print("[SCARA::CMD] JOINT")
                    self.updata_current_pos(recv_j_id)
                    self.pub_scara_status()
                    self.__tar_pos = [recv_cmd.dist1, recv_cmd.angle1, recv_cmd.angle2, recv_cmd.angle3]
                    for idx in range(len(recv_j_id)):
                        if  len(set([recv_j_id[idx]]) & set(self.__list_joint_P)) == 0:
                            self.scara.set_tar_pos_deg([recv_j_id[idx]], [self.__tar_pos[recv_j_id[idx]-1]])
                        else:
                            self.scara.set_tar_pos_mm([recv_j_id[idx]], [self.__tar_pos[recv_j_id[idx]-1]])
                    
                elif recv_cmd.cmd == "velocity":
                    print("[SCARA::CMD] VELOCITY")
                    self.updata_current_pos(recv_j_id)
                    self.pub_scara_status()
                    self.__tar_vel = [recv_cmd.angle1, recv_cmd.angle2, recv_cmd.angle3, recv_cmd.dist1]
                    for idx in range(len(recv_j_id)):
                        if  len(set([recv_j_id[idx]]) & set(self.__list_joint_P)) == 0:
                            self.scara.set_tar_vel_deg([recv_j_id[idx]], [self.__tar_vel[recv_j_id[idx]-1]])
                        else:
                            self.scara.set_tar_vel_mm([recv_j_id[idx]], [self.__tar_vel[recv_j_id[idx]-1]])

                    
                elif recv_cmd.cmd == "joint_time":
                    print("[SCARA::CMD] JOINT, Time")
                    self.updata_current_pos(recv_j_id)
                    self.pub_scara_status()
                    self.__tar_pos_deg = [recv_cmd.angle1, recv_cmd.angle2, recv_cmd.angle3, recv_cmd.dist1]
                    print(self.__tar_pos_deg)
                    tar_pos = {}
                    for idx in range(len(recv_j_id)):
                        tar_pos[idx] = self.__tar_pos_deg[recv_j_id[idx]-1]
                                      
                    if len(recv_cmd.time_list) == 0:
                        tar_time = [recv_cmd.time]
                    else:
                        tar_time = list(recv_cmd.time_list)
                    print(tar_time)
                    
                    self.scara.set_tar_pos_deg_time(recv_j_id, tar_pos, tar_time)
                
                else:
                    print("[SCARA::WARN] INVALID COMMAND")
                # ===============================

if __name__=='__main__':
    try:
        SC = Scara_Client()
        SC.control()


    except rospy.ROSInterruptException:
        pass
