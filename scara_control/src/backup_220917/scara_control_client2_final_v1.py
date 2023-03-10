# -*- coding:utf-8 -*-

import rospy
from scara_function2_final_v1 import Scara as scara_func
from scara_control.msg import Scara_cmd, Scara_status
from std_msgs.msg import String, Bool
import threading
import time

# Scara arm
class Scara_Client():
    def __init__(self):
        rospy.init_node('Scara_Client2')
        self.scara_status_pub = rospy.Publisher('/scara_status2', Scara_status, queue_size=100)
        self.scara_cmd = None
        self.Scara_status = Scara_status()
        self.scara_func = scara_func()
        rospy.loginfo(":: Scara Client Started ::")
        self.thread_pause = False
        self.init_pulse = [0.0, 0.0, 0.0]
        print("[SCARA::SET_INIT_ANGLE]")
        self.scara_func.index_setting([4,5,6])
        # ===for motion===
        self.x_list = []
        self.y_list = []
        self.p_time_list = []
        self.ang1_list = []
        self.ang2_list = []
        self.ang3_list = []
        self.j_motion_time_list = []
        self.j_time_list = []
        # ================
        self.pub_scara_status()
        print("init_done")

    def msg_callback(self, data):
        self.scara_cmd = data
    
    def get_scara_msg(self):
        data = rospy.wait_for_message('/scara_control', Scara_cmd)
        return data

    def pub_scara_status(self):
        while True:
            error_flag = False
            status = []
            position = []
            joint = []
            pulse = []
            node_id = [4,5,6]
            for i in node_id:
                try:
                    status.append(self.scara_func.read_status(i))
                    raw_pulse, _ = self.scara_func.read_position(i)
                except:
                    print("Position read error")
                    error_flag = True
                    break
                if i == 4:
                    deg = float(262144*80)/360
                else:
                    deg = float(262144*40)/360
                tune_pulse = raw_pulse - self.init_pulse[i-4]
                degree_val = round(tune_pulse/deg,2)
                joint.append(degree_val)
                pulse.append(raw_pulse)
            if None in status:
                error_flag = True
            if error_flag:
                continue
            else:
                self.pub_status = self.Scara_status
                self.pub_status.servo = [status[0]['servo'],status[1]['servo'], status[2]['servo']]
                self.pub_status.alarm = [status[0]['alarm'],status[1]['alarm'], status[2]['alarm']]
                self.pub_status.warn = [status[0]['warn'],status[1]['warn'], status[2]['warn']]
                self.pub_status.ready = [status[0]['ready'],status[1]['ready'], status[2]['ready']]
                self.pub_status.inpos1 = [status[0]['inpos1'],status[1]['inpos1'], status[2]['inpos1']]
                self.pub_status.zspd = [status[0]['zspd'],status[1]['zspd'], status[2]['zspd']]
                self.pub_status.current_joint = [joint[0], joint[1], joint[2]]
                self.pub_status.current_pulse = [pulse[0], pulse[1], pulse[2]]
                self.scara_status_pub.publish(self.pub_status)
                break

    def check_ready(self, node):
        check_ready_list = []
        result = True

        for i in node:
            if (self.pub_status.servo[i-4]==0 and  self.pub_status.alarm[i-4]==0 and self.pub_status.warn[i-4]==0
                and self.pub_status.ready[i-4]==1 and self.pub_status.inpos1[i-4]==1 and self.pub_status.zspd[i-4]==1):
                check_ready_list.append(True)
            else:
                check_ready_list.append(False)

        for i in check_ready_list:
            result = result and i
        return result

    def control(self):
        while True:
            print("test")
            recv_cmd = self.get_scara_msg()
            if recv_cmd == None:
                print("[SCARA::WARN] Command is None")
            else:
                if recv_cmd.cmd == "on":
                    node_id = recv_cmd.node
                    for i in node_id:
                        self.scara_func.sv_on(i)
                        self.scara_func.flush()
                    print("[SCARA::SV_ON]")

                elif recv_cmd.cmd == "off":
                    node_id = recv_cmd.node
                    for i in node_id:
                        self.scara_func.sv_off(i)
                        self.scara_func.flush()
                    print("[SCARA::SV_OFF]")

                elif recv_cmd.cmd == "status":
                    self.pub_scara_status()

                elif recv_cmd.cmd == "position":
                    target_position = [recv_cmd.x, recv_cmd.y]
                    th1, th2 = self.scara_func.inverse_calc(target_position[0], target_position[1])
                    th3 = -(th1 + th2)
                    target_angle = [round(th1, 3), round(th2, 3), round(th3, 3)]
                    limit_flag1 = True
                    limit_flag2 = True
                    
                    if abs(recv_cmd.angle1) > 126:
                        limit_flag1 = False
                        
                    limit_flag = limit_flag1

                    if limit_flag:
                        target_time = recv_cmd.time
                        node_id = recv_cmd.node
                        control_flag = self.check_ready(node_id)
                        if control_flag:
                            for i in node_id:
                                init_pulse = self.init_pulse[i-4]
                                raw_pulse = self.pub_status.current_pulse[i-4]
                                current_deg = self.pub_status.current_joint[i-4]
                                self.scara_func.joint_control(raw_pulse, init_pulse, current_deg, i, target_angle[i-4], target_time)
                            self.scara_func.start(0)
                            self.scara_func.flush()
                            t_start = time.time()
                            while True:
                                self.pub_scara_status()
                                inpos_result = True
                                for i in node_id:
                                    inpos = self.pub_status.inpos1[i-4]
                                    inpos_result = inpos_result and inpos
                                if inpos_result:
                                    t_finish = time.time()
                                    print(">> operating_time:"+str(t_finish-t_start))
                                    break
                            print("[SCARA::POSITION]")
                        else:
                            print("[SCARA::WARN] Driver is not ready")
                    else:
                        print("[SCARA::WARN] POSITION limit")
                
                elif recv_cmd.cmd == "joint":
                    self.pub_scara_status()           
                    target_angle = [recv_cmd.angle1, recv_cmd.angle2, recv_cmd.angle3]
                    if len(recv_cmd.time_list) == 0:
                        target_time = recv_cmd.time
                    else:
                        target_time = list(recv_cmd.time_list)

                    limit_flag1 = True
                    limit_flag2 = True
                    
                    if abs(recv_cmd.angle1) > 126:
                        limit_flag1 = False
                    if abs(recv_cmd.angle2) > 165:
                        limit_flag2 = False
                        
                    limit_flag = limit_flag1 and limit_flag2
#
                    if limit_flag:
                        node_id = recv_cmd.node
                        control_flag = self.check_ready(node_id)
                        if control_flag:
                            for i in node_id:
                                init_pulse = self.init_pulse[i-4]
                                raw_pulse = self.pub_status.current_pulse[i-4]
                                current_deg = self.pub_status.current_joint[i-4]
                                if type(target_time) is list:
                                    self.scara_func.joint_control(raw_pulse, init_pulse, current_deg, i, target_angle[i-4], target_time[i-4])
                                else:
                                    target_time = recv_cmd.time
                                    self.scara_func.joint_control(raw_pulse, init_pulse, current_deg, i, target_angle[i-4], target_time)
                            self.scara_func.start(0)
                            self.scara_func.flush()
                            t_start = time.time()
                            while True:
                                print("control while")
                                self.pub_scara_status()
                                inpos_result = True
                                for i in node_id:
                                    inpos = self.pub_status.inpos1[i-4]
                                    inpos_result = inpos_result and inpos
                                if inpos_result:
                                    t_finish = time.time()
                                    print(">> operating_time:" + str(t_finish-t_start))
                                    break
                            print("[SCARA::JOINT]")
                        else:
                            print("[SCARA::WARN] Driver is not ready")
                    else:
                        print("[SCARA::WARN] Joint limit")
                
                elif recv_cmd.cmd == "position_motion":
                    node_id = recv_cmd.node
                    self.x_list.append(recv_cmd.x)
                    self.y_list.append(recv_cmd.y)
                    self.p_time_list.append(recv_cmd.time)
                    
                    if recv_cmd.count != 0:
                        for count in range(recv_cmd.count):
                            print("-------------------------------------------------", count+1)
                            for i in range(len(self.x_list)):
                                print("----------------", i+1)
                                temp = 0
                                while True:
                                    self.pub_scara_status()
                                    control_flag = self.check_ready(node_id)
                                    if control_flag:
                                        temp += 1
                                        if temp > 5:
                                            break
                                
                                target_position = [self.x_list[i], self.y_list[i]]
                                th1, th2 = self.scara_func.inverse_calc(target_position[0], target_position[1])
                                th3 = -(th1 + th2)
                                target_angle = [round(th1, 3), round(th2, 3), round(th3, 3)]
                                limit_flag1 = True
                                limit_flag2 = True
                                
                                if abs(target_angle[0]) > 126:
                                    limit_flag1 = False
                                
                                if self.scara_func.check_workspace == False:
                                    limit_flag2 = False
                                    
                                limit_flag = limit_flag1 and limit_flag2
                                
                                if limit_flag:
                                    target_time = self.p_time_list[i]
                                    if control_flag:
                                        for j in node_id:
                                            init_pulse = self.init_pulse[j-4]
                                            raw_pulse = self.pub_status.current_pulse[j-4]
                                            current_deg = self.pub_status.current_joint[j-4]
                                            self.scara_func.joint_control(raw_pulse, init_pulse, current_deg, j, target_angle[j-4], target_time)
                                        self.scara_func.start(0)
                                        self.scara_func.flush()
                                        t_start = time.time()
                                        while True:
                                            self.pub_scara_status()
                                            inpos1 = self.pub_status.inpos1[0]
                                            inpos2 = self.pub_status.inpos1[1]
                                            inpos3 = self.pub_status.inpos1[2]
                                            inpos = inpos1 and inpos2 and inpos3
                                            if inpos:
                                                t_finish = time.time()
                                                print("operating_time:", str(t_finish-t_start))
                                                break
                                        print("[SCARA::POSITION]")
                                    else:
                                        print("[SCARA::WARN] Driver is not ready")
                                else:
                                    print("[SCARA::WARN] POSITION limit")
                        del self.x_list [:]
                        del self.y_list [:]
                        del self.p_time_list [:]

                elif recv_cmd.cmd == "joint_motion":
                    node_id = recv_cmd.node
                    self.ang1_list.append(recv_cmd.angle1)
                    self.ang2_list.append(recv_cmd.angle2)
                    self.ang3_list.append(recv_cmd.angle3)
                    if len(recv_cmd.time_list) == 0:
                        self.j_motion_time_list.append(recv_cmd.time)
                        print("append time")
                    else:
                        time_tupleTolist = list(recv_cmd.time_list)
                        self.j_motion_time_list.append(time_tupleTolist)
                        print("append time list")
                    
                    if recv_cmd.count != 0:
                        for count in range(recv_cmd.count):
                            print("-------------------------------------------------", count+1)
                            for i in range(len(self.ang1_list)):
                                print("----------------", i+1)
                                temp = 0
                                while True:
                                    self.pub_scara_status()
                                    control_flag = self.check_ready(node_id)
                                    if control_flag:
                                        temp += 1
                                        if temp > 5:
                                            break
                                
                                target_angle = [self.ang1_list[i], self.ang2_list[i], self.ang3_list[i]]
                                target_time = self.j_motion_time_list[i]

                                limit_flag1 = True
                                limit_flag2 = True
                                
                                if abs(self.ang1_list[i]) > 126:
                                    limit_flag1 = False
                                if abs(self.ang2_list[i]) > 165:
                                    limit_flag2 = False
                                    
                                limit_flag = limit_flag1 and limit_flag2
                                
                                if limit_flag:
                                    if control_flag:
                                        for j in node_id:
                                            init_pulse = self.init_pulse[j-4]
                                            raw_pulse = self.pub_status.current_pulse[j-4]
                                            current_deg = self.pub_status.current_joint[j-4]
                                            if type(target_time) is list:
                                                self.scara_func.joint_control(raw_pulse, init_pulse, current_deg, j, target_angle[j-4], target_time[j-4])
                                            else:
                                                self.scara_func.joint_control(raw_pulse, init_pulse, current_deg, j, target_angle[j-4], target_time)
                                        self.scara_func.start(0)
                                        self.scara_func.flush()
                                        t_start = time.time()
                                        while True:
                                            self.pub_scara_status()
                                            inpos1 = self.pub_status.inpos1[0]
                                            inpos2 = self.pub_status.inpos1[1]
                                            inpos3 = self.pub_status.inpos1[2]
                                            inpos = inpos1 and inpos2 and inpos3
                                            if inpos:
                                                t_finish = time.time()
                                                print("operating_time:" + str(t_finish-t_start))
                                                break
                                        print("[SCARA::JOINT]")
                                    else:
                                        print("[SCARA::WARN] Driver is not ready")
                                else:
                                    print("[SCARA::WARN] Joint limit")
                        del self.ang1_list [:]
                        del self.ang2_list [:]
                        del self.ang3_list [:]
                        del self.j_motion_time_list [:]

                elif recv_cmd.cmd == "joint_motion_test":
                    node_id = recv_cmd.node
                    self.ang1_list.append(recv_cmd.angle1)
                    self.ang2_list.append(recv_cmd.angle2)
                    self.ang3_list.append(recv_cmd.angle3)
                    if len(recv_cmd.time_list) == 0:
                        self.j_motion_time_list.append(recv_cmd.time)
                    else:
                        time_tupleTolist = list(recv_cmd.time_list)
                        self.j_motion_time_list.append(time_tupleTolist)

                    if recv_cmd.count != 0:
                        target_angle_list = [self.ang1_list, self.ang2_list, self.ang3_list]
                        joint_count = [0, 0, 0]
                        operating_count = [0, 0, 0]
                        joint_limit_flag = [True, True, True]
                        operating_flag = [True, True, True]
                        operating_done = False
                        while True:
                            for i in node_id:
                                for j in range(5):
                                    self.pub_scara_status()
                                if self.check_ready([i]) and operating_flag[i-4]:
                                    if operating_count[i-4] < recv_cmd.count:
                                        target_angle = target_angle_list[i-4][joint_count[i-4]]
                                        if type(self.j_motion_time_list[joint_count[i-4]]) is list:
                                            target_time = self.j_motion_time_list[joint_count[i-4]][i-4]
                                        else:
                                            target_time = self.j_motion_time_list[joint_count[i-4]]

                                        if i == 4:
                                            if abs(target_angle) > 126:
                                                joint_limit_flag[i-4] = False
                                                print("[SCARA::WARN] JOINT LIMIT")
                                            else:
                                                joint_limit_flag[i-4] = True
                                        elif i == 5:
                                            if abs(target_angle) > 140:
                                                joint_limit_flag[i-4] = False
                                                print("[SCARA::WARN] JOINT LIMIT")
                                            else:
                                                joint_limit_flag[i-4] = True

                                        if (joint_limit_flag[i-4] == True) and (self.pub_status.inpos1[i-4] == True):
                                            init_pulse = self.init_pulse[i-4]
                                            raw_pulse = self.pub_status.current_pulse[i-4]
                                            current_deg = self.pub_status.current_joint[i-4]

                                            self.scara_func.joint_control(raw_pulse, init_pulse, current_deg, i, target_angle, target_time)
                                            self.scara_func.start(0)
                                            self.scara_func.flush()
                                            joint_count[i-4] += 1
                                            if joint_count[i-4] == len(target_angle_list[i-4]):
                                                operating_count[i-4] += 1
                                                joint_count[i-4] = 0
                                                if operating_count[i-4] == recv_cmd.count:
                                                    operating_flag[i-4] = False
                                        # init_joint limit flag
                                        joint_limit_flag[i-4] = True
                                         
                            for i in node_id:
                                operating_done = operating_done or operating_flag[i-4] 
                                
                            if operating_done == False:
                                print("[SCARA::JOINT MOTION]")
                                break
                            # init_operating_done
                            operating_done = False
                        del self.ang1_list[:]
                        del self.ang2_list[:]
                        del self.ang3_list[:]
                        del self.j_motion_time_list[:]

                # ===============================
                elif recv_cmd.cmd == "stop":
                    node_id = recv_cmd.node
                    for i in node_id:
                        self.scara_func.stop(i)
                    print("[SCARA::STOP]")

                elif recv_cmd.cmd == "emg_on":
                    self.scara_func.emg_on(0)
                    print("[SCARA::EMG_ON]")

                elif recv_cmd.cmd == "emg_off":
                    self.scara_func.emg_off(0)
                    print("[SCARA::EMG_OFF]")

                elif recv_cmd.cmd == "reset_alarm":
                    self.scara_func.alarm_reset(0)
                    print("[SCARA::RESET_ALARM]")

                else:
                    print("[SCARA::WARN] INVALID COMMAND")
                # ===============================

if __name__=='__main__':
    try:
        SC = Scara_Client()
        SC.control()


    except rospy.ROSInterruptException:
        pass
