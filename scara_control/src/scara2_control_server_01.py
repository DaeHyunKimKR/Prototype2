# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import String, Bool
from scara_control.msg import Scara_cmd, Scara_status


# main control
class Scara_Server():
    def __init__(self):
        rospy.init_node("Scara_Server")
        self.scara_msg_pub = rospy.Publisher('/scara_control', Scara_cmd, queue_size=100)
        rospy.loginfo(":: Scara Server Started ::")

    def pub_scara_cmd(self, node=[], cmd=None, dist1=0.0, ang1=0.0, ang2=0.0, ang3=0.0, t=0.0, t_list = [], count=0):
        
        scara_msg = Scara_cmd()
        scara_msg.node      = node
        scara_msg.cmd       = cmd
        scara_msg.dist1     = dist1
        scara_msg.angle1    = ang1
        scara_msg.angle2    = ang2
        scara_msg.angle3    = ang3
        scara_msg.time      = t
        scara_msg.time_list = t_list
        scara_msg.count     = count
        
        self.scara_msg_pub.publish(scara_msg)

if __name__=='__main__':
    try:
        SS = Scara_Server()
        node=[]
        cmd=None
        dist1=0.0
        ang1=0.0 
        ang2=0.0
        ang3=0.0
        t=0.0
        t_list = []
        count=0
        while True:
            cmd = raw_input("> cmd: ")
            if cmd == "joint":
                try:
                    node = input(">> node: ")
                    dist1 = input(">> distance1: ")
                    ang1 = input(">> angle1: ")
                    ang2 = input(">> angle2: ")
                    ang3 = input(">> angle3: ")
                    SS.pub_scara_cmd(node, cmd, dist1, ang1, ang2, ang3)
                    print("[SCARA::CMD] Joint")
                except:
                    print("[SCARA::WARN] INVALID INPUT error1")
                    
            elif cmd == "velocity":
                try:
                    node = input(">> node: ")
                    dist1 = input(">> velocity1: ")
                    ang1 = input(">> velocity2: ")
                    ang2 = input(">> velocity3: ")
                    ang3 = input(">> velocity4: ")
                    SS.pub_scara_cmd(node, cmd, dist1, ang1, ang2, ang3)
                    print("[SCARA::CMD] PUB SCARA CMD type2")
                except:
                    print("[SCARA::WARN] INVALID INPUT error1")
                    
            elif cmd == "joint_time":
                try:
                    node = input(">> node: ")
                    dist1 = input(">> distance1: ")
                    ang1 = input(">> angle1: ")
                    ang2 = input(">> angle2: ")
                    ang3 = input(">> angle3: ")
                    t_temp = input(">> time or time_list: ")
                    if (type(t_temp) is int) or (type(t_temp) is float):
                        SS.pub_scara_cmd(node, cmd, dist1, ang1, ang2, ang3, t_temp, [], 0)
                        print("[SCARA::CMD] PUB SCARA CMD type1")
                    elif type(t_temp) is list:
                        print('CP 2')
                        SS.pub_scara_cmd(node, cmd, dist1, ang1, ang2, ang3, 0, t_temp, 0)
                        print("[SCARA::CMD] PUB SCARA CMD type2")
                except:
                    print("[SCARA::WARN] INVALID INPUT error1")


            elif (cmd == "on") or (cmd == "off") or (cmd == "reset_alarm") or (cmd == 'stop') or (cmd == 'stop_emg')  or (cmd == 'status'):
                try:
                    node = input(">> node: ")
                    SS.pub_scara_cmd(node, cmd)
                    print("[SCARA::CMD] PUB SCARA CMD type5")
                except:
                    print("[SCARA::WARN] INVALID INPUT error3")
                    
            elif (cmd[:4] == "TRAY"):
                try:
                    node = input(">> node: ")
                    SS.pub_scara_cmd(node, cmd)
                    print("[SCARA::CMD] PUB SCARA CMD KISTTray")
                except:
                    print("[SCARA::WARN] INVALID INPUT error3")        



            elif (cmd == 'Q') or (cmd == 'q'):
                print("[SCARA] QUIT")
                break
                    
            else:
                print("[SCARA::INVALID_CMD]")

    except rospy.ROSInterruptException:
        pass
