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

    def scara_cmd_publish(self, node=[], cmd=None, x=0.0, y=0.0, ang1=0.0, ang2=0.0, ang3=0.0, t=0.0, count=0):
        scara_msg = Scara_cmd()
        scara_msg.node = node
        scara_msg.cmd = cmd
        scara_msg.x = x
        scara_msg.y = y
        scara_msg.angle1 = ang1
        scara_msg.angle2 = ang2
        scara_msg.angle3 = ang3
        scara_msg.time = t
        scara_msg.count = count
        self.scara_msg_pub.publish(scara_msg)

if __name__=='__main__':
    try:
        SS = Scara_Server()
        while True:
            cmd = raw_input("cmd: ")
            if cmd == "joint":
                #node = [1, 2, 3]
                node = input("node: ")
                ang1 = input("angle1: ")
                ang2 = input("angle2: ")
                ang3 = input("angle3: ")
                t = input("time: ")
                SS.scara_cmd_publish(node, cmd, ang1=ang1, ang2=ang2, ang3=ang3, t=t)

            elif cmd == "position":
                #node = [1, 2, 3]
                node = input("node: ")
                x = input("position_x: ")
                y = input("position_y: ")
                t = input("time: ")
                SS.scara_cmd_publish(node, cmd, x, y, t=t)
                
            elif cmd == "position_define":
                node = input("node: ")
                count = input("count: ")
                SS.scara_cmd_publish(node, cmd, count=count)
            elif cmd == "position_motion":
                i = 1
                node = input("node: ")
                while True:
                    print("====="+"motion"+str(i)+"=====")
                    x = input("x: ")
                    y = input("y: ")
                    t = input("time: ")
                    flag = raw_input("next(n), quit(q): ")
                    i += 1
                    if flag == 'q':
                        count = input("count: ")
                        print("=================")
                        SS.scara_cmd_publish(node, cmd, x=x, y=y, t=t, count=count)
                        break
                    SS.scara_cmd_publish(node, cmd, x=x, y=y, t=t)        
                
            elif cmd == "joint_motion":
                i = 1
                node = input("node: ")
                while True:
                    print("====="+"motion"+str(i)+"=====")
                    ang1 = input("angle1: ")
                    ang2 = input("angle2: ")
                    ang3 = input("angle3: ")
                    t = input("time: ")
                    flag = raw_input("next(n), quit(q): ")
                    i += 1
                    if flag == 'q':
                        count = input("count: ")
                        print("=================")
                        SS.scara_cmd_publish(node, cmd, ang1=ang1, ang2=ang2, ang3=ang3, t=t, count=count)
                        break
                    SS.scara_cmd_publish(node, cmd, ang1=ang1, ang2=ang2, ang3=ang3, t=t)             
                    
            else:
                node = input("node: ")
                ang1 = 0.0
                ang2 = 0.0
                ang3 = 0.0
                t = 0.0
                SS.scara_cmd_publish(node, cmd, ang1=ang1, ang2=ang2, ang3=ang3, t=t)

    except rospy.ROSInterruptException:
        pass
