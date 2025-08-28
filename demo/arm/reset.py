# -*- coding: utf-8 -*-  
# import sys  
# sys.path.append('D:\\vs2019ws\PythonCtt\PythonCtt')  
import time        
import jkrc  
import math

#运动模式  
ABS = 0  
INCR= 1 
joint_origin_pos = [-math.pi/2.0, math.pi/2.0, 0, 0, 0, 0]  
# joint_pos=[-math.pi/2.0, math.pi/2.0, math.pi/2.0, 0.0, 0, 0]  
robot = jkrc.RC("192.168.10.90")#回机器人对象  
robot.login()#登录  
robot.power_on() #上电  
robot.enable_robot() 

robot.joint_move([0, 0, 0, 0, 0, 0], ABS, True, 20.0) 

print("reset") 
robot.joint_move(joint_origin_pos, ABS, True, 20.0) 

# robot.joint_move([0, 0, 0, 0, 0, 0], ABS, True, 20.0)

robot.disable_robot() 
robot.power_off()   

time.sleep(1)  
robot.logout()  
