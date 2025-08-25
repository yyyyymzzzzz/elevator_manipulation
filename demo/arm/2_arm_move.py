# -*- coding: utf-8 -*-  
import sys  
# sys.path.append('D:\\vs2019ws\PythonCtt\PythonCtt')  
import time        
import jkrc  
	  
#坐标系  
COORD_BASE  = 0  
COORD_JOINT = 1  
COORD_TOOL  = 2  
#运动模式  
ABS = 0  
INCR= 1  
#关节1~6依次对应0~5,  
	  
robot = jkrc.RC("192.168.10.90")#返回机器人对象  
robot.login()#登录  
robot.power_on() #上电  
robot.enable_robot()  
print("move1")  
robot.jog(0, ABS, COORD_JOINT, 0.1, -3.14) 
time.sleep(5)#jog为非阻塞指令，运动状态下接收jog指令会被丢弃  
# print("move2")  
# robot.jog(0,INCR,COORD_JOINT,1.0,-2.0)  
time.sleep(3)  
robot.jog_stop(0)  
robot.logout()
