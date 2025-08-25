import jkrc
import time

robot = jkrc.RC("192.168.10.90")   #返回一个机器人对象
robot.login()     #登录

robot.power_on() #上电
print("power on success")

robot.enable_robot() #使能
print("enable robot success")

for i in range (0, 5):
    print(f"waiting for {i} seconds...")
    time.sleep(1)
print("waiting for 5 seconds...")

robot.disable_robot() #禁用
print("disable robot success")

robot.power_off() #下电
print("power off success")

robot.logout()    #登出
