import socket
import time
import json
import math

# --- 配置 ---
ROBOT_IP = "192.168.10.10"
ROBOT_PORT = 31001
MOVE_SPEED = 0.2  # (米/秒) 前进时的速度
TURN_SPEED = 0.5  # (弧度/秒) 旋转时的角速度

class ManualController:
    def __init__(self, ip, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(10)
        print(f"正在连接机器人 {ip}:{port}...")
        self.sock.connect((ip, port))
        print("连接成功。")

    def send_control_command(self, linear_v=0.0, angular_v=0.0):
        """发送单次joy_control指令"""
        command = f"/api/joy_control?linear_velocity={linear_v}&angular_velocity={angular_v}\n"
        self.sock.sendall(command.encode('utf-8'))

    def execute_timed_move(self, duration, linear_v=0.0, angular_v=0.0):
        """在指定时间内，以10Hz的频率持续发送移动指令"""
        print(f"  执行动作: 持续 {duration:.2f} 秒, 线速度={linear_v}, 角速度={angular_v}")
        end_time = time.time() + duration
        while time.time() < end_time:
            self.send_control_command(linear_v, angular_v)
            time.sleep(0.1) # 以10Hz频率发送
        
        # 动作结束后，必须发送停止指令
        self.send_control_command(0.0, 0.0)
        print("  动作完成，已发送停止指令。")
        time.sleep(1) # 等待机器人稳定

    def move_forward(self, distance_m):
        """前进指定距离"""
        duration = abs(distance_m / MOVE_SPEED)
        self.execute_timed_move(duration, linear_v=MOVE_SPEED)

    def turn_around(self):
        """原地掉头180度"""
        angle_rad = math.pi
        duration = abs(angle_rad / TURN_SPEED)
        self.execute_timed_move(duration, angular_v=TURN_SPEED)

    def interactive_test(self):
        """交互式测试主循环"""
        try:
            while True:
                print("\n请选择要执行的动作:")
                print("  1: 前进 1 米")
                print("  2: 原地掉头 (180度)")
                print("  q: 退出")
                choice = input("请输入选项: ").strip()

                if choice == '1':
                    self.move_forward(1.0)
                elif choice == '2':
                    self.turn_around()
                elif choice.lower() == 'q':
                    print("退出测试。")
                    break
                else:
                    print("无效选项，请重新输入。")
        except KeyboardInterrupt:
            print("\n测试结束。")
        finally:
            self.send_control_command(0.0, 0.0) # 确保退出时机器人停止
            self.sock.close()
            print("连接已断开。")

if __name__ == "__main__":
    try:
        controller = ManualController(ROBOT_IP, ROBOT_PORT)
        controller.interactive_test()
    except Exception as e:
        print(f"脚本启动失败: {e}")