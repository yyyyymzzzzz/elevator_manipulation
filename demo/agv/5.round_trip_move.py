import socket
import json
import time
import uuid

# --- 机器人配置 ---
ROBOT_IP = '192.168.10.10'
ROBOT_PORT = 31001

class RobotAPI:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = None
        self.buffer = b""

    def connect(self):
        try:
            print(f"正在连接到机器人 {self.ip}:{self.port}...")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(1)
            self.sock.connect((self.ip, self.port))
            print("连接成功!")
            return True
        except socket.error as e:
            print(f"连接失败: {e}")
            self.sock = None
            return False

    def disconnect(self):
        if self.sock:
            self.sock.close()
            self.sock = None
            print("连接已关闭。")

    def _receive_json(self):
        try:
            end_brace_pos = self.buffer.find(b'}{')
            if end_brace_pos != -1:
                json_str = self.buffer[:end_brace_pos + 1]
                self.buffer = self.buffer[end_brace_pos + 1:]
            else:
                json_str = self.buffer
                self.buffer = b""
            return json.loads(json_str.decode('utf-8'))
        except (json.JSONDecodeError, IndexError):
            self.buffer = json_str + self.buffer
            return None

    def send_command(self, command, silent=False):
        if not self.sock:
            print("错误: 未连接到机器人。")
            return None
        request_uuid = str(uuid.uuid4())
        if '?' in command:
            command_with_uuid = f"{command}&uuid={request_uuid}"
        else:
            command_with_uuid = f"{command}?uuid={request_uuid}"
        if not silent:
            print(f"发送指令: {command_with_uuid}")
        try:
            self.sock.sendall(command_with_uuid.encode('utf-8'))
        except socket.error as e:
            print(f"指令发送失败: {e}")
            return None
        start_time = time.time()
        while time.time() - start_time < 10:
            try:
                self.buffer += self.sock.recv(4096)
            except socket.timeout:
                pass
            while True:
                response_json = self._receive_json()
                if response_json:
                    if response_json.get("type") == "notification":
                        print("--- 收到异步通知 ---")
                        print(json.dumps(response_json, indent=2, ensure_ascii=False))
                        continue
                    if response_json.get("uuid") == request_uuid:
                        if not silent:
                            print("收到匹配的响应:")
                            print(json.dumps(response_json, indent=2, ensure_ascii=False))
                        return response_json
                    else:
                        print(f"--- 收到UUID不匹配的响应 (期望: {request_uuid}) ---")
                        print(json.dumps(response_json, indent=2, ensure_ascii=False))
                else:
                    break
        print(f"错误: 等待指令 '{command}' 的响应超时。")
        return None

def define_marker(api, marker_name):
    """引导用户通过物理操作来定义一个目标点（Marker）的完整流程。"""
    print(f"\n--- 准备定义目标点: '{marker_name}' ---")
    print(f"  [1/4] 发送指令让机器人进入可推动模式...")
    api.send_command("/api/estop?flag=true")
    input(f"  [2/4] 机器人电机已解锁，请现在手动将它推到 '{marker_name}' 的期望位置，然后按Enter键继续...")
    print(f"  [3/4] 发送指令锁定机器人位置...")
    api.send_command("/api/estop?flag=false")
    time.sleep(1)
    print(f"  [4/4] 正在当前位置创建标记点 '{marker_name}'...")
    response = api.send_command(f"/api/markers/insert?name={marker_name}")
    if response and response.get("status") == "OK":
        print(f"--- 成功定义目标点: '{marker_name}' ---")
    else:
        print(f"--- 未能定义目标点: '{marker_name}'，请检查机器人状态。 ---")
        raise RuntimeError(f"创建Marker '{marker_name}' 失败")

def adjust_robot_speed(api, new_speed=0.8):
    """检查、设置并验证机器人的移动速度。"""
    print("\n--- 正在调整机器人速度 ---")
    print("\n[1/3] 正在检查当前最大速度...")
    params = api.send_command("/api/get_params")
    if params and "results" in params:
        current_speed = params["results"].get("max_speed_linear", "未知")
        print(f"当前最大直线速度为: {current_speed} m/s")
    else:
        print("未能获取当前速度参数。")
        return

    if current_speed == new_speed:
        print("当前速度已是目标速度，无需调整。")
        return

    print(f"\n[2/3] 正在设置新最大速度为 {new_speed} m/s...")
    new_speed = max(0.1, min(1.0, new_speed))
    set_response = api.send_command(f"/api/set_params?max_speed_linear={new_speed}")
    if not set_response or set_response.get("status") != "OK":
        print("设置速度失败！")
        return
    print("设置指令已发送。")

    print("\n[3/3] 正在验证新设置...")
    time.sleep(1)
    params = api.send_command("/api/get_params")
    if params and "results" in params:
        updated_speed = params["results"].get("max_speed_linear", "未知")
        print(f"验证完成，当前最大直线速度为: {updated_speed} m/s")
        if updated_speed == new_speed:
            print("速度调整成功！")
        else:
            print("速度调整未完全生效，可能受限于机器人内部更高优先级配置。")
    else:
        print("未能获取参数以验证。")

def main():
    """主程序，执行定义两点、调整速度并开始无限往返的完整流程。"""
    api = RobotAPI(ROBOT_IP, ROBOT_PORT)
    try:
        if not api.connect():
            return
            
        marker_a = "point_A"
        marker_b = "point_B"
        
        define_marker(api, marker_a)
        
        # print("\n正在重置与机器人的连接，以确保状态刷新...")
        # api.disconnect()
        # time.sleep(1)
        # if not api.connect():
        #     print("未能重新连接到机器人，程序终止。")
        #     return
        
        define_marker(api, marker_b)

        adjust_robot_speed(api, 0.8)

        print("\n========================================================")
        print("所有目标点已定义完毕，且速度已设定。机器人即将开始往返移动。")
        print("========================================================")
        time.sleep(2)
        
        cruise_command = f"/api/move?markers={marker_a},{marker_b}&count=-1"
        api.send_command(cruise_command)

        print("\n机器人已开始往返移动。")
        input("任务正在后台执行。按Enter键可随时发送“停止”指令来结束任务...")
        
        api.send_command("/api/move/cancel")
        print("已发送停止指令。演示结束。")

    except RuntimeError as e:
        print(f"\n程序因错误中断: {e}")
    except KeyboardInterrupt:
        print("\n用户中断程序，正在关闭连接...")
    finally:
        api.disconnect()

if __name__ == "__main__":
    main()