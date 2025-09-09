import socket
import json
import time
import uuid
import sys # 导入sys模块以读取命令行参数

# --- 机器人配置 ---
ROBOT_IP = '192.168.10.10'
ROBOT_PORT = 31001

# ... 此处省略与之前版本相同的 RobotAPI 类代码 ...
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

def main():
    """
    本脚本专用于定义一个目标点。
    用法: python3 define_marker.py <marker_name>
    """
    if len(sys.argv) < 2:
        print("错误: 请提供一个目标点名称作为参数。")
        print("用法: python3 define_marker.py point_A")
        return

    marker_name = sys.argv[1]
    api = RobotAPI(ROBOT_IP, ROBOT_PORT)

    try:
        if not api.connect():
            return

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
            print(f"\n--- 成功定义目标点: '{marker_name}' ---")
        else:
            print(f"\n--- 未能定义目标点: '{marker_name}'，请检查机器人状态。 ---")
            
    finally:
        api.disconnect()

if __name__ == "__main__":
    main()