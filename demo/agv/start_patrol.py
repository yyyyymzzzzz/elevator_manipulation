import socket
import json
import time
import uuid

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
                        if not silent:
                            print("--- 收到异步通知 ---")
                            print(json.dumps(response_json, indent=2, ensure_ascii=False))
                        continue
                    if response_json.get("uuid") == request_uuid:
                        if not silent:
                            print("收到匹配的响应:")
                            print(json.dumps(response_json, indent=2, ensure_ascii=False))
                        return response_json
                    else:
                        if not silent:
                            print(f"--- 收到UUID不匹配的响应 (期望: {request_uuid}) ---")
                            print(json.dumps(response_json, indent=2, ensure_ascii=False))
                else:
                    break
        if not silent:
            print(f"错误: 等待指令 '{command}' 的响应超时。")
        return None


def main():
    """
    本脚本通过手动循环发送单点移动指令来控制机器人往返。
    """
    api = RobotAPI(ROBOT_IP, ROBOT_PORT)
    targets = ["A", "B", "C"]
    target_index = 0
    
    try:
        if not api.connect():
            return

        # 主循环，用于无限往返
        while True:
            current_target = targets[target_index]
            print(f"\n========================================================")
            print(f"指令: 前往目标点 -> {current_target}")
            print(f"========================================================")

            # 1. 发送单点移动指令
            api.send_command(f"/api/move?marker={current_target}")

            # 2. 监控循环：持续查询状态，直到任务完成
            while True:
                time.sleep(1) # 每秒查询一次状态
                
                # 以静默模式发送状态查询，避免打印过多信息
                response = api.send_command("/api/robot_status", silent=True)
                
                if not response or "results" not in response:
                    print("  > 无法获取机器人状态，可能已断开连接。")
                    break

                move_status = response["results"].get("move_status")
                print(f"  > 正在前往 {current_target}... 当前状态: {move_status}")

                # 检查任务是否成功
                if move_status == "succeeded":
                    print(f"--- 已成功到达 {current_target}! ---")
                    break # 跳出监控循环，进行下一个目标

                # 检查任务是否失败
                if move_status in ["failed", "canceled"]:
                    print(f"--- 前往 {current_target} 的任务失败或被取消! ---")
                    # 选择是重试还是终止整个程序
                    raise RuntimeError(f"任务失败，状态: {move_status}")

            # 3. 切换到下一个目标点
            target_index = (target_index + 1) % len(targets)
            
    except (KeyboardInterrupt, RuntimeError) as e:
        print(f"\n程序中断: {e}")
        print("正在发送取消指令以防万一...")
        api.send_command("/api/move/cancel") # 确保机器人停止
    finally:
        api.disconnect()

if __name__ == "__main__":
    main()