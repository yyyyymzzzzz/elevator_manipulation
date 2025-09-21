import socket
import time
import json
import math

# --- 用户配置区 ---
CONFIG = {
    "robot_ip": "192.168.10.10",
    "robot_port": 31001,
    "start_marker": "start",
    "wait_marker": "wait",
    "back_marker": "back",
    "move_speed": 0.2,
    "turn_speed": 0.5,
    "probe_distance": 1.5,
    "distance_threshold": 0.5,
    "move_timeout": 60,
    # 以下为融合测距与10Hz控制新增参数
    "near_obstacle_threshold": 0.6,   # 近距离阈值（米）：小于此值时直接采用中心点测距
    "far_probe_distance": 0.5,        # 远距离探测点距离（米）：沿着机器人朝向的前方
    "send_rate_hz": 10                # 手动控制命令发送频率（Hz）
}
# --------------------

class RobotDemoController:
    # ... (除了 wait_for_distance_increase 之外的其他函数与之前版本相同) ...
    def __init__(self, ip, port):
        self.robot_ip = ip
        self.robot_port = port
        self.sock = None
        self.connect()

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(10)
            self.sock.connect((self.robot_ip, self.robot_port))
            print(f"成功连接到机器人 {self.robot_ip}:{self.robot_port}")
        except socket.error as e:
            print(f"连接机器人失败: {e}")
            raise

    def send_command(self, command, expect_response=True):
        """
        发送命令，并仅返回与本次请求路径匹配的 response。
        为避免历史/异步响应（如 joy_control）干扰，这里会忽略 command 不匹配的 response。
        匹配规则：比较不带 query 的路径前缀（例如 '/api/map/distance_probe'）。
        """
        if not command.endswith('\n'):
            command += '\n'

        print(f"  [发送]: {command.strip()}")
        self.sock.sendall(command.encode('utf-8'))

        if not expect_response:
            return None

        expected_base = command.strip().split('?', 1)[0]

        while True:
            try:
                response_data = self.sock.recv(4096).decode('utf-8')
                for line in response_data.strip().split('\n'):
                    if not line:
                        continue
                    response_json = json.loads(line)
                    msg_type = response_json.get("type")
                    if msg_type == "response":
                        resp_cmd = (response_json.get("command") or "").split('?', 1)[0]
                        if resp_cmd == expected_base:
                            print(f"  [接收 Response]: {response_json}")
                            return response_json
                        else:
                            print(f"  [忽略 Response(非本命令)]: 期望={expected_base}, 实际={resp_cmd}")
                            # 继续等待匹配本命令的响应
                    elif msg_type in ["notification", "callback"]:
                        print(f"  [忽略 {msg_type.capitalize()}]: {response_json}")
                    else:
                        print(f"  [接收 未知类型]: {response_json}")
            except socket.timeout:
                print("  [错误] 等待匹配回复超时。")
                return None
            except (json.JSONDecodeError, IndexError) as e:
                print(f"  [警告] 解析返回时出错: {e}, 返回数据: '{response_data}'")
                continue

    def get_robot_status(self):
        return self.send_command("/api/robot_status")

    def wait_for_move_completion(self, task_id, timeout):
        print(f"  [状态] 等待任务ID: {task_id} 完成...")
        start_time = time.time()
        while time.time() - start_time < timeout:
            status_data = self.get_robot_status()
            if status_data and "results" in status_data:
                if status_data["results"].get("task_id") == task_id:
                    move_status = status_data["results"].get("move_status")
                    if move_status == "succeeded":
                        print("  [状态] 任务成功。")
                        return True
                    if move_status in ["failed", "canceled"]:
                        print(f"  [状态] 任务失败或被取消 ({move_status})。")
                        return False
            time.sleep(1)
        print("  [状态] 任务超时。")
        return False

    def get_distance_probe(self, x, y):
        command = f"/api/map/distance_probe?x={x}&y={y}"
        response = self.send_command(command)
        if response and response.get("status") == "OK" and "results" in response:
            return response["results"]["env_dist"].get("obstacle", -1)
        return -1

    # ==================== 融合测距工具函数 ====================
    def get_current_pose(self):
        """从 robot_status 中提取当前位姿 (x, y, theta)。失败返回 None。"""
        status = self.get_robot_status()
        if status and status.get("status") == "OK":
            return status.get("results", {}).get("current_pose")
        return None

    def fused_forward_distance(self):
        """
        使用融合策略估计“正前方”障碍物距离。
        策略：
          - 先在机器人中心 (x,y) 测一次距离 dist_center。
          - 若 dist_center 在 [0, near_threshold) 内，则直接返回 dist_center（近距离模式）。
          - 若 dist_center >= near_threshold，则在前方 far_probe_distance 处测距 dist_probe，
            若有效则返回 far_probe_distance + dist_probe（远距离模式）。

        返回: (final_distance, method_used, dist_center, dist_probe)
               其中 dist_probe 若未测则为 -1。
        失败: ( -1, "测距失败", dist_center, dist_probe )
        """
        pose = self.get_current_pose()
        if not pose:
            return -1, "位姿获取失败", -1, -1

        near_th = CONFIG.get("near_obstacle_threshold", 0.6)
        far_d = CONFIG.get("far_probe_distance", CONFIG.get("probe_distance", 0.5))

        dist_center = self.get_distance_probe(pose['x'], pose['y'])

        if dist_center >= 0 and dist_center < near_th:
            return dist_center, "近距离模式(中心点)", dist_center, -1

        if dist_center >= near_th:
            probe_x = pose['x'] + far_d * math.cos(pose['theta'])
            probe_y = pose['y'] + far_d * math.sin(pose['theta'])
            dist_probe = self.get_distance_probe(probe_x, probe_y)
            if dist_probe >= 0:
                return far_d + dist_probe, "远距离模式(前方点)", dist_center, dist_probe
            return -1, "前方点测距失败", dist_center, dist_probe

        # dist_center < 0 的情况
        return -1, "中心点测距失败", dist_center, -1

    def wait_for_distance_increase(self, probe_dist, threshold):
        """使用融合测距策略，等待正前方距离变大到超出阈值，并实时打印。"""
        print("  [动作] 开始融合测距，等待距离增加...")

        # 计算初始距离（融合策略）
        init_dist, init_mode, init_center, init_probe = self.fused_forward_distance()
        if init_dist < 0:
            print(f"  [错误] 无法获取初始距离（{init_mode}）。")
            return False

        print(f"  [信息] 初始距离: {init_dist:.2f} m (模式: {init_mode})，触发阈值: +{threshold:.2f} m")

        while True:
            cur_dist, mode, d_center, d_probe = self.fused_forward_distance()
            if cur_dist >= 0:
                print(f"\r  [调试] 当前距离: {cur_dist:.3f} m (模式: {mode})    ", end="")
            else:
                print(f"\r  [调试] 当前距离: 探测失败 (原因: {mode})    ", end="")

            if cur_dist >= 0 and cur_dist > init_dist + threshold:
                print(f"\n  [成功] 距离由 {init_dist:.2f} m 增至 {cur_dist:.2f} m，已达阈值。")
                return True

            time.sleep(0.5)

    # ==================== 手动控制：10Hz持续发送 ====================
    def send_control_command(self, linear_v: float = 0.0, angular_v: float = 0.0):
        command = f"/api/joy_control?linear_velocity={linear_v}&angular_velocity={angular_v}"
        self.send_command(command, expect_response=False)

    def execute_timed_move(self, duration: float, linear_v: float = 0.0, angular_v: float = 0.0):
        rate_hz = max(1, int(CONFIG.get("send_rate_hz", 10)))
        period = 1.0 / rate_hz
        print(f"  [动作] 持续 {duration:.2f}s, 线速度={linear_v}, 角速度={angular_v}, 频率={rate_hz}Hz")
        end_time = time.time() + duration
        while time.time() < end_time:
            self.send_control_command(linear_v, angular_v)
            time.sleep(period)
        # 停止
        self.send_control_command(0.0, 0.0)
        print("  [状态] 动作完成，已发送停止指令。")
        time.sleep(1)

    def move_blind(self, distance, speed):
        """以固定速度前进/后退指定距离，期间以固定频率刷新命令。"""
        if speed == 0:
            print("  [警告] 速度为0，跳过 move_blind。")
            return
        duration = abs(distance / speed)
        direction = 1.0 if distance > 0 else -1.0
        self.execute_timed_move(duration, linear_v=speed * direction, angular_v=0.0)
        print(f"  [动作] 盲走完成: {distance} 米。")

    def turn_around_blind(self, turn_speed):
        """原地掉头180度，期间以固定频率刷新命令。"""
        if turn_speed == 0:
            print("  [警告] 角速度为0，跳过 turn_around_blind。")
            return
        angle = math.pi
        duration = abs(angle / turn_speed)
        self.execute_timed_move(duration, linear_v=0.0, angular_v=turn_speed)
        print("  [动作] 原地掉头完成。")
        
    def move_and_wait(self, marker_name):
        move_response = self.send_command(f"/api/move?marker={marker_name}")
        if move_response and move_response.get("status") == "OK":
            task_id = move_response.get("results", {}).get("task_id")
            if task_id:
                return self.wait_for_move_completion(task_id, CONFIG['move_timeout'])
        print(f"  [错误] 移动到 {marker_name} 任务启动失败。")
        return False

    def run_demo(self):
        try:
            if not self.move_and_wait(CONFIG['wait_marker']): return
            if not self.wait_for_distance_increase(CONFIG['probe_distance'], CONFIG['distance_threshold']): return
            self.move_blind(1.0, CONFIG['move_speed'])
            self.turn_around_blind(CONFIG['turn_speed'])
            if not self.wait_for_distance_increase(CONFIG['probe_distance'], CONFIG['distance_threshold']): return
            self.move_blind(1.0, CONFIG['move_speed'])
            if not self.move_and_wait(CONFIG['back_marker']): return
            if not self.move_and_wait(CONFIG['start_marker']): return
            print("\n✅ Demo成功完成！")
        except Exception as e:
            print(f"Demo执行过程中出现异常: {e}")
        finally:
            self.close()

    def close(self):
        if self.sock:
            self.sock.close()
            print("已断开与机器人的连接。")

if __name__ == "__main__":
    try:
        controller = RobotDemoController(CONFIG['robot_ip'], CONFIG['robot_port'])
        controller.run_demo()
    except Exception as e:
        print(f"脚本启动失败: {e}")