import socket
import time
import json
import math

# --- 配置 ---
ROBOT_IP = "192.168.10.10"
ROBOT_PORT = 31001
# 当障碍物距离小于此值时，直接采用中心点测量法，认为最近的就是正前方的
NEAR_OBSTACLE_THRESHOLD = 0.6
# 当障碍物较远时，在机器人前方多远处设置探测点
FAR_PROBE_DISTANCE = 0.5 

class RobustDistanceMeter:
    def __init__(self, ip, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(10)
        print(f"正在连接机器人 {ip}:{port}...")
        self.sock.connect((ip, port))
        print("连接成功。")

    def send_and_get_response(self, command):
        """发送指令并等待一个有效的 'response' 类型的回复"""
        self.sock.sendall((command + '\n').encode('utf-8'))
        while True:
            try:
                data = self.sock.recv(4096).decode('utf-8')
                for line in data.strip().split('\n'):
                    if not line: continue
                    msg = json.loads(line)
                    if msg.get("type") == "response":
                        return msg
            except (socket.timeout, json.JSONDecodeError, IndexError):
                print("\n错误：接收或解析数据失败。")
                return None

    def get_current_pose(self):
        """获取机器人当前姿态 (x, y, theta)"""
        status_data = self.send_and_get_response("/api/robot_status")
        if status_data and status_data.get("status") == "OK":
            return status_data.get("results", {}).get("current_pose")
        return None

    def get_distance_at_point(self, x, y):
        """获取指定坐标点的障碍物距离"""
        response = self.send_and_get_response(f"/api/map/distance_probe?x={x}&y={y}")
        if response and response.get("status") == "OK":
            return response.get("results", {}).get("env_dist", {}).get("obstacle", -1)
        return -1

    def run_measurement(self):
        """主测试循环，使用融合策略持续测量距离"""
        print(f"\n测试开始：将使用融合策略测量正前方障碍物距离。")
        print(f"(近距离阈值: {NEAR_OBSTACLE_THRESHOLD}米, 远距离探测点: {FAR_PROBE_DISTANCE}米)")
        print("按 Ctrl+C 结束测试。")
        try:
            while True:
                final_distance = -1
                method_used = "未知"
                
                pose = self.get_current_pose()
                if not pose:
                    print("\r错误：无法获取机器人位置...         ", end="")
                    time.sleep(1)
                    continue

                # --- 融合策略核心逻辑 ---
                # 1. 先从机器人中心探测一次，用于判断远近
                dist_from_center = self.get_distance_at_point(pose['x'], pose['y'])

                if dist_from_center >= 0 and dist_from_center < NEAR_OBSTACLE_THRESHOLD:
                    # 2. 如果障碍物很近，直接采纳中心点的测量值
                    final_distance = dist_from_center
                    method_used = "近距离模式(中心点)"
                elif dist_from_center >= NEAR_OBSTACLE_THRESHOLD:
                    # 3. 如果障碍物较远，使用前方探测点法以获得更好的方向性
                    method_used = "远距离模式(前方点)"
                    probe_x = pose['x'] + FAR_PROBE_DISTANCE * math.cos(pose['theta'])
                    probe_y = pose['y'] + FAR_PROBE_DISTANCE * math.sin(pose['theta'])
                    
                    dist_from_probe = self.get_distance_at_point(probe_x, probe_y)
                    if dist_from_probe >= 0:
                        final_distance = FAR_PROBE_DISTANCE + dist_from_probe
                
                # 打印结果
                if final_distance >= 0:
                    print(f"\r正前方距离: {final_distance:.4f} 米 (模式: {method_used})    ", end="")
                else:
                    print(f"\r正前方距离: 探测失败 (中心点距离: {dist_from_center:.2f}米)...         ", end="")
                
                time.sleep(0.5)

        except KeyboardInterrupt:
            print("\n测试结束。")
        finally:
            self.sock.close()
            print("连接已断开。")

if __name__ == "__main__":
    try:
        meter = RobustDistanceMeter(ROBOT_IP, ROBOT_PORT)
        meter.run_measurement()
    except Exception as e:
        print(f"脚本启动失败: {e}")