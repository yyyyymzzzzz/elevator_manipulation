import os
import glob
import base64
import requests
import json
import time 
import cv2
import numpy as np

IMAGE_DIRECTORY = "/home/nvidia/Workspace/elevator_manipulation/data/floor/valid/images" 
OLLAMA_MODEL = "gemma3:4b"  
OLLAMA_API_URL = "http://127.0.0.1:11434/api/generate"
WINDOW_NAME = "Elevator Floor Recognition Test"

# --- 图像预处理 ---
def preprocess_image(image_bgr: np.ndarray) -> np.ndarray:
    """对输入图像进行增强预处理，以提升显示屏数字/字符的可辨识度。

    流程：
    1) 限制最大边长，仅缩小不放大（减少噪声与请求体大小）
    2) 双边滤波去噪，保留边缘
    3) LAB 空间对 L 通道进行 CLAHE 提升对比度
    4) 轻度反锐化（Unsharp Mask）增强细节
    返回 BGR 图像。
    """
    if image_bgr is None or image_bgr.size == 0:
        return image_bgr

    # 1) 尺寸规范：最长边不超过 1280，仅缩小不放大
    h, w = image_bgr.shape[:2]
    max_side = 1280
    scale = min(max_side / max(h, w), 1.0)
    if scale < 1.0:
        new_w, new_h = int(w * scale), int(h * scale)
        image_bgr = cv2.resize(image_bgr, (new_w, new_h), interpolation=cv2.INTER_AREA)

    # 2) 双边滤波去噪（保边）
    denoised = cv2.bilateralFilter(image_bgr, d=9, sigmaColor=75, sigmaSpace=75)

    # 3) CLAHE 提升亮度对比度（在 LAB 的 L 通道上）
    lab = cv2.cvtColor(denoised, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    l2 = clahe.apply(l)
    lab2 = cv2.merge((l2, a, b))
    enhanced = cv2.cvtColor(lab2, cv2.COLOR_LAB2BGR)

    # 4) 反锐化（先轻度高斯模糊，再加权相减）
    blur = cv2.GaussianBlur(enhanced, ksize=(0, 0), sigmaX=1.0)
    sharpened = cv2.addWeighted(enhanced, 1.5, blur, -0.5, 0)

    return sharpened

def get_image_paths(directory):
    """获取目录下所有支持的图片文件路径"""
    supported_formats = ('*.png', '*.jpg', '*.jpeg', '*.bmp', '*.tiff')
    image_paths = []
    for fmt in supported_formats:
        image_paths.extend(glob.glob(os.path.join(directory, fmt)))
    if not image_paths:
        print(f"错误: 在目录 '{directory}' 中没有找到任何图片文件。")
        print("请检查 IMAGE_DIRECTORY 变量是否设置正确。")
    return image_paths

def analyze_image_with_ollama(image_np, prompt):
    """调用Ollama API分析图片并返回识别结果和耗时""" # <<< 修改
    # 将OpenCV的Numpy数组格式的图片编码为JPG，然后转为Base64
    _, buffer = cv2.imencode('.jpg', image_np)
    base64_image = base64.b64encode(buffer).decode('utf-8')

    payload = {
        "model": OLLAMA_MODEL,
        "prompt": prompt,
        "images": [base64_image],
        "stream": False,
        "format": "json"
    }

    try:
        start_time = time.monotonic() # <<< 新增：记录开始时间
        response = requests.post(OLLAMA_API_URL, json=payload, timeout=20)
        end_time = time.monotonic() # <<< 新增：记录结束时间
        
        response.raise_for_status()
        
        duration = end_time - start_time # <<< 新增：计算耗时

        response_data = json.loads(response.json().get('response', '{}'))
        floor = response_data.get("floor", "N/A")
        
        return f"Floor: {floor}", duration # <<< 修改：返回结果和耗时

    except requests.exceptions.RequestException as e:
        print("\n错误: 调用Ollama API失败。请确认Ollama服务正在运行。")
        print(f"详细信息: {e}")
        return "Error: API Call Failed", 0.0 # <<< 修改：返回错误和0耗时
    except json.JSONDecodeError:
        print("\n错误: 解析模型返回的JSON失败。")
        return "Error: Invalid JSON", 0.0 # <<< 修改：返回错误和0耗时


def main():
    """主函数，用于循环显示图片并调用模型分析"""
    print("--- 电梯楼层识别可视化脚本 ---")
    print(f"模型: {OLLAMA_MODEL}")
    print(f"图片目录: {IMAGE_DIRECTORY}")
    print("按 'q' 键退出程序。")
    
    image_paths = get_image_paths(IMAGE_DIRECTORY)
    if not image_paths:
        return

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    prompt = """Analyze the elevator panel in the image. 
    Identify the current floor number shown on the display. 
    Respond with ONLY a JSON object containing a single key 'floor'. 
    For example: {\"floor\": \"10\"}"""

    for image_path in image_paths:
        frame = cv2.imread(image_path)
        if frame is None:
            print(f"警告: 无法读取图片 {image_path}, 跳过。")
            continue

        # --- 预处理：增强图像后再进行识别 ---
        proc_frame = preprocess_image(frame)

        # <<< 修改：接收两个返回值
        prediction_text, duration = analyze_image_with_ollama(proc_frame, prompt)
        
        # <<< 新增：格式化耗时文本
        time_text = f"Time: {duration:.2f} s" 
        
        # <<< 修改：在终端打印也加入耗时信息
        print(f"分析图片: {os.path.basename(image_path)} -> 预测: {prediction_text} ({time_text})")
        
        # --- 在图片上绘制结果 ---
        # <<< 修改：增大背景矩形的高度以容纳两行文字
        overlay = proc_frame.copy()
        cv2.rectangle(overlay, (0, 0), (proc_frame.shape[1], 100), (0, 0, 0), -1)
        alpha = 0.6
        proc_frame = cv2.addWeighted(overlay, alpha, proc_frame, 1 - alpha, 0)

        # 绘制楼层预测结果
        cv2.putText(
            proc_frame,
            prediction_text,
            (20, 40),                   
            cv2.FONT_HERSHEY_SIMPLEX,   
            1.5,                        
            (255, 255, 255),            
            3                           
        )

        # <<< 新增：绘制耗时信息
        cv2.putText(
            proc_frame,
            time_text,
            (20, 85),                   # Y坐标下移，放在楼层信息的下方
            cv2.FONT_HERSHEY_SIMPLEX,   
            1.2,                        # 使用稍小一点的字体
            (50, 205, 50),              # 字体颜色 (酸橙绿)
            2                           
        )

        cv2.imshow(WINDOW_NAME, proc_frame)

        if cv2.waitKey(1000) & 0xFF == ord('q'):
            print("用户请求退出...")
            break
    
    cv2.destroyAllWindows()
    print("程序已退出。")


if __name__ == "__main__":
    main()