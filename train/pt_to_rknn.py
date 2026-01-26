import os
import sys
from ultralytics import YOLO
from rknn.api import RKNN

# ================= 配置区域 =================
# 1. 输入模型路径
MODEL_NAME = 'best_v3'            # 你的模型文件名（不带后缀）
PT_MODEL_PATH = f'./{MODEL_NAME}.pt'

# 2. 输出路径
ONNX_MODEL_PATH = f'./{MODEL_NAME}.onnx'
RKNN_MODEL_PATH = f'./{MODEL_NAME}.rknn'

# 3. 目标平台
TARGET_PLATFORM = 'rk3588'

# 4. 输入形状 (必须与训练时一致，通常是 640)
IMG_SIZE = (640, 640)
# ===========================================

def export_onnx():
    """第一步：将 PT 导出为 ONNX"""
    print(f"\n[Step 1] Exporting {PT_MODEL_PATH} to ONNX...")
    
    if not os.path.exists(PT_MODEL_PATH):
        print(f"Error: File {PT_MODEL_PATH} not found!")
        sys.exit(1)

    try:
        # 加载 PyTorch 模型
        model = YOLO(PT_MODEL_PATH)
        
        # 导出 ONNX
        # opset=12: 对 RKNN 最友好的版本
        # simplify=True: 消除冗余算子，关键步骤
        path = model.export(format="onnx", opset=12, simplify=True, imgsz=IMG_SIZE)
        
        print(f"[Step 1] ONNX export success: {path}")
        return path
    except Exception as e:
        print(f"Error during ONNX export: {e}")
        sys.exit(1)

def convert_to_rknn(onnx_path):
    """第二步：将 ONNX 转换为 RKNN (FP16 模式)"""
    print(f"\n[Step 2] Converting {onnx_path} to RKNN (FP16)...")

    # 1. 初始化 RKNN
    rknn = RKNN(verbose=False) # verbose=True 可以看详细日志

    # 2. 配置
    # mean=[0,0,0], std=[255,255,255] 表示将输入 (0~255) 归一化到 (0~1)
    # 这是 YOLOv8 的标准预处理
    rknn.config(mean_values=[[0, 0, 0]], 
                std_values=[[255, 255, 255]], 
                target_platform=TARGET_PLATFORM)

    # 3. 加载 ONNX
    ret = rknn.load_onnx(model=onnx_path)
    if ret != 0:
        print('Load ONNX failed!')
        sys.exit(ret)

    # 4. 构建模型
    # do_quantization=False: 关闭量化，使用 FP16 精度
    # dataset=None: FP16 模式下不需要校准数据集
    ret = rknn.build(do_quantization=False, dataset=None)
    if ret != 0:
        print('Build RKNN failed!')
        sys.exit(ret)

    # 5. 导出 RKNN 文件
    ret = rknn.export_rknn(RKNN_MODEL_PATH)
    if ret != 0:
        print('Export RKNN failed!')
        sys.exit(ret)
    
    # 释放资源
    rknn.release()
    print(f"\n[Success] RKNN model saved to: {RKNN_MODEL_PATH}")

if __name__ == '__main__':
    # 执行第一步
    generated_onnx_path = export_onnx()
    
    # 确保用的是生成的那个 onnx 路径（通常 Ultralytics 会返回路径，或者就是我们定义的路径）
    if generated_onnx_path is None:
        generated_onnx_path = ONNX_MODEL_PATH

    # 执行第二步
    convert_to_rknn(generated_onnx_path)