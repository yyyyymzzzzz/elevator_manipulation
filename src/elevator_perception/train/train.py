from ultralytics import YOLO

# 1. 选择一个更大尺寸的预训练模型
model = YOLO('yolov8s.pt') # 从 'yolov8n.pt' 更换为 'yolov8s.pt' 或 'yolov8m.pt'

# 2. 启动模型训练，并传入详细的超参数
if __name__ == '__main__':
    results = model.train(
        # --- 核心训练参数 ---
        data='/path/to/your/data.yaml',  # [请务必修改] 你的data.yaml文件路径
        epochs=150,                     # 增加训练轮次
        imgsz=640,
        patience=30,                    # 如果30个epoch内验证集性能没有提升，则提前停止训练
        optimizer='AdamW',              # 可以尝试 'SGD'
        lr0=0.001,                      # 初始学习率 (可以尝试 1e-3, 1e-4 等)
        lrf=0.01,                       # 最终学习率
        weight_decay=0.0005,            # 权重衰减

        # --- 数据增强参数 ---
        degrees=10.0,                   # 随机旋转 (-10, +10) 度
        translate=0.1,                  # 随机平移 10%
        scale=0.2,                      # 随机缩放 (-20%, +20%)
        shear=5.0,                      # 随机剪切 (-5, +5) 度
        flipud=0.5,                     # 50% 概率垂直翻转
        fliplr=0.5,                     # 50% 概率水平翻转
        mosaic=1.0,                     # 始终启用 Mosaic 增强 (在训练初期很有用)
        mixup=0.1,                      # 10% 的概率启用 MixUp 增强
        hsv_h=0.015,                    # 色调增强
        hsv_s=0.7,                      # 饱和度增强
        hsv_v=0.4,                      # 明度增强

        # --- 项目和名称 ---
        project='runs/train_tuned',     # 将调优后的训练结果保存在新文件夹
        name='elevator_detection_s_model'
    )

    # https://universe.roboflow.com/ds/MqKaCbcWL9?key=umWksrCtCC