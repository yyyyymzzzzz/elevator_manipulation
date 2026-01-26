from ultralytics import YOLO

model = YOLO('yolov8s.pt') 

if __name__ == '__main__':
    results = model.train(
        data='/home/ymz/Workspace/data/xiazhi_dataset_v3/elevator.yaml',  # data.yaml文件路径
        epochs=300,                   
        imgsz=640,
        patience=30,                    
        optimizer='AdamW',              # 或者 'SGD'
        lr0=0.001,                      # 初始学习率 (可以尝试 1e-3, 1e-4 等)
        lrf=0.01,                       # 最终学习率
        weight_decay=0.0005,            # 权重衰减

        degrees=5.0,                    # 减少旋转角度，避免小目标变形
        translate=0.2,                  # 增加平移幅度，让目标出现在画面各个位置 (从0.05改为0.2)
        scale=0.3,                      # 增加缩放幅度，让目标变得更小 (从0.1改为0.5，允许目标缩小到50%)
        shear=2.0,                      # 减少剪切变形
        flipud=0.0,                     # 0% 概率垂直翻转
        fliplr=0.0,                     # 0% 概率水平翻转
        mosaic=1.0,                     # 始终启用 Mosaic 增强 (将4张图拼接，目标自然变小)
        mixup=0.0,                      # 禁用 MixUp，避免小目标被混合模糊
        copy_paste=0.5,                 # 增加复制粘贴概率，创造更多小目标样本 (从0.3改为0.5)
        hsv_h=0.1,                      # 适度色调增强
        hsv_s=0.6,                      # 适度饱和度增强
        hsv_v=0.3,                      # 适度明度增强
        bgr=0.5,                        # 50%的概率进行BGR转换（颜色通道互换）

        # --- 项目和名称 ---
        project='train_tuned_v3',     # 将调优后的训练结果保存在新文件夹
        name='vision_test_model'
    )

    # https://universe.roboflow.com/ds/MqKaCbcWL9?key=umWksrCtCC
