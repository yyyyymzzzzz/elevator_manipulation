from ultralytics import YOLO
from PIL import Image
import os

# 加载训练好的模型
model = YOLO('/home/ymz/Workspace/elevator_manipulation/src/elevator_perception/train/runs3/train_tuned/elevator_detection_s_model/weights/best.pt')

# 预处理函数：调整图片大小为640x640
def preprocess_image(image_path):
    img = Image.open(image_path)
    img = img.resize((640, 640))
    return img

# 获取图片路径列表
image_dir = '/home/ymz/Workspace/elevator_manipulation/src/elevator_perception/train/image'
image_paths = [os.path.join(image_dir, fname) for fname in os.listdir(image_dir) if fname.lower().endswith(('.jpg', '.png', '.jpeg'))]

# 预处理所有图片
preprocessed_images = [preprocess_image(p) for p in image_paths]

# 对预处理后的图片进行推理
results = model(preprocessed_images)

# 显示并保存结果
for i, result in enumerate(results):
    # result.show()
    result.save(f'/home/ymz/Workspace/elevator_manipulation/src/elevator_perception/train/result/inference_result_{i}.jpg')
    print(f'Processed image {i+1}/{len(results)}')
    # break  # 只处理第一张图片
