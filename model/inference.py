import argparse
from ultralytics import YOLO
import os

def run_inference(model_path, image_path):
    """
    使用指定的YOLO模型对单张图片进行推理。

    参数:
    model_path (str): 训练好的模型文件路径 (.pt).
    image_path (str): 需要进行推理的图片文件路径.
    """
    # 检查模型文件是否存在
    if not os.path.exists(model_path):
        print(f"错误: 模型文件未找到 '{model_path}'")
        return

    # 检查图片文件是否存在
    if not os.path.exists(image_path):
        print(f"错误: 图片文件未找到 '{image_path}'")
        return

    # 加载模型
    try:
        model = YOLO(model_path)
    except Exception as e:
        print(f"加载模型时出错: {e}")
        return

    # 执行推理
    print(f"正在使用模型 '{model_path}' 对图片 '{image_path}' 进行推理...")
    try:
        # 将 save=True 参数传递给模型调用，Ultralytics会自动处理保存
        # 使用 stream=False 因为我们只处理单张图片，这样代码更简洁
        # 使用 project 参数指定输出目录的根路径
        results = model(image_path, save=True, project='runs/detect', stream=False)
        
        # 结果会自动保存，下面的代码块可以用来额外显示或处理
        # for result in results:
        #     # result.plot() 会在内存中创建图像，可以用于显示
        #     annotated_image = result.plot()
        #     # 如果需要手动保存，可以使用cv2
        #     # import cv2
        #     # cv2.imwrite('my_result.jpg', annotated_image)
            
        print("推理完成。")
        print(f"结果已自动保存至 'runs/detect' 目录下的子文件夹中。")

    except Exception as e:
        print(f"推理过程中发生错误: {e}")


if __name__ == '__main__':
    # 创建一个参数解析器
    parser = argparse.ArgumentParser(description="使用YOLO模型对单张图片进行推理。")
    
    # 添加参数
    parser.add_argument('--model', type=str, required=True, help='训练好的模型文件路径 (.pt)。')
    parser.add_argument('--image', type=str, required=True, help='需要进行推理的图片文件路径。')

    # 解析命令行参数
    args = parser.parse_args()

    # 运行推理函数
    run_inference(args.model, args.image)

    # 示例用法:
    # python inference.py --model /path/to/your/best.pt --image /path/to/your/image.jpg
    # 例如:
    # python inference.py --model train_tuned/elevator_detection_model/weights/best.pt --image dataset/20251018_223639/frame_1760798200384462298.png
