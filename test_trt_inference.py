import numpy as np
np.bool = bool          # 手动 alias，欺骗 tensorrt 的旧代码
np.bool_ = np.bool_

from ultralytics import YOLO
import torch

# 检查 GPU 可用
print(f"CUDA 可用: {torch.cuda.is_available()}")  # 应为 True

# 加载第一个 TensorRT 模型
# try:
#     trt_model1 = YOLO('/home/nvidia/rm_robot/src/rm_vision_control/models/crack_model.engine')
#     print(f"✅ model1.engine 加载成功，设备: {trt_model1.device}")
# except Exception as e:
#     print(f"❌ 加载失败: {str(e)}")

# 加载第二个模型
try:
    trt_model2 = YOLO('/home/nvidia/rm_robot/src/rm_vision_control/models/target_model.engine')
    print(f"✅ model2.engine 加载成功，设备: {trt_model2.device}")
except Exception as e:
    print(f"❌ 加载失败: {str(e)}")

# 测试预测 (用测试图像；替换为你的图像路径或 URL)
test_image = '/home/nvidia/rm_robot/src/rm_vision_control/test.jpg'  

# model1 预测
# results1 = trt_model1(test_image)
# print(f"✅ model1 预测成功: 检测到 {len(results1[0].boxes)} 个目标")
# results1[0].show()  # 显示结果 (需 GUI 或保存 results1[0].save('result1.jpg'))

# model2 预测
results2 = trt_model2(test_image)
print(f"✅ model2 预测成功: 检测到 {len(results2[0].boxes)} 个目标")
results2[0].show()  # 显示结果s