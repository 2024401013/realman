from ultralytics import YOLO

# 第一个模型（替换路径）
model1 = YOLO('/home/nvidia/rm_robot/src/rm_vision_control/models/crack_best.pt')  # 或转换后的 .pt
model1.export(
    format='engine',
    device=0,
    half=True,             # FP16，Orin NX 推荐
    imgsz=640,             # 调整为你的输入大小
    workspace=256,        # 如果 OOM 减到 512
    batch=1,
    dynamic=False
)
print("model1.engine 导出完成")

# 第二个模型
# model2 = YOLO('/home/nvidia/rm_robot/src/rm_vision_control/models/your_model2.pt')
# model2.export(
#     format='engine',
#     device=0,
#     half=True,
#     imgsz=640,
#     workspace=1024,
#     batch=1,
#     dynamic=False
# )
# print("model2.engine 导出完成")