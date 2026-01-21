import onnx
from onnx import version_converter

model_path = "/home/nvidia/rm_robot/src/rm_vision_control/models/crack_model.onnx"
model = onnx.load(model_path)

# 查看导入的 opset
for imp in model.opset_import:
    print(f"Domain: '{imp.domain}', Opset Version: {imp.version}")

# 尝试转换为 opset 18（不会修改原文件，仅测试可行性）
try:
    converted = version_converter.convert_version(model, 18)
    print("✅ 模型可安全降级到 opset 18（说明未使用 opset 19 特有算子）")
except Exception as e:
    print("⚠️ 无法降级到 opset 18，可能使用了新算子:", str(e))