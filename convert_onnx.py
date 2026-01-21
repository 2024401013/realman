import onnx
from onnx import version_converter
import os

def convert_onnx_opset(model_input_path, model_output_path, target_opset=20, target_ir_version=9):
    """
    将模型降级到 Opset 20 + IR 9（完全兼容所有 ONNX Runtime）
    """
    if not os.path.exists(model_input_path):
        print(f"❌ 原始模型不存在 → {model_input_path}")
        return False

    try:
        original_model = onnx.load(model_input_path)
        original_opset = original_model.opset_import[0].version
        original_ir = original_model.ir_version
        print(f"📌 原始模型 - Opset：{original_opset}，IR：{original_ir}")

        # 转换 Opset 到 20
        converted_model = version_converter.convert_version(original_model, target_opset)
        # 强制设置 IR 9
        converted_model.ir_version = target_ir_version
        
        # 验证并保存
        onnx.checker.check_model(converted_model)
        onnx.save(converted_model, model_output_path)
        
        # 验证最终版本
        final_model = onnx.load(model_output_path)
        print(f"✅ 转换完成 - Opset：{final_model.opset_import[0].version}，IR：{final_model.ir_version}")
        print(f"📤 保存路径：{output_model_path}")
        return True

    except Exception as e:
        print(f"❌ 转换失败：{str(e)}")
        return False

# 目标模型路径
input_model_path = "/home/nvidia/rm_robot/src/rm_vision_control/models/crack_model_ori.onnx"
output_model_path = "/home/nvidia/rm_robot/src/rm_vision_control/models/crack_model.onnx"

# 执行转换（Opset 20 + IR 9）
convert_onnx_opset(input_model_path, output_model_path, target_opset=19, target_ir_version=8)
