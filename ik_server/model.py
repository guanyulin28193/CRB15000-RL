import onnx
import os

# 获取当前脚本的目录
script_dir = os.path.dirname(os.path.abspath(__file__))
file_path = os.path.join(script_dir, "Hold.onnx")

# 加载ONNX模型
model = onnx.load(file_path)

# 查找并修改 obs_0 的输入维度
for input in model.graph.input:
    if input.name == "obs_0":
        # 修改输入维度为71
        input.type.tensor_type.shape.dim[1].dim_value = 71

# 检查修改后的输入形状
for input in model.graph.input:
    print(f'{input.name}: {input.type.tensor_type}')

# 保存修改后的模型
modified_file_path = os.path.join(script_dir, "modified_model.onnx")
onnx.save(model, modified_file_path)
