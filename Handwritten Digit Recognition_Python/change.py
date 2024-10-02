import torch

# 加载训练好的 PyTorch 模型
model_path = "model.pt"
model = torch.load(model_path)
model.eval()

# 将模型移动到 GPU 上
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model.to(device)

# 定义模型的输入
dummy_input = torch.randn(1, 1, 28, 28).to(device)

# 导出为 ONNX
onnx_path = "model.onnx"
torch.onnx.export(
    model,                # 要转换的模型
    dummy_input,          # 模型输入示例
    onnx_path,            # 输出文件路径
    export_params=True,   # 保存模型参数
    opset_version=11,     # ONNX 版本
    do_constant_folding=True,  # 是否执行常量折叠优化
    input_names=['input'],     # 输入节点名称
    output_names=['output'],   # 输出节点名称
    dynamic_axes={'input': {0: 'batch_size'}, 'output': {0: 'batch_size'}}  # 动态轴设置
)

print(f"模型已成功导出为 {onnx_path}")
