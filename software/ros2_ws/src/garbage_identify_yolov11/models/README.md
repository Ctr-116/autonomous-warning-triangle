# 原始训练模型与本地验证样本

`best1.onnx` 与 `best1.pt` 来自用户提供的 `ultralytics.zip`，并非重新训练或重新导出的模型。
原始归档路径、解压后的字节数和 SHA256 见 `SOURCE_MODELS.json`。
`garbage_data.yaml` 保留原数据集说明，训练路径属于原作者的 Jetson 环境；运行推理不使用这些训练路径。
数据集说明文件标注 CC BY 4.0，含原始 Roboflow 来源链接。模型和代码的来源应分别保留，不将整个归档重新指定为同一许可证。

`samples/` 只包含原归档验证集的六张图片。来源与校验值在 `samples/SOURCE_SAMPLES.json`。
这些图片用于验证实际模型推理、坐标解码与 ROS 2 标签链路；此验证不等同于实车识别准确率评测。

实际模型的元数据包含 16 类，原生输出形状为 `[1,20,8400]`：四个框坐标加十六个类别置信度。
节点优先读取模型元数据中的标签，包括 `Toilet_paper`；训练配置中存在 `Toliet_paper` 的拼写，分类器同时接受该兼容别名。

Ubuntu 22.04 随附 OpenCV 4.5.4 无法解析此 YOLO11 ONNX 的 Split 算子。
`.onnx` 路径使用独立的 ONNX Runtime CPU 后端，无需 PyTorch 或 ultralytics。
`.pt` 路径保留为可选 ultralytics 后端，需要单独准备相应推理环境。
已有 TensorRT `.engine` 没有复制，它依赖生成时的硬件和 TensorRT 环境。

## 可选 ONNX 推理环境

在已安装 Humble、完成本工作空间构建的 Ubuntu 22.04 上：

```bash
sudo apt-get install python3-venv
python3 -m venv --system-site-packages .venv-onnx
source /opt/ros/humble/setup.bash
source install/setup.bash
source .venv-onnx/bin/activate
python -m pip install -r src/garbage_identify_yolov11/requirements-onnx.txt
python -m garbage_identify_yolov11.detector --ros-args \
  -p model_path:=best1.onnx -p backend:=onnxruntime
```

使用 `python -m` 确保运行的是上述 venv 解释器。
默认 `model_path` 为空时节点不加载模型，也不发布虚构识别结果。
默认输入为命名空间内的 `camera/color/image_raw`，输出为 `DetectMsg`。
默认置信度保留原值 `0.88`；应根据实际传感器和使用场景另行评估效果。
