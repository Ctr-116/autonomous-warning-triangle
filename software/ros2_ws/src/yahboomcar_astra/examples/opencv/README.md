# 原版 OpenCV 教学例程

这里保留原 `yahboomcar_astra/scripts/opencv` 的 21 个非 ROS 教学脚本和 3 张图片。它们使用 OpenCV 读图、显示窗口及处理图像，不包含 ROS 1 节点，因此不需要改写为 ROS 2 节点。

在有桌面图形环境的 Ubuntu 中，先进入这个目录，再运行对应脚本，例如 `python3 2_1.py`。`1_3.py` 会打开摄像头，需要实际设备；其他脚本会打开图形窗口。无桌面环境时不要直接启动这些窗口例程。

迁移时移除了原程序对 NumPy 图像调用 `img.release()` 的无效清理语句，保留摄像头的 `VideoCapture.release()`。图片保持原样，来源、修改与文件校验值记录在 `provenance.json`。

已检查 Python 3 语法。GUI 交互和摄像头运行尚未在实机验收。
