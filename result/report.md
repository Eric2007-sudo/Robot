# DetectorNode 任务报告

## 一、培训感受与难度
1. Foxglove 部分感觉还不太熟悉。
2. ROS 2 部分有点停留在理论层面，真正实践、从零开始搭建一个节点时遇到了很多困难，求助了 AI。

## 二、对后续培训的建议
可以在理论部分设计一些小实操，这样到实践部分跨度不会太大。

## 三、收获与考察点
- 核心考察点：从零开始搭建 ROS 2 节点，以及熟悉四种通信方式（本项目主要实现 topic 的发布/订阅）。
- 其他部分（CMake 构建、launch、可视化调试）主要是为了让节点正常运行、方便调试；这些辅助部分个人感觉即使出错，交给 AI 改到能跑通即可。

## 四、完成情况
- 完成了 `config/params.yaml` 参数补充、`src/detector_node.cpp` 节点实现、`CMakeLists.txt` 构建配置、`launch/detector.launch.py`。
- `rm -rf build install log` 后从零 `./compile.sh` 编译通过。
- 运行验证：节点正常启动；图像回调被触发，检测话题持续发布（约 15~17 Hz，CPU 推理）；`detections` 消息的 header 透传、`mode/label`、四个关键点字段正确；标注图与二值图正常发布；`foxglove_bridge` 监听 8765，可在 Foxglove 查看标注图像。
- 录包循环回放过程中节点不崩、不报错。
