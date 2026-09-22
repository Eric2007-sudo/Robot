# 先读这里（30 秒）

**这题你要写的代码约 65 行。**

目录里有 20 多个文件和一个 14 MB 的模型，但模型是训好的，你只负责调用：

| | 内容 | 行数 |
|---|---|---|
| **要你写** | `src/detector.cpp` 里 **4 个 TODO** | 约 65 行 |
| 已给你 | 主流程、画框、图片遍历、结果统计 | 约 520 行 |

另外要改 `config/params.yaml` 里的两个阈值，改完不用重新编译。

**不需要训练模型，不需要 GPU，不需要装 PyTorch。** OpenCV 自带的 dnn 模块就能完成推理。

## 第一步

```bash
sudo apt install build-essential cmake libopencv-dev
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
./build/car_detector
```

**骨架现在就能编译运行。** 跑完后 `result/` 里的图上没有任何检测框，因为 4 个 TODO 还是空的。填完就会出现框。

## 卡住了怎么办

- 每个 TODO 上面都有注释，写清了要做什么、用哪个函数、容易在哪里出错
- `decode()` 最需要动脑，注释里建议你先把张量维度打印出来确认
- 把注释和报错一起贴给 AI 提问，这是我们鼓励的做法
- 原理见配套文档：[20-关于神经网络](https://hello-world-vision.github.io/Vision_Website/induction-training/20-关于神经网络/)、[10-OpenCV](https://hello-world-vision.github.io/Vision_Website/induction-training/10-opencv/)

## 然后看 QUESTION.md

完整的任务说明、评分关注点和提交要求都在 `QUESTION.md` 里。
