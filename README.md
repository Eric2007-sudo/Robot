# 先读这里

**这题你要写的代码约 60 行。**

目录里有 40 多个文件，其中 41 张是标定图片，代码只有一个文件：

| | 内容 | 行数 |
|---|---|---|
| **要你写** | `calibrate.py` 里 **7 个 TODO** | 约 60 行 |
| 已给你 | 参数解析、主流程、终端输出 | 约 220 行 |

`calibrate.py` 一共 286 行，7 个 TODO 分别在第 **70、101、119、155、179、209、238** 行。

## 第一步

```bash
pip install opencv-python numpy
python3 calibrate.py
```

**骨架现在就能运行。** 它会报"0 张图检测成功"，因为标定板规格还是占位值，函数也是空的。这个报错是预期的，从这里开始往下填。

## 卡住了怎么办

- 每个 TODO 上面的 docstring 写清了要做什么、用哪个 OpenCV 函数、容易在哪里出错
- 把 docstring 和报错一起贴给 AI 提问，这是我们鼓励的做法
- 原理见配套文档：[12-相机标定与位姿解算](https://hello-world-vision.github.io/Vision_Website/induction-training/12-相机标定与位姿解算/)，模块一对应标定，模块三对应 PnP

## 然后看 QUESTION.md

完整的任务说明、评分关注点和提交要求都在 `QUESTION.md` 里。
