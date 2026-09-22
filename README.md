# Car Detector（YOLO 车辆检测）

基于 OpenCV `dnn` 模块加载 YOLO ONNX 模型的车辆检测器，用于 RoboMaster 视觉链路的前端：在赛场俯视图里检测出车辆，为后续定位与小地图共享提供目标框。

整个流程只用 OpenCV 一个库完成推理（**不需要 PyTorch / CUDA / onnxruntime**），CPU 即可运行。

## 功能

- **端到端检测**：`letterbox` → `preprocess` → `net.forward` → `decode` → `nms`，输入图片直接得到检测框。
- **等比缩放预处理**：保持长宽比缩放到模型输入尺寸，短边用灰边（114）填充，避免拉伸导致框变形。
- **坐标还原**：把网络输出坐标按 letterbox 参数还原回原图像素坐标，并裁剪到图像范围内。
- **非极大值抑制**：手写 IoU 与贪心 NMS，去除同一辆车的重叠框。
- **结果可视化**：把框、类别、置信度画到原图，按运行时间戳输出到 `result/`，并写出统计 `summary.txt`。

## 环境依赖

- CMake ≥ 3.16
- C++17 编译器（GCC / Clang / MSVC）
- OpenCV 4（需要 `core`、`imgproc`、`imgcodecs`、`dnn` 模块）

```bash
# Ubuntu / Debian
sudo apt install build-essential cmake libopencv-dev
```

## 编译与运行

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
./build/car_detector
```

程序读取 `config/params.yaml`、`weights/car.onnx` 与 `image/` 下所有图片，结果写入 `result/<时间戳>[_标签]/`。

## 使用方法

```bash
./build/car_detector                        # 用 config/params.yaml 跑 image/ 下所有图
./build/car_detector --conf 0.4             # 临时改置信度阈值
./build/car_detector --nms 0.3              # 临时改 NMS IoU 阈值
./build/car_detector --tag baseline         # 给本次运行起名
./build/car_detector --image image/03.jpg   # 只跑单张，调试用
./build/car_detector --help                 # 查看全部选项
```

### 命令行参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--config` | `config/params.yaml` | 参数文件路径 |
| `--weights` | 读配置文件 | ONNX 模型路径 |
| `--image-dir` | `image` | 图片目录 |
| `--image` | 空 | 只跑单张图（调试用） |
| `--result-dir` | `result` | 结果输出根目录 |
| `--tag` | 空 | 本次运行标签，会拼到输出目录名里 |
| `--conf` | 读配置文件 | 临时覆盖置信度阈值 |
| `--nms` | 读配置文件 | 临时覆盖 NMS IoU 阈值 |
| `--input-size` | 读配置文件 | 临时覆盖模型输入尺寸 |
| `--quiet` | 关闭 | 只输出汇总，不逐张打印 |

## 参数配置（`config/params.yaml`）

```yaml
weights_path: "weights/car.onnx"   # 模型路径
input_size: 640                    # 模型输入尺寸，须与导出时一致
conf_threshold: 0.30               # 置信度阈值
nms_threshold: 0.40                # NMS IoU 阈值
max_detections: 20                 # 单图最多保留的检测数
```

改完直接重跑即可，**不需要重新编译**。

## 实现说明

四个核心函数位于 `src/detector.cpp`：

### `letterbox()`
保持长宽比缩放到 `input_size_ × input_size_`：先按长边算缩放比例，再缩放，最后贴到灰底（`Scalar(114,114,114)`）方形画布中央，记录 `scale / pad_x / pad_y` 供坐标还原使用。

### `preprocess()`
用 `cv::dnn::blobFromImage` 把方形图转成网络输入 blob：`scalefactor = 1/255`（归一化到 0~1）、`swapRB = true`（BGR→RGB）、`crop = false`、输出 `CV_32F`，形状 `[1, 3, 640, 640]`。

### `decode()`
解析网络输出张量，逐候选提取框参数与置信度，过滤低分候选，并把坐标还原回原图：

```
x_原图 = (x_网络 - pad_x) / scale
```

其中横向用 `pad_x`、纵向用 `pad_y`；最后用 letterbox 参数反推原图尺寸并把框裁剪到图像范围内。

### `nms()`
按置信度降序排序，逐个保留最高分框，并剔除与它 IoU 超过 `nms_threshold_` 的其它框，结果数不超过 `max_detections_`。IoU = 交集面积 / 并集面积。

## 模型说明

`weights/car.onnx` 为训练好的 **YOLOv5** 模型（约 14 MB），单次前向输出形状 `[1, 25200, 10]`：

- 通道 0~3：框参数 `(cx, cy, w, h)`，网络输入尺度下的中心点与宽高
- 通道 4：objectness（该位置存在目标的置信度）
- 通道 5~9：5 个类别分数，类别为 `{0: car, 1: armor, 2: ignore, 3: watcher, 4: base}`

本检测器只关心 `car`（类别 0），置信度取 `objectness × car 分数`。

## 调参结果

对 `conf_threshold`（0.10~0.50）与 `nms_threshold`（0.20~0.60）做了 0.05 步长的网格扫描（81 组），最终选定：

| 参数 | 取值 |
|---|---|
| `conf_threshold` | **0.30** |
| `nms_threshold` | **0.40** |

选值理由：

- **conf 取 0.30**：再调高（如 0.40）会漏掉远处/被遮挡的低置信真车（实测有 3 辆车的置信度仅 0.31~0.35）；再调低则误检增多。0.30 在两者间取得平衡。
- **nms 取 0.40**：比默认 0.45 略强，可合并同一辆车的重复框，同时不会把挨得近的两辆车误合成一个。

该组参数下，20 张测试图每张检出 **3~7 辆**，合计 **106 个**，与预期规模一致。

## 目录结构

```
Car_Detector_Test/
├── CMakeLists.txt
├── config/
│   └── params.yaml          # 阈值等参数（改完免编译）
├── include/
│   ├── detector.hpp         # Detector 接口
│   └── visualize.hpp        # 画框、配置读取
├── src/
│   ├── main.cpp             # 主流程：遍历图片 → 检测 → 画框存图 + 统计
│   └── detector.cpp         # 四个核心函数实现
├── weights/
│   └── car.onnx             # YOLOv5 车辆检测模型
├── image/                   # 测试图片（20 张 1600×1069）
├── result/                  # 输出：画框图 + summary.txt（按时间戳分目录）
└── README.md
```

## 结果输出

每次运行在 `result/` 下新建带时间戳的目录，不覆盖上一次：

```
result/20260922_194728_conf030/
├── 00.jpg ... 19.jpg    画好检测框的图（框 + 类别 + 置信度）
└── summary.txt          本次参数 + 每张图的检出数与推理耗时
```

## 说明

- **坐标系还原是关键**：`decode` 若漏掉 `pad` 或 `scale`，框会整体偏移或缩放，图上一眼可见。
- **输入尺寸须一致**：`params.yaml` 的 `input_size` 必须与导出 ONNX 时的尺寸一致，否则检测结果错误。
- **Windows 编译**：`CMakeLists.txt` 已按编译器区分告警选项，并为 MSVC 加了 `/utf-8`（源码含中文注释）；`main.cpp` 的 `localtime_r` 在 Windows 下走 `localtime_s` 分支。
