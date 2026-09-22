# Camera Calibration & PnP

基于 OpenCV 的单目相机标定与位姿解算工具，用于 RoboMaster 视觉链路的前端：由棋盘格图片标定出相机内参与畸变系数，再用标定结果通过 PnP 解算标定板（装甲板）相对相机的位姿。

## 功能

- **相机标定**：遍历标定图片，检测棋盘格角点并做亚像素精化，求解内参矩阵 `K`、畸变系数 `dist` 与每张图的外参。
- **误差评估**：逐张计算重投影误差（RMS），定位离群图片，并输出总体 RMS。
- **结果输出**：标定结果写入 OpenCV `FileStorage` 格式的 YAML，C++/Python 均可直接读回。
- **位姿解算（PnP）**：对每张成功检测的图片解算 `[R|t]`，输出平移距离、重投影误差，并用"刚体变换不改变长度"反算对角线做正确性校验。
- **可视化验证**：可选输出去畸变前后对比图、角点检测图、位姿坐标轴图。

## 环境依赖

- Python 3
- `opencv-python`
- `numpy`

```bash
pip install opencv-python numpy
```

## 目录结构

```
Camera_Calibration/
├── calibrate.py            # 主程序（标定 + PnP）
├── image/                  # 标定图片（41 张 1280x720 JPG）
├── result/                 # 输出目录
│   ├── calibration_result.yaml   # 内参、畸变、重投影误差
│   ├── pnp_result.yaml           # PnP 位姿解算结果
│   ├── corners/                  # 角点检测图（--visualize）
│   ├── undistorted/              # 去畸变前后对比图（--visualize）
│   └── pose/                     # 位姿坐标轴图（--visualize）
└── README.md
```

## 使用方法

```bash
# 直接标定（默认 9x6 内部角点、方格边长 17mm）
python3 calibrate.py

# 标定 + PnP 位姿解算
python3 calibrate.py --pnp

# 标定 + PnP + 输出验证图
python3 calibrate.py --pnp --visualize
```

### 命令行参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--image-dir` | `image` | 标定图片所在目录 |
| `--glob` | `*.jpg` | 图片名匹配模式 |
| `--board` | `9x6` | 棋盘格内部角点数，写成 `列x行`（不是方格数） |
| `--square` | `17.0` | 方格边长，单位 mm |
| `--output` | `result/calibration_result.yaml` | 标定结果输出路径 |
| `--pnp` | 关闭 | 标定完顺便做 PnP 位姿解算 |
| `--pnp-output` | `result/pnp_result.yaml` | PnP 结果输出路径 |
| `--visualize` | 关闭 | 输出去畸变/角点/位姿验证图到 `result/` |
| `--visualize-limit` | `0` | 验证图最多输出多少张，`0` 表示全部 |

## 输出说明

### `calibration_result.yaml`

```yaml
image_width / image_height      # 图像尺寸
board_cols / board_rows         # 标定板规格（内部角点数）
square_size_mm                  # 方格边长
camera_matrix                   # 3x3 内参矩阵
distortion_coefficients         # [k1, k2, p1, p2, k3]
rms_reprojection_error          # 总体 RMS（px）
mean_per_view_error             # 逐张误差均值
max_per_view_error              # 逐张误差最大值
worst_image                     # 最差图片文件名
per_view_errors                 # 逐张误差（Nx1）
images_used / images_total      # 有效/总图片数
images_used_list / images_failed_list
```

### `pnp_result.yaml`

```yaml
board_true_diagonal_mm          # 棋盘格真值对角线（160.38mm）
pnp_repro_error_mean / max      # PnP 重投影误差统计（px）
distance_range_mm               # 目标距离范围
per_view:                       # 逐张：image / distance_mm / repro_error / diagonal_mm
```

## 本次标定结果

| 项目 | 结果 |
|---|---|
| 有效图片 | 41 / 41 |
| 图像尺寸 | 1280 × 720 |
| 内参 | fx=913.18, fy=766.05, cx=642.63, cy=365.05 |
| 畸变 | [-0.4337, 0.2957, -0.00048, -0.00025, -0.1387] |
| 总体 RMS | 0.2621 px |
| 逐张误差 | 均值 0.2595 px，最大 0.3712 px |
| PnP 距离范围 | 147.1 ~ 267.1 mm |
| 刚体对角线校验 | 全部 160.377679 mm（互差 ~1e-14 mm） |

## 说明

- **标定板规格**：本题为 9×6 内部角点、方格边长 17mm，真值对角线 `17×√(8²+5²)=160.38mm`。
- **亚像素精化不可省**：`findChessboardCorners` 只有整像素精度，`cornerSubPix` 可把误差降低数倍。
- **参数外置**：所有随数据变化的量（图片目录、标定板规格、方格边长、输出路径）均通过命令行传入，无硬编码。
- **PnP 方法**：默认 `SOLVEPNP_ITERATIVE`。平面目标也可用 `SOLVEPNP_IPPE`，但在近正平面（倾角接近 0）时 IPPE 可能选错解，本题 `1.jpg`（倾角 4.4°）会出现约 1.5px 的离群。
