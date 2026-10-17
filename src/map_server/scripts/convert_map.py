#!/usr/bin/env python3
"""把比赛地形标注地图（msgpack）转成 ROS 标准地图（PGM + YAML）。

来源地图：src/map_server/maps/RMUC202605.msgpack
它是一个 map：width / height / resolution / terrain / direction。
terrain 每格是一个语义标签：

    0 FLAT        平地（可通行）
    1 OBSTACLE    障碍（墙）
    2 SLOPE       斜坡
    3 STEP_L1     一级台阶
    4 STEP_L2     二级台阶
    5 FLY_SLOPE   飞坡
    6 STEP_HIGH   高地台阶

本脚本只保留"墙 / 可通行"二值信息。映射规则：

    1 OBSTACLE  -> 0  （黑，墙）
    其余标签    -> 254（白，可通行；坡/台阶/飞坡对差速车视为可通行）

说明：坡/台阶/飞坡在原地图里是"连接各区域的通道"（轮腿专用）。若把它们也当墙，
整张 28x15m 场地会被切成 7 个互不连通的小块，无法跨半场导航；对简化的差速车
仿真，把它们当可通行，场地保持连通（见 docs/Part_of_Map.md）。

msgpack 的行序直接对应地图坐标，row=0 对应世界 y 最小的一侧。

运行（需要 python3-msgpack）：
    python3 src/map_server/scripts/convert_map.py
    # 可选：--src / --out-pgm / --out-yaml 改路径

产出：
    src/map_server/maps/RMUC202605.pgm
    src/map_server/maps/RMUC202605.yaml
"""
import argparse
import os

import msgpack

DEFAULT_SRC = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "maps", "RMUC202605.msgpack"))
DEFAULT_OUT_PGM = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "maps", "RMUC202605.pgm"))
DEFAULT_OUT_YAML = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "maps", "RMUC202605.yaml"))

FREE_GRAY = 254  # 自由
OCC_GRAY = 0     # 障碍（含地形）


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=DEFAULT_SRC)
    ap.add_argument("--out-pgm", default=DEFAULT_OUT_PGM)
    ap.add_argument("--out-yaml", default=DEFAULT_OUT_YAML)
    args = ap.parse_args()

    with open(args.src, "rb") as f:
        data = msgpack.unpackb(f.read(), raw=False)

    width = int(data["width"])
    height = int(data["height"])
    resolution = float(data["resolution"])
    terrain = data["terrain"]
    assert len(terrain) == width * height, "terrain size mismatch"

    # terrain -> 灰度，保持地图坐标的行序
    # 只有 OBSTACLE(1) 是墙，其余（平地/坡/台阶/飞坡）可通行。
    gray = bytearray(width * height)
    free = 0
    for y in range(height):
        dst_row = y * width
        for x in range(width):
            if terrain[dst_row + x] == 1:  # OBSTACLE
                gray[dst_row + x] = OCC_GRAY
            else:
                gray[dst_row + x] = FREE_GRAY
                free += 1

    os.makedirs(os.path.dirname(args.out_pgm), exist_ok=True)
    with open(args.out_pgm, "wb") as f:
        f.write(f"P5\n{width} {height}\n255\n".encode())
        f.write(bytes(gray))

    with open(args.out_yaml, "w") as f:
        f.write(
            f"image: {os.path.basename(args.out_pgm)}\n"
            f"resolution: {resolution}\n"
            f"origin: [0.0, 0.0, 0.0]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.196\n"
        )

    print(f"src     : {args.src}")
    print(f"size    : {width} x {height} @ {resolution} m "
          f"-> {width * resolution:.2f} m x {height * resolution:.2f} m")
    print(f"free    : {free} cells ({100.0 * free / (width * height):.1f}%)")
    print(f"pgm     : {args.out_pgm}")
    print(f"yaml    : {args.out_yaml}")


if __name__ == "__main__":
    main()
