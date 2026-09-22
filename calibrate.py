#!/usr/bin/env python3
# ============================================================================
#  棋盘格相机标定 —— 骨架代码
#
#  已经给好的：
#      parse_board()          标定板规格字符串解析
#      parse_args()           命令行参数框架（可以按需增删）
#      main()                 主流程骨架，调用顺序已排好
#
#  需要你实现的（函数体里标了 TODO），合计约 60 行：
#      --board 默认值         改成你确定出来的标定板规格  <- 第 70 行
#      build_object_points()  生成理论角点坐标            <- 第 101 行
#      find_corners()         角点检测 + 亚像素精确化      <- 第 119 行
#      collect()              遍历图片，收集 3D-2D 对应   <- 第 155 行
#      per_view_errors()      逐张重投影误差              <- 第 179 行
#      write_result()         结果写入 YAML               <- 第 209 行
#      solve_pnp_all()        PnP 位姿解算（任务 5）       <- 第 238 行
#
#  本文件 286 行，其余是已经写好的参数解析和主流程，不用改。
#
#  当前状态可以直接运行，但因为几个函数还是空的，标定会直接失败退出。
#  填完 TODO 才会输出结果。
#
#  用法：
#      python3 calibrate.py                          # 用默认参数标定 image/ 下的图片
#      python3 calibrate.py --board 9x6 --square 17  # 显式指定标定板规格
#      python3 calibrate.py --help                   # 看全部参数
# ============================================================================

import argparse
import glob
import os
import sys

import cv2
import numpy as np

# 亚像素精确化的迭代终止条件：迭代 30 次，或角点移动小于 0.001 像素
SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
SUBPIX_WIN = (11, 11)


def parse_board(text):
    """解析形如 9x6 的标定板规格，返回 (cols, rows)，单位是内部角点数。"""
    try:
        cols, rows = (int(v) for v in text.lower().split("x"))
    except ValueError:
        raise argparse.ArgumentTypeError(f"标定板规格应写成 列x行，例如 9x6，收到的是 {text!r}")
    if cols < 2 or rows < 2:
        raise argparse.ArgumentTypeError("内部角点数至少为 2x2")
    if cols == rows:
        # 正方形板存在 90 度旋转歧义，同一块板在不同图里的角点编号可能对不上
        print("[warn] 正方形棋盘格的角点顺序存在旋转歧义，建议换成长宽不等的板子", file=sys.stderr)
    return cols, rows


def parse_args(argv=None):
    """命令行参数。默认值应当使得直接跑 `python3 calibrate.py` 就能标定本题数据。

    这里给的是一个够用的起点，你可以按自己的实现增删参数。
    判断一个值该不该外置的标准是：换一批数据时它会不会变。
    """
    here = os.path.dirname(os.path.abspath(__file__))
    p = argparse.ArgumentParser(
        description="基于 OpenCV 的单目棋盘格标定",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--image-dir", default=os.path.join(here, "image"), help="标定图片所在目录")
    p.add_argument("--glob", default="*.jpg", help="图片名匹配模式")
    # TODO: 默认值填你确定出来的标定板规格
    p.add_argument("--board", type=parse_board, default="9x6",
                   help="棋盘格内部角点数，写成 列x行（不是方格数）")
    p.add_argument("--square", type=float, default=17.0, help="方格边长，单位 mm")
    p.add_argument("--output", default=os.path.join(here, "result", "calibration_result.yaml"),
                   help="结果文件路径")
    p.add_argument("--pnp", action="store_true",
                   help="标定完顺便做 PnP 位姿解算（任务 5）")
    p.add_argument("--pnp-output", default=os.path.join(here, "result", "pnp_result.yaml"),
                   help="PnP 结果文件路径")
    p.add_argument("--visualize", action="store_true",
                   help="输出角点图、去畸变对比图、位姿坐标轴图到 result/ 下（任务 4）")
    p.add_argument("--visualize-limit", type=int, default=0,
                   help="验证图最多输出多少张，0 表示全部")
    return p.parse_args(argv)


def build_object_points(board, square):
    """生成标定板坐标系下的理论角点坐标。

    board 是 (cols, rows) 内部角点数，square 是方格边长（mm）。
    返回 shape 为 (cols*rows, 3) 的 float32 数组。

    TODO 1: 实现它。

    要点：
      - 把板面当作 Z=0 平面，所以第三列全是 0
      - 角点的排列顺序必须和 findChessboardCorners 的输出顺序一致，
        否则 3D-2D 对应错位，标定结果会错得莫名其妙且不报错
      - 乘上 square 之后单位是 mm。想清楚这个尺度影响的是内参还是外参

    提示：np.mgrid 可以一步生成网格坐标。
    """
    cols, rows = board

    gy, gx = np.mgrid[0:rows, 0:cols]
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, 0] = gx.ravel()
    objp[:, 1] = gy.ravel()
    objp *= square
    return objp


def find_corners(gray, board):
    """在灰度图上检测棋盘格角点，返回角点数组；检测失败返回 None。

    TODO 2: 实现它。

    两步：
      1. findChessboardCorners 粗检测。建议加上
         CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE 这两个标志位，
         对光照不均的图片更稳
      2. cornerSubPix 亚像素精确化，用上面的 SUBPIX_CRITERIA 和 SUBPIX_WIN

    第 2 步不能省。findChessboardCorners 只有整像素精度，
    省掉亚像素精化，最终误差会差出好几倍——实现完自己注释掉对比一下。
    """
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    retval, corners = cv2.findChessboardCorners(gray, board, flags)
    if not retval:
        return None
    corners = cv2.cornerSubPix(gray, corners, SUBPIX_WIN, (-1, -1), SUBPIX_CRITERIA)
    return corners


def collect(args):
    """遍历图片，收集所有成功检测的 3D-2D 对应。

    返回 (obj_points, img_points, used, failed, image_size)：
        obj_points  每张图的理论角点列表
        img_points  每张图的实测角点列表
        used        检测成功的文件名列表
        failed      检测失败的文件名列表
        image_size  (width, height)

    TODO 3: 实现它。

    每张图的处理流程：读图 → 转灰度 → find_corners() → 成功则记入两个列表。

    必须处理的边界情况：
      - 目录下没有匹配的图片：报错退出，别让后面的代码拿着空列表跑
      - 单张图片读取失败或检测不到角点：记入 failed 跳过，不要让整个程序挂掉
      - 图片尺寸不一致：应当拒绝标定。想清楚为什么——
        不同分辨率对应不同的内参，混在一起标出来的结果是什么含义？
      - 成功的图片太少（比如少于 3 张）：约束不够，报错并提示检查 --board

    注意 OpenCV 的尺寸约定：cv2 的 img.shape 是 (height, width)，
    而 calibrateCamera 要的 imageSize 是 (width, height)，顺序相反。
    """
    files = sorted(glob.glob(os.path.join(args.image_dir, args.glob)))
    if not files:
        sys.exit(f"[error] {args.image_dir} 下没有匹配 {args.glob} 的图片")

    grid = build_object_points(args.board, args.square)
    obj_points, img_points, used, failed = [], [], [], []
    image_size = None

    for f in files:
        name = os.path.basename(f)
        img = cv2.imread(f)
        if img is None:
            failed.append(name)
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        h, w = gray.shape[:2]
        if image_size is None:
            image_size = (w, h)
        elif (w, h) != image_size:
            sys.exit(f"[error] 图片尺寸不一致，拒绝标定：{name} 是 {w}x{h}，"
                     f"而之前的是 {image_size[0]}x{image_size[1]}")

        corners = find_corners(gray, args.board)
        if corners is None:
            failed.append(name)
            continue
        obj_points.append(grid)
        img_points.append(corners)
        used.append(name)

    if len(used) < 3:
        sys.exit(f"[error] 只有 {len(used)} 张图检测成功，无法标定")
    assert image_size is not None
    return obj_points, img_points, used, failed, image_size


def per_view_errors(obj_points, img_points, rvecs, tvecs, K, dist):
    """逐张计算重投影误差（RMS，单位像素），返回一维 ndarray。

    TODO 4: 实现它。

    对每张图片：用 projectPoints 把理论角点按该图的外参投影回图像，
    与实测角点求像素距离，再对该图的所有角点取 RMS。

    为什么不能只用 calibrateCamera 的返回值？
    那个值是所有角点合在一起的总体 RMS，看不出是哪张图片拖后腿。
    有了逐张误差，才能定位到具体是哪几张拍虚了。

    注意 projectPoints 的输出 shape 是 (N, 1, 2)，
    和 img_points 里的角点一样，求差之前先 reshape(-1, 2)。
    """
    errors = []

    for i in range(len(obj_points)):
        proj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], K, dist)
        diff = img_points[i].reshape(-1, 2) - proj.reshape(-1, 2)
        sq = np.sum(diff ** 2, axis=1)
        errors.append(np.sqrt(np.mean(sq)))

    return np.array(errors)


def write_result(args, used, failed, image_size, K, dist, rms, errors):
    """把标定结果写入 YAML。

    TODO 5: 实现它。

    用 cv2.FileStorage 写，这样 C++ 和 Python 都能直接读回：

        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
        fs.write("camera_matrix", K)          # 矩阵直接写
        fs.write("fx", float(K[0, 0]))        # 标量记得转 float
        fs.writeComment("这是注释")            # 可选，但建议给关键字段加
        fs.release()                          # 别忘了

    写字符串列表（比如用到的图片名）要用 startWriteStruct / endWriteStruct：

        fs.startWriteStruct("images_used_list", cv2.FILE_NODE_SEQ)
        for name in used:
            fs.write("", name)
        fs.endWriteStruct()

    需要包含哪些字段见 QUESTION.md 任务 3。
    衡量标准是：半年后别人拿到这个文件，不需要问你任何问题。

    记得先 os.makedirs 确保输出目录存在。
    """
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    fs = cv2.FileStorage(args.output, cv2.FILE_STORAGE_WRITE)

    fs.write("image_width", int(image_size[0]))
    fs.write("image_height", int(image_size[1]))
    fs.write("board_cols", int(args.board[0]))
    fs.write("board_rows", int(args.board[1]))
    fs.write("square_size_mm", float(args.square))

    fs.write("camera_matrix", K)
    fs.write("distortion_coefficients", dist.reshape(1, 5))

    fs.write("rms_reprojection_error", float(rms))
    fs.write("mean_per_view_error", float(errors.mean()))
    fs.write("max_per_view_error", float(errors.max()))
    fs.write("worst_image", used[int(errors.argmax())])
    fs.write("per_view_errors", errors.reshape(-1, 1))

    fs.write("images_used", int(len(used)))
    fs.write("images_total", int(len(used) + len(failed)))

    fs.startWriteStruct("images_used_list", cv2.FILE_NODE_SEQ)
    for name in used:
        fs.write("", name)
    fs.endWriteStruct()

    fs.startWriteStruct("images_failed_list", cv2.FILE_NODE_SEQ)
    for name in failed:
        fs.write("", name)
    fs.endWriteStruct()

    fs.release()


def solve_pnp_all(args, used, obj_points, img_points, K, dist):
    """任务 5：用标定出的内参，对每张图解一次 PnP。

    TODO 6: 实现它（任务 5，见 QUESTION.md）。

    obj_points / img_points 是 collect() 已经收集好的 3D-2D 对应，
    直接复用即可，不用重新检测角点。

    对每一张：
      1. cv2.solvePnP(objp, corners, K, dist) 解出 rvec, tvec
         flags 可选 SOLVEPNP_ITERATIVE / SOLVEPNP_IPPE / SOLVEPNP_SQPNP，
         建议三种都试试，对比重投影误差（这是加分点）
      2. 用 cv2.projectPoints 算重投影误差，应当和标定的逐张误差量级相当
      3. 距离 = np.linalg.norm(tvec)，单位由 objp 决定（本题是 mm）
      4. 刚体校验：R, _ = cv2.Rodrigues(rvec)，把角点变换到相机系
             pc = (R @ objp.T + tvec).T
         再量任意两点距离（比如首尾对角线），它必须恒等于原始 objp 里的
         同一距离 —— 刚体变换不改变长度。这是实现正确性的硬校验，
         偏差应当是浮点精度级别（1e-5 mm 以下）

    返回一个列表，每项含 image / distance_mm / repro_error / diagonal_mm，
    然后写进 result/pnp_result.yaml（字段格式见 QUESTION.md 任务 5）。

    提示：棋盘格真值对角线 = np.linalg.norm(objp[-1] - objp[0])，
    9x6 格距 17mm 时应当是 160.38 mm。
    """
    # TODO: 实现 PnP 解算
    rows = []
    for i in range(len(used)):
        objp = obj_points[i]
        corners = img_points[i]

        ok, rvec, tvec = cv2.solvePnP(objp, corners, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)

        proj, _ = cv2.projectPoints(objp, rvec, tvec, K, dist)
        diff = corners.reshape(-1, 2) - proj.reshape(-1, 2)
        repro = float(np.sqrt(np.mean(np.sum(diff ** 2, axis=1))))

        distance = float(np.linalg.norm(tvec))

        R, _ = cv2.Rodrigues(rvec)
        pc = (R @ objp.T + tvec).T
        diagonal = float(np.linalg.norm(pc[-1] - pc[0]))

        rows.append(dict(image=used[i], distance_mm=distance,
                         repro_error=repro, diagonal_mm=diagonal))

    true_diag = float(np.linalg.norm(obj_points[0][-1] - obj_points[0][0]))
    errs = np.array([r["repro_error"] for r in rows])
    ds = np.array([r["distance_mm"] for r in rows])

    os.makedirs(os.path.dirname(os.path.abspath(args.pnp_output)), exist_ok=True)
    fs = cv2.FileStorage(args.pnp_output, cv2.FILE_STORAGE_WRITE)
    fs.write("board_true_diagonal_mm", true_diag)
    fs.write("pnp_repro_error_mean", float(errs.mean()))
    fs.write("pnp_repro_error_max", float(errs.max()))
    fs.write("distance_range_mm", np.array([[ds.min(), ds.max()]], np.float32))

    fs.startWriteStruct("per_view", cv2.FILE_NODE_SEQ)
    for r in rows:
        fs.startWriteStruct("", cv2.FILE_NODE_MAP)
        fs.write("image", r["image"])
        fs.write("distance_mm", r["distance_mm"])
        fs.write("repro_error", r["repro_error"])
        fs.write("diagonal_mm", r["diagonal_mm"])
        fs.endWriteStruct()
    fs.endWriteStruct()

    fs.release()
    return rows


def draw_visuals(args, used, img_points, K, dist, rvecs, tvecs, obj_points):
    """任务 4 的可选验证图输出：角点图、去畸变前后对比、位姿坐标轴图。

    只在 --visualize 打开时由 main() 调用。图片写入 result/ 下的
    corners/、undistorted/、pose/ 三个子目录；--visualize-limit 限制张数
    （0 表示全部）。rvecs/tvecs 复用 calibrateCamera 的输出，不重新求解。
    """
    root = os.path.dirname(os.path.abspath(args.output))
    dir_corners = os.path.join(root, "corners")
    dir_undist = os.path.join(root, "undistorted")
    dir_pose = os.path.join(root, "pose")
    for d in (dir_corners, dir_undist, dir_pose):
        os.makedirs(d, exist_ok=True)

    # --visualize-limit 限制输出张数，0 表示全部
    limit = args.visualize_limit if args.visualize_limit > 0 else len(used)
    # 坐标轴长度取 3 个方格边长，太小看不见、太大遮挡画面
    axis_len = args.square * 3
    for i, name in enumerate(used):
        if i >= limit:
            break
        img = cv2.imread(os.path.join(args.image_dir, name))
        if img is None:
            continue

        # 角点图：在图上标出检测到的 54 个角点，用于检查检测是否准确
        marked = img.copy()
        cv2.drawChessboardCorners(marked, args.board, img_points[i], True)
        cv2.imwrite(os.path.join(dir_corners, name), marked)

        # 去畸变对比：左原图、右校正图并排，用于观察边缘直线是否被拉直
        undistorted = cv2.undistort(img, K, dist)
        cv2.imwrite(os.path.join(dir_undist, name), np.hstack([img, undistorted]))

        # 位姿图：用该图外参画出板坐标系的三轴（红X/绿Y/蓝Z），直观看板面朝向
        axes = img.copy()
        cv2.drawFrameAxes(axes, K, dist, rvecs[i], tvecs[i], axis_len)
        cv2.imwrite(os.path.join(dir_pose, name), axes)

    print(f"[info] 验证图已写入 {root} 下的 corners/、undistorted/、pose/")


def main(argv=None):
    args = parse_args(argv)
    print(f"[info] 标定板 {args.board[0]}x{args.board[1]} 内部角点，方格边长 {args.square} mm")

    obj_points, img_points, used, failed, image_size = collect(args)

    # calibrateCamera 的第一个返回值是总体 RMS 重投影误差，
    # 按全部图片的所有角点一起算出来的
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, image_size, None, None)
    errors = per_view_errors(obj_points, img_points, rvecs, tvecs, K, dist)

    # 终端输出，不能只写文件
    print()
    print(f"图像尺寸      : {image_size[0]}x{image_size[1]}")
    print(f"有效图片      : {len(used)}/{len(used) + len(failed)}")
    print(f"内参矩阵      :\n{np.array2string(K, precision=4, suppress_small=True)}")
    print(f"畸变系数      : {np.array2string(dist.ravel(), precision=6, suppress_small=True)}")
    print(f"总体 RMS 误差 : {rms:.4f} px")
    if errors.size:
        print(f"单图误差      : 均值 {errors.mean():.4f} px，最大 {errors.max():.4f} px"
              f"（{used[int(errors.argmax())]}）")

    write_result(args, used, failed, image_size, K, dist, rms, errors)
    print(f"[info] 结果已写入 {args.output}")

    # 任务 4：输出验证图（角点、去畸变对比、位姿坐标轴）
    if args.visualize:
        draw_visuals(args, used, img_points, K, dist, rvecs, tvecs, obj_points)

    # 任务 5：位姿解算
    if args.pnp:
        print()
        print("[info] 开始 PnP 位姿解算")
        pnp_rows = solve_pnp_all(args, used, obj_points, img_points, K, dist)
        if not pnp_rows:
            print("[warn] PnP 没有输出结果（solve_pnp_all 还没实现？）")
        else:
            errs = np.array([r["repro_error"] for r in pnp_rows])
            ds = np.array([r["distance_mm"] for r in pnp_rows])
            print(f"解算图片      : {len(pnp_rows)}")
            print(f"重投影误差    : 均值 {errs.mean():.4f} px，最大 {errs.max():.4f} px")
            print(f"距离范围      : {ds.min():.1f} ~ {ds.max():.1f} mm")

    return 0


if __name__ == "__main__":
    sys.exit(main())
