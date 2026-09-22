// ============================================================================
//  检测器实现 —— 本文件是你唯一需要修改的文件
//
//  已经给好的：load()、detect()（负责按顺序调用你写的四个函数）
//
//  需要你实现的四个函数（函数体里标了 TODO），合计约 65 行：
//      letterbox()    预处理：等比缩放 + 填充   <- 第 80 行
//      preprocess()   图像转 blob              <- 第 99 行
//      decode()       解析网络输出              <- 第 135 行（最需要动脑的一个）
//      nms()          非极大值抑制              <- 第 166 行
//
//  本文件 172 行，其余是已经写好的 load() 和 detect()，不用改。
// ============================================================================

#include "detector.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace car_detector
{

bool Detector::load(const std::string & onnx_path)
{
  try {
    net_ = cv::dnn::readNetFromONNX(onnx_path);
  } catch (const cv::Exception & e) {
    return false;
  }
  if (net_.empty()) {return false;}
  net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
  return true;
}

std::vector<Detection> Detector::detect(const cv::Mat & image)
{
  if (net_.empty() || image.empty()) {return {};}

  cv::Mat padded;
  LetterboxInfo info{};
  letterbox(image, padded, info);

  const cv::Mat blob = preprocess(padded);
  if (blob.empty()) {return {};}

  net_.setInput(blob);

  const double t0 = static_cast<double>(cv::getTickCount());
  const cv::Mat output = net_.forward();
  last_inference_ms_ =
    (static_cast<double>(cv::getTickCount()) - t0) / cv::getTickFrequency() * 1000.0;

  const std::vector<Detection> candidates = decode(output, info);
  return nms(candidates);
}

// ---------------------------------------------------------------- 要你写
/**
 * TODO 1: letterbox 预处理
 *
 * 把任意尺寸的原图变成 input_size_ x input_size_ 的方形图，且不改变长宽比。
 *
 * 步骤：
 *   1. 算缩放比例 scale = input_size_ / max(原图宽, 原图高)
 *   2. 按 scale 等比缩放（cv::resize）
 *   3. 创建 input_size_ x input_size_ 的灰底图，底色 cv::Scalar(114,114,114)
 *   4. 把缩放后的图贴进去，记录 pad_x / pad_y
 *   5. scale / pad_x / pad_y 都填进 info
 *
 * 关于贴图位置：贴左上角（pad_x=pad_y=0）或者居中都可以，
 * 但 info 里记的值必须和实际贴的位置一致，否则 decode 还原坐标会偏。
 *
 * 为什么不能直接 resize 成正方形？图会被拉变形，模型没见过这种输入，
 * 检测框会跟着歪。自己可以试试直接 resize，看看框偏成什么样。
 */
void Detector::letterbox(const cv::Mat & src, cv::Mat & dst, LetterboxInfo & info) const
{
  if (src.empty()) {
    dst = cv::Mat();
    info = LetterboxInfo{1.0f, 0, 0};
    return;
  }

  const float scale = static_cast<float>(input_size_) / static_cast<float>(std::max(src.cols, src.rows));

  const int new_w = static_cast<int>(std::round(src.cols * scale));
  const int new_h = static_cast<int>(std::round(src.rows * scale));

  cv::Mat resized;
  cv::resize(src, resized, cv::Size(new_w, new_h));

  dst = cv::Mat(input_size_, input_size_, src.type(), cv::Scalar(114, 114, 114));

  const int pad_x = (input_size_ - resized.cols) / 2;
  const int pad_y = (input_size_ - resized.rows) / 2;
  resized.copyTo(dst(cv::Rect(pad_x, pad_y, resized.cols, resized.rows)));

  info.scale = scale;
  info.pad_x = pad_x;
  info.pad_y = pad_y;
}

/**
 * TODO 2: 图像转网络输入 blob
 *
 * 用 cv::dnn::blobFromImage，需要做的事：
 *   - 归一化：像素值从 0~255 缩到 0~1，即 scalefactor = 1/255
 *   - 尺寸：input_size_ x input_size_
 *   - 通道顺序：OpenCV 读图是 BGR，YOLO 要 RGB，所以 swapRB = true
 *   - crop = false
 *
 * 返回的 blob 形状是 [1, 3, input_size_, input_size_]（NCHW）。
 */
cv::Mat Detector::preprocess(const cv::Mat & padded) const
{
  return cv::dnn::blobFromImage(
    padded, 1.0 / 255.0, cv::Size(input_size_, input_size_),
    cv::Scalar(), true, false, CV_32F);
}

/**
 * TODO 3: 解析网络输出
 *
 * 这一步最容易出错，动手前先把输出的维度搞清楚。
 *
 * 本模型是单类别（car），YOLO 的输出通常是 [1, 4+1, N] 或 [1, N, 4+1]：
 *   4 个框参数 + 1 个类别分数，N 是候选框个数（比如 8400）。
 *   框参数一般是 (cx, cy, w, h)，即中心点和宽高，且都是网络输入尺度下的值。
 *
 * 建议先打印一下维度确认，别凭猜：
 *      std::cout << output.size << std::endl;
 * 如果拿到的是 [1, 5, 8400]，说明要按列取；[1, 8400, 5] 则按行取。
 * 可以用 output.reshape() 把它整成方便遍历的形状。
 *
 * 对每个候选框：
 *   1. 取出类别分数，低于 conf_threshold_ 的直接跳过
 *   2. 把 (cx, cy, w, h) 转成 (x1, y1, x2, y2)
 *   3. 坐标还原回原图 —— 先减掉 letterbox 的填充，再除以缩放比例：
 *          x_原图 = (x_网络 - info.pad_x) / info.scale
 *      这一步漏了的话，框会整体偏移或缩放，在图上一眼就能看出来
 *   4. 用 cv::Rect 存框，class_id 填 0
 *
 * 顺带一提：记得把框裁剪到图像范围内，负坐标和超出边界的框画出来会很难看。
 * 但 decode 拿不到原图尺寸，所以裁剪放在哪一步、怎么做，你自己想。
 */
std::vector<Detection> Detector::decode(
  const cv::Mat & output, const LetterboxInfo & info) const
{
  std::vector<Detection> candidates;
  if (output.empty() || output.dims != 3) {
    return candidates;
  }

  const int n = output.size[1];
  const cv::Mat out = output.reshape(1, n);

  for (int i = 0; i < n; ++i) {
    const float cx = out.at<float>(i, 0);
    const float cy = out.at<float>(i, 1);
    const float w  = out.at<float>(i, 2);
    const float h  = out.at<float>(i, 3);
    const float obj = out.at<float>(i, 4);
    const float car = out.at<float>(i, 5);
    const float score = obj * car;
    if (score < conf_threshold_) {continue;}

    float x1 = cx - w / 2;
    float y1 = cy - h / 2;
    float x2 = cx + w / 2;
    float y2 = cy + h / 2;

    x1 = (x1 - info.pad_x) / info.scale;
    y1 = (y1 - info.pad_y) / info.scale;
    x2 = (x2 - info.pad_x) / info.scale;
    y2 = (y2 - info.pad_y) / info.scale;

    const int orig_w =
      static_cast<int>(std::round((input_size_ - 2 * info.pad_x) / info.scale));
    const int orig_h =
      static_cast<int>(std::round((input_size_ - 2 * info.pad_y) / info.scale));

    x1 = std::clamp(x1, 0.0f, static_cast<float>(orig_w));
    y1 = std::clamp(y1, 0.0f, static_cast<float>(orig_h));
    x2 = std::clamp(x2, 0.0f, static_cast<float>(orig_w));
    y2 = std::clamp(y2, 0.0f, static_cast<float>(orig_h));

    const int bx = static_cast<int>(std::round(x1));
    const int by = static_cast<int>(std::round(y1));
    const int bw = static_cast<int>(std::round(x2)) - bx;
    const int bh = static_cast<int>(std::round(y2)) - by;
    if (bw <= 0 || bh <= 0) {continue;}

    Detection d;
    d.box = cv::Rect(bx, by, bw, bh);
    d.confidence = score;
    d.class_id = 0;
    candidates.push_back(d);
  }

  return candidates;
}

/**
 * TODO 4: 非极大值抑制
 *
 * 同一辆车往往会被检出好几个重叠的框，要只留最好的那个。
 *
 * 标准做法：
 *   1. 按置信度从高到低排序
 *   2. 取出当前置信度最高的框，加入结果
 *   3. 把与它 IoU 超过 nms_threshold_ 的其余框全部剔除
 *   4. 重复 2~3 直到没有候选框
 *   5. 结果数量不超过 max_detections_
 *
 * IoU = 交集面积 / 并集面积。cv::Rect 支持 & 求交、| 求并，
 * 所以 IoU 可以写得很短：
 *      float inter = (a & b).area();
 *      float iou   = inter / (a.area() + b.area() - inter);
 *
 * 也可以直接调 cv::dnn::NMSBoxes，但自己手写一遍更能说明你懂了，
 * 面试时我们会问 IoU 怎么算、阈值调大调小分别意味着什么。
 */
std::vector<Detection> Detector::nms(const std::vector<Detection> & candidates) const
{
  std::vector<Detection> result;

  std::vector<Detection> boxes = candidates;
  std::sort(boxes.begin(), boxes.end(),
    [](const Detection & a, const Detection & b) {
      return a.confidence > b.confidence;
    });

  const auto iou = [](const cv::Rect & a, const cv::Rect & b) {
      const float inter = static_cast<float>((a & b).area());
      const float uni = static_cast<float>(a.area() + b.area()) - inter;
      return inter / uni;
    };

  const int total = static_cast<int>(boxes.size());
  std::vector<bool> removed(boxes.size(), false);

  for (int i = 0; i < total; ++i) {
    if (removed[i]) {continue;}
    result.push_back(boxes[i]);
    if (static_cast<int>(result.size()) >= max_detections_) {break;}

    for (int j = i + 1; j < total; ++j) {
      if (removed[j]) {continue;}
      if (iou(boxes[i].box, boxes[j].box) > nms_threshold_) {
        removed[j] = true;
      }
    }
  }

  return result;
}

}  // namespace car_detector
