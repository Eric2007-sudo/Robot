// ============================================================================
//  YOLO 车辆检测器
//
//  本题用 OpenCV 的 dnn 模块加载一个 YOLO ONNX 模型，检测图片里的机器人车辆。
//  不需要 PyTorch、不需要 CUDA、不需要 onnxruntime —— OpenCV 一个库就够了。
//
//  已经给好的（不用改）
//  ---------------------------------------------------------------------------
//      Detection          检测结果的数据结构
//      load()             加载 ONNX 模型
//      drawDetections()   把结果画到图上（在 visualize.hpp 里）
//
//  需要你实现的四个函数（在 detector.cpp 里，标了 TODO）
//  ---------------------------------------------------------------------------
//      letterbox()        预处理：等比缩放 + 填充到模型输入尺寸
//      preprocess()       图像转成网络输入的 blob
//      decode()           解析网络输出，取出候选框
//      nms()              非极大值抑制，去掉重叠框
//
//  四个函数加起来大约六十行。重点不是代码量，是每一步的维度和坐标对不对。
// ============================================================================

#ifndef DETECTOR_HPP_
#define DETECTOR_HPP_

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

namespace car_detector
{

/// 一个检测结果
struct Detection
{
  cv::Rect box;        ///< 框，原图像素坐标
  float confidence;    ///< 置信度
  int class_id;        ///< 类别 id（本模型只有一类：car，id=0）
};

/// letterbox 变换的参数，把框从网络输入坐标还原回原图时要用
struct LetterboxInfo
{
  float scale;   ///< 缩放比例：网络输入尺寸 / 原图尺寸
  int pad_x;     ///< 左侧填充的像素数
  int pad_y;     ///< 顶部填充的像素数
};

class Detector
{
public:
  Detector() = default;

  /// 加载 ONNX 模型。已实现
  bool load(const std::string & onnx_path);

  bool loaded() const {return !net_.empty();}

  // -------------------------------------------------------------- 参数
  void setInputSize(int s) {input_size_ = s;}
  void setConfThreshold(float t) {conf_threshold_ = t;}
  void setNmsThreshold(float t) {nms_threshold_ = t;}
  void setMaxDetections(int n) {max_detections_ = n;}

  int inputSize() const {return input_size_;}
  float confThreshold() const {return conf_threshold_;}
  float nmsThreshold() const {return nms_threshold_;}
  int maxDetections() const {return max_detections_;}

  /**
   * @brief 跑一遍完整检测流程。已实现，它按顺序调用你写的四个函数
   *
   * 流程：letterbox -> preprocess -> net.forward -> decode -> nms
   */
  std::vector<Detection> detect(const cv::Mat & image);

  /// 最近一次推理耗时（毫秒）
  double lastInferenceMs() const {return last_inference_ms_;}

  // -------------------------------------------------------------- 要你写
  /**
   * @brief 等比缩放 + 填充，把任意尺寸的图变成 input_size_ x input_size_
   *
   * 直接 resize 会把图拉变形，检测框也会跟着歪，所以要保持长宽比：
   * 先按长边等比缩放，短边两侧用灰色 (114,114,114) 填充。
   *
   * @param[in]  src  原图
   * @param[out] dst  输出的方形图
   * @param[out] info 填好 scale / pad_x / pad_y，后面还原坐标要用
   */
  void letterbox(const cv::Mat & src, cv::Mat & dst, LetterboxInfo & info) const;

  /// 把 letterbox 后的图转成网络输入 blob
  cv::Mat preprocess(const cv::Mat & padded) const;

  /**
   * @brief 解析网络输出，取出所有置信度达标的候选框
   *
   * @param output 网络原始输出
   * @param info   letterbox 参数，用来把坐标还原回原图
   * @return 候选框（还没做 NMS，可能互相重叠）
   */
  std::vector<Detection> decode(const cv::Mat & output, const LetterboxInfo & info) const;

  /// 非极大值抑制：同类框重叠超过 nms_threshold_ 的，只保留置信度最高的
  std::vector<Detection> nms(const std::vector<Detection> & candidates) const;

private:
  cv::dnn::Net net_;

  int input_size_{640};
  float conf_threshold_{0.25f};
  float nms_threshold_{0.45f};
  int max_detections_{20};

  double last_inference_ms_{0.0};
};

}  // namespace car_detector

#endif  // DETECTOR_HPP_
