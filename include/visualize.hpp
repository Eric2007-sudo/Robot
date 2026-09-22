// ============================================================================
//  脚手架：可视化绘制、配置读取
//  这些都已经写好了，你不需要修改本文件。
// ============================================================================

#ifndef VISUALIZE_HPP_
#define VISUALIZE_HPP_

#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "detector.hpp"

namespace car_detector
{

// ---------------------------------------------------------------- 极简配置
/// 极简 key: value 解析器，够读 config/params.yaml 这种平铺结构
class Config
{
public:
  bool load(const std::string & path)
  {
    std::ifstream f(path);
    if (!f) {return false;}
    std::string line;
    while (std::getline(f, line)) {
      const auto hash = line.find('#');
      if (hash != std::string::npos) {line = line.substr(0, hash);}
      const auto colon = line.find(':');
      if (colon == std::string::npos) {continue;}
      std::string key = trim(line.substr(0, colon));
      std::string val = trim(line.substr(colon + 1));
      if (key.empty() || val.empty()) {continue;}
      kv_[key] = val;
    }
    return true;
  }

  double num(const std::string & key, double fallback) const
  {
    auto it = kv_.find(key);
    if (it == kv_.end()) {return fallback;}
    try {return std::stod(it->second);} catch (...) {return fallback;}
  }

  std::string str(const std::string & key, const std::string & fallback) const
  {
    auto it = kv_.find(key);
    if (it == kv_.end()) {return fallback;}
    std::string v = it->second;
    if (v.size() >= 2 && (v.front() == '"' || v.front() == '\'')) {
      v = v.substr(1, v.size() - 2);
    }
    return v;
  }

private:
  static std::string trim(const std::string & s)
  {
    const auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) {return "";}
    const auto e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
  }
  std::map<std::string, std::string> kv_;
};

// ---------------------------------------------------------------- 画检测结果
/// 把检测框、类别、置信度画到图上
inline cv::Mat drawDetections(
  const cv::Mat & image, const std::vector<Detection> & dets,
  const std::vector<std::string> & class_names = {"car"},
  const std::string & subtitle = "")
{
  cv::Mat vis = image.clone();

  // 不同类别用不同颜色（BGR）
  const std::vector<cv::Scalar> colors = {
    {80, 220, 100},    // 绿
    {70, 70, 240},     // 红
    {240, 190, 60},    // 青黄
    {240, 120, 220},   // 粉
  };

  for (const auto & d : dets) {
    const cv::Scalar color = colors[static_cast<std::size_t>(d.class_id) % colors.size()];

    cv::rectangle(vis, d.box, color, 2, cv::LINE_AA);

    const std::string name =
      (d.class_id >= 0 && static_cast<std::size_t>(d.class_id) < class_names.size()) ?
      class_names[static_cast<std::size_t>(d.class_id)] :
      ("cls" + std::to_string(d.class_id));

    std::ostringstream os;
    os.precision(2);
    os << std::fixed << name << " " << d.confidence;
    const std::string label = os.str();

    int base = 0;
    const cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &base);
    // 标签底框，贴在检测框上沿；靠顶时翻到框内侧，避免画到图外
    int ly = d.box.y - ts.height - 6;
    const bool below = ly < 0;
    if (below) {ly = d.box.y + 2;}
    cv::rectangle(
      vis, cv::Rect(d.box.x, ly, ts.width + 8, ts.height + 6), color, -1);
    cv::putText(
      vis, label, cv::Point(d.box.x + 4, ly + ts.height + 1),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(20, 20, 20), 1, cv::LINE_AA);
  }

  // 左上角总览信息
  std::ostringstream head;
  head << "detections: " << dets.size();
  if (!subtitle.empty()) {head << "   " << subtitle;}
  const std::string ht = head.str();
  int hb = 0;
  const cv::Size hs = cv::getTextSize(ht, cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &hb);
  cv::rectangle(vis, cv::Rect(8, 8, hs.width + 14, hs.height + 12), cv::Scalar(30, 30, 30), -1);
  cv::putText(
    vis, ht, cv::Point(15, 8 + hs.height + 4),
    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(230, 230, 230), 1, cv::LINE_AA);

  return vis;
}

}  // namespace car_detector

#endif  // VISUALIZE_HPP_
