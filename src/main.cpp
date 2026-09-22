// ============================================================================
//  主程序：遍历图片 -> 跑检测 -> 画框存图 + 写统计
//  这是脚手架，你不需要修改本文件。
//
//  用法：
//      ./car_detector                      # 用 config/params.yaml 跑 image/ 下所有图
//      ./car_detector --conf 0.4           # 临时改置信度阈值
//      ./car_detector --tag conf04         # 给本次运行起名
//      ./car_detector --image image/03.jpg # 只跑单张，调试用
//      ./car_detector --help               # 看全部选项
//
//  每次运行都会在 result/ 下新建一个带时间戳的目录，不会覆盖上一次的结果：
//      result/20260921_143022_conf04/
//          00.jpg ... 19.jpg    画好检测框的图
//          summary.txt          本次参数 + 每张图的检出数与耗时
// ============================================================================

#include <algorithm>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "detector.hpp"
#include "visualize.hpp"

namespace fs = std::filesystem;
using car_detector::Detector;
using car_detector::Detection;

namespace
{

struct Options
{
  std::string image_dir = "image";
  std::string single_image;
  std::string config_path = "config/params.yaml";
  std::string weights_path;
  std::string result_dir = "result";
  std::string tag;
  double conf = -1;
  double nms = -1;
  int input_size = -1;
  bool quiet = false;
};

void printHelp()
{
  std::cout <<
    "用法: ./car_detector [选项]\n\n"
    "  --config <path>     参数文件，默认 config/params.yaml\n"
    "  --weights <path>    ONNX 模型路径，默认读配置文件\n"
    "  --image-dir <path>  图片目录，默认 image\n"
    "  --image <path>      只跑单张图（调试用）\n"
    "  --result-dir <path> 输出目录，默认 result\n"
    "  --tag <name>        给本次运行起名，出现在输出目录名里\n"
    "  --conf <v>          临时覆盖置信度阈值\n"
    "  --nms <v>           临时覆盖 NMS IoU 阈值\n"
    "  --input-size <n>    临时覆盖模型输入尺寸\n"
    "  --quiet             只输出汇总，不逐张打印\n"
    "  --help              显示本帮助\n";
}

bool parseArgs(int argc, char ** argv, Options & o)
{
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const auto nextNum = [&](double & dst) {
        if (i + 1 >= argc) {return false;}
        dst = std::stod(argv[++i]);
        return true;
      };
    const auto nextStr = [&](std::string & dst) {
        if (i + 1 >= argc) {return false;}
        dst = argv[++i];
        return true;
      };

    if (a == "--help" || a == "-h") {printHelp(); return false;}
    else if (a == "--config") {if (!nextStr(o.config_path)) {return false;}}
    else if (a == "--weights") {if (!nextStr(o.weights_path)) {return false;}}
    else if (a == "--image-dir") {if (!nextStr(o.image_dir)) {return false;}}
    else if (a == "--image") {if (!nextStr(o.single_image)) {return false;}}
    else if (a == "--result-dir") {if (!nextStr(o.result_dir)) {return false;}}
    else if (a == "--tag") {if (!nextStr(o.tag)) {return false;}}
    else if (a == "--conf") {if (!nextNum(o.conf)) {return false;}}
    else if (a == "--nms") {if (!nextNum(o.nms)) {return false;}}
    else if (a == "--input-size") {
      double v = 0;
      if (!nextNum(v)) {return false;}
      o.input_size = static_cast<int>(v);
    } else if (a == "--quiet") {o.quiet = true;}
    else {
      std::cerr << "未知选项: " << a << "\n用 --help 查看可用选项\n";
      return false;
    }
  }
  return true;
}

std::string timestamp()
{
  const std::time_t now = std::time(nullptr);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &now);
#else
  localtime_r(&now, &tm);
#endif
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
  return buf;
}

}  // namespace

int main(int argc, char ** argv)
{
  Options opt;
  if (!parseArgs(argc, argv, opt)) {return 0;}

  // ------------------------------------------------------------ 读配置
  car_detector::Config cfg;
  if (!cfg.load(opt.config_path)) {
    std::cerr << "[error] 读不到参数文件: " << opt.config_path << "\n"
              << "        请在项目根目录下运行，或用 --config 指定路径\n";
    return 1;
  }

  std::string weights = cfg.str("weights_path", "weights/car.onnx");
  int input_size = static_cast<int>(cfg.num("input_size", 640));
  float conf = static_cast<float>(cfg.num("conf_threshold", 0.25));
  float nms = static_cast<float>(cfg.num("nms_threshold", 0.45));
  const int max_det = static_cast<int>(cfg.num("max_detections", 20));

  if (!opt.weights_path.empty()) {weights = opt.weights_path;}
  if (opt.input_size > 0) {input_size = opt.input_size;}
  if (opt.conf >= 0) {conf = static_cast<float>(opt.conf);}
  if (opt.nms >= 0) {nms = static_cast<float>(opt.nms);}

  // ------------------------------------------------------------ 加载模型
  Detector det;
  if (!det.load(weights)) {
    std::cerr << "[error] 模型加载失败: " << weights << "\n"
              << "        确认文件存在，且是 OpenCV dnn 能读的 ONNX\n";
    return 1;
  }
  det.setInputSize(input_size);
  det.setConfThreshold(conf);
  det.setNmsThreshold(nms);
  det.setMaxDetections(max_det);

  if (!opt.quiet) {
    std::cout << "[info] 模型 " << weights << "  输入 " << input_size
              << "  conf " << conf << "  nms " << nms << "\n";
  }

  // ------------------------------------------------------------ 收集图片
  std::vector<fs::path> images;
  if (!opt.single_image.empty()) {
    images.emplace_back(opt.single_image);
  } else {
    std::error_code ec;
    if (!fs::is_directory(opt.image_dir, ec)) {
      std::cerr << "[error] 图片目录不存在: " << opt.image_dir << "\n";
      return 1;
    }
    for (const auto & e : fs::directory_iterator(opt.image_dir)) {
      const std::string ext = e.path().extension().string();
      if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp") {
        images.push_back(e.path());
      }
    }
    std::sort(images.begin(), images.end());
  }
  if (images.empty()) {
    std::cerr << "[error] 没找到图片\n";
    return 1;
  }

  // ------------------------------------------------------------ 建输出目录
  std::string name = timestamp();
  if (!opt.tag.empty()) {name += "_" + opt.tag;}
  const fs::path out = fs::path(opt.result_dir) / name;
  std::error_code ec;
  fs::create_directories(out, ec);
  if (ec) {
    std::cerr << "[error] 建不了输出目录 " << out << ": " << ec.message() << "\n";
    return 1;
  }

  // ------------------------------------------------------------ 逐张检测
  std::ostringstream rep;
  rep << std::fixed;
  rep << "================ 本次运行 ================\n";
  rep << "  weights        : " << weights << "\n";
  rep << "  input_size     : " << input_size << "\n";
  rep << "  conf_threshold : " << std::setprecision(2) << conf << "\n";
  rep << "  nms_threshold  : " << nms << "\n";
  rep << "  max_detections : " << max_det << "\n";
  rep << "  images         : " << images.size() << "\n";
  rep << "------------------------------------------\n";
  rep << "  image                 dets    infer(ms)\n";

  int total_dets = 0;
  double total_ms = 0.0;
  int empty_images = 0;

  for (const auto & p : images) {
    const cv::Mat img = cv::imread(p.string(), cv::IMREAD_COLOR);
    if (img.empty()) {
      std::cerr << "[warn] 读不了 " << p.filename().string() << "，跳过\n";
      continue;
    }

    const std::vector<Detection> dets = det.detect(img);
    total_dets += static_cast<int>(dets.size());
    total_ms += det.lastInferenceMs();
    if (dets.empty()) {++empty_images;}

    std::ostringstream sub;
    sub << "conf>" << std::fixed << std::setprecision(2) << conf
        << "  " << std::setprecision(1) << det.lastInferenceMs() << "ms";
    const cv::Mat vis = car_detector::drawDetections(img, dets, {"car"}, sub.str());
    cv::imwrite((out / p.filename()).string(), vis);

    rep << "  " << std::left << std::setw(22) << p.filename().string()
        << std::right << std::setw(4) << dets.size()
        << std::setw(12) << std::setprecision(1) << det.lastInferenceMs() << "\n";

    if (!opt.quiet) {
      std::cout << "[ ok ] " << std::left << std::setw(12) << p.filename().string()
                << " 检出 " << dets.size() << " 个"
                << "  " << std::fixed << std::setprecision(1)
                << det.lastInferenceMs() << " ms\n";
    }
  }

  rep << "------------------------------------------\n";
  rep << "  total detections : " << total_dets << "\n";
  rep << "  images with none : " << empty_images << " / " << images.size() << "\n";
  if (!images.empty()) {
    rep << "  avg per image    : " << std::setprecision(2)
        << static_cast<double>(total_dets) / static_cast<double>(images.size()) << "\n";
    rep << "  avg inference    : " << std::setprecision(1)
        << total_ms / static_cast<double>(images.size()) << " ms\n";
  }
  rep << "==========================================\n";
  rep << "\n";
  rep << "怎么看这些数：\n";
  rep << "  所有图都检出 0 个 -> 大概率是 decode 没写对（维度/阈值），\n";
  rep << "    先把网络输出的维度打印出来确认。\n";
  rep << "  检出数明显偏多且框重叠 -> NMS 没起作用。\n";
  rep << "  框的位置整体偏移或缩放 -> letterbox 的坐标还原漏了。\n";

  std::cout << "\n" << rep.str();
  std::ofstream(out / "summary.txt") << rep.str();

  std::cout << "[info] 结果已写入 " << out.string() << "/\n"
            << "       画框图 + summary.txt\n";
  return 0;
}
