#include "WeightEstmation.h"
#undef slots
#include <torch/script.h>
#include <torch/torch.h>
#define slots Q_SLOTS
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<vector>
WeightEstmation::WeightEstmation(QWidget* parent) : QWidget(parent) {
  ui.setupUi(this);

  // 获取RGB相机的所有流配置，包括流的分辨率，帧率，以及帧的格式
  auto colorProfiles = pipe.getStreamProfileList(OB_SENSOR_COLOR);
  std::shared_ptr<ob::VideoStreamProfile> profile = nullptr;
  try {
    profile = colorProfiles->getVideoStreamProfile(1280, 0, OB_FORMAT_RGB, 30);
  } catch (const ob::Error&) {
    profile =
        std::const_pointer_cast<ob::StreamProfile>(colorProfiles->getProfile(0))
            ->as<ob::VideoStreamProfile>();
  }
  // 通过创建Config来配置Pipeline要启用或者禁用哪些流，这里将启用RGB流
  config->enableStream(profile);
  // 获取深度相机的所有流配置，包括流的分辨率，帧率，以及帧的格式
  auto depthProfiles = pipe.getStreamProfileList(OB_SENSOR_DEPTH);
  try {
    // 根据指定的格式查找对应的Profile,优先查找Y16格式
    profile = depthProfiles->getVideoStreamProfile(1920, 0, OB_FORMAT_Y16, 30);
  } catch (const ob::Error&) {
    // 没找到Y16格式后不匹配格式查找对应的Profile进行开流
    profile =
        std::const_pointer_cast<ob::StreamProfile>(depthProfiles->getProfile(0))
            ->as<ob::VideoStreamProfile>();
  }
  // 通过创建Config来配置Pipeline要启用或者禁用哪些流，这里将启用深度流
  config->enableStream(profile);
  // 关闭相机的镜像模式，先判断设备是否有可读可写的权限，再进行设置
  const auto& device = pipe.getDevice();
  if (device->isPropertySupported(OB_PROP_DEPTH_MIRROR_BOOL,
                                  OB_PERMISSION_WRITE)) {
    device->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, false);
  }
  if (device->isPropertySupported(OB_PROP_COLOR_MIRROR_BOOL,
                                  OB_PERMISSION_WRITE)) {
    device->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, false);
  }
  connect(ui.btn_select, &QPushButton::clicked, [=]() {
    QString dirpath = QFileDialog::getExistingDirectory(
        this, QString::fromLocal8Bit("大北农专项_选择需要估计的数据"), "./",
        QFileDialog::ShowDirsOnly);

    QDir dir(dirpath);
    if (!dir.exists()) {
      dir.mkdir(dirpath);
    }

    // 加载模型
    torch::Device device(torch::kCPU);  //定义cpu设备
    std::string img_path = "test.png";  //测试图片路径
    std::string model_path = "mobilenet.pt";  //模型存储的路径
    auto module = torch::jit::load(model_path);
    module.to(
        device);  //模型加载到cpu上，这个模型就是一个分类的模型，我们暂时先输出类别预测的置信度。
    auto test_image = cv::imread(img_path, -1);  //读取测试图片
    cv::Mat norm_img;
    cv::normalize(test_image, norm_img, 0, 255,
                  cv::NORM_MINMAX);  //将图片归一化
    cv::Mat input_img;
    cv::cvtColor(norm_img, input_img, cv::COLOR_GRAY2RGB);  //变成RGB三通道
    cv::Mat trans_img;
    cv::resize(input_img, trans_img,
               cv::Size(448, 448));  //模型输入大小固定为488*488
    torch::Tensor tensor_image =
        torch::from_blob(trans_img.data, {trans_img.rows, trans_img.cols, 3},
                         torch::kByte);              //图片变为tensor类型
    tensor_image = tensor_image.permute({2, 0, 1});  //转置（H,W,C）-->(C,H,W)
    tensor_image =
        tensor_image.toType(torch::kFloat);    //改变数据类型int-->float
    tensor_image = tensor_image.unsqueeze(0);  //扩展维数(B,C,H,W)
    tensor_image = tensor_image.to(device);    //将图片也加载到cpu上
    torch::Tensor output0 =
        module.forward({tensor_image}).toTensor();  //前向传播预测结果
    torch::Tensor output = torch::softmax(output0[0], -1);  // softmax处理
    torch::Tensor result =
        std::get<0>(torch::max(output, -1));  //返回一个预测概率值的最大值

    std::vector<float> res(
        result.data_ptr<float>(),
        result.data_ptr<float>() + result.numel());  // tensor类型变为vector类型
    //输出float类型的值
    ui.label_weight->setText(QString::number(res[0] * 100));
  });
  connect(ui.btn_exit, &QPushButton::clicked, [=]() {
    pipe.stop();
    close();
  });
  connect(ui.btn_start, &QPushButton::clicked, [=]() {

  });
}

WeightEstmation::WeightEstmation(const WeightEstmation& window) {
  ui = window.ui;
}

WeightEstmation::~WeightEstmation() {}

void WeightEstmation::showDepth() {
  try {
    pipe.stop();
    pipe.enableFrameSync();
    pipe.start(config);
    while (cv::waitKey() != 27 && this->isVisible()) {
      // 以阻塞的方式等待一帧数据，该帧是一个复合帧，配置里启用的所有流的帧数据都会包含在frameSet内，
      // 并设置帧的等待超时时间为100ms
      auto frame_set = pipe.waitForFrames(100);
      if (frame_set == nullptr) {
        continue;
      }
      ob::FormatConvertFilter formatConvertFilter;
      cv::Mat imgDepth;
      if (!frame_set->depthFrame()) {
        continue;
      }
      cv::Mat depthMat1;
      // 由灰色图像转为伪彩色图像
      imgDepth = Train::frame2Mat(frame_set->depthFrame())[0];
      cv::applyColorMap(imgDepth, depthMat1, cv::COLORMAP_JET);
      ui.label_depth->setPixmap(
          QPixmap::fromImage(Train::mat2QImage(depthMat1)));
    }
  } catch (const ob::Error& e) {
    pipe.stop();
    std::cerr << "Function:" << e.getName() << "\nargs:" << e.getArgs()
              << "\nmessage:" << e.getMessage()
              << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
  } catch (const std::exception& e) {
    pipe.stop();
    std::cerr << e.what() << std::endl;
    exit(EXIT_FAILURE);
  } catch (...) {
    pipe.stop();
    std::cerr << "Unexpected Error!" << std::endl;
    exit(EXIT_FAILURE);
  }
}
