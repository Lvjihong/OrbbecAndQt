#include <iostream>
#include <Windows.h>
#include "WeightEstmation.h"

#undef slots
#include <torch/script.h>
#include <torch/torch.h>
#define slots Q_SLOTS
#include <math.h>
#include <time.h>

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
  // 设置深度图和RGB图像对齐
  config->setAlignMode(ALIGN_D2C_HW_MODE);
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

  // 加载实例分割模型
  std::string seg_model_path = "./models/best.onnx";
  if (task_segment.ReadModel(net, seg_model_path, true)) {
    std::cout << "read net ok!" << std::endl;
  }

  connect(ui.btn_select, &QPushButton::clicked, [=]() {
    pipe.stop();
    QString rgb_file_path = QFileDialog::getOpenFileName(
        this, QString::fromLocal8Bit("大北农专项_选择需要估计的彩色图像"), "./",
        tr("images(*.png *jpeg *jpg)"));
    if (rgb_file_path.isEmpty()) return;
    cv::Mat img_rgb = cv::imread(rgb_file_path.toStdString(), -1);

    QString depth_file_path = QFileDialog::getOpenFileName(
        this, QString::fromLocal8Bit("大北农专项_选择需要对应的深度图像"), "./",
        tr("images(*.png *jpeg *jpg)"));
    if (depth_file_path.isEmpty()) return;
    cv::Mat img_depth = cv::imread(depth_file_path.toStdString(), -1);

    test_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();



    estimate_weight(img_rgb, img_depth);
  });

  connect(ui.btn_exit, &QPushButton::clicked, [=]() {
    pipe.stop();
    close();
  });

  connect(ui.btn_start, &QPushButton::clicked, [=]() { showDepth(); });

  connect(this, &WeightEstmation::show_img,this,&WeightEstmation::showImg);
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
      // 以阻塞的方式等待一帧数据，该帧是一个复合帧，
      // 配置里启用的所有流的帧数据都会包含在frameSet内，
      // 并设置帧的等待超时时间为100ms
      auto frame_set = pipe.waitForFrames(200);
      if (frame_set == nullptr) {
        continue;
      }
      cv::Mat img_color;
      if (!frame_set->colorFrame()) continue;
      if (!frame_set->depthFrame()) continue;
      //segment(Train::frame2Mat(frame_set->colorFrame())[0]);

    //QMessageBox msgBox;
    //test_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - test_time;
    //msgBox.setText(QString::fromStdString(std::to_string(Train::frame2Mat(frame_set->depthFrame())[1].depth())));
    //msgBox.exec();

      segment(Train::frame2Mat(frame_set->colorFrame())[0],
          Train::frame2Mat(frame_set->depthFrame())[1]);

      // cv::Mat imgDepth;
      // if (!frame_set->depthFrame()) {
      //  continue;
      //}
      // cv::Mat depthMat1;
      //// 由灰色图像转为伪彩色图像
      // imgDepth = Train::frame2Mat(frame_set->depthFrame())[0];
      // cv::applyColorMap(imgDepth, depthMat1, cv::COLORMAP_JET);
      // ui.label_depth->setPixmap(
      // QPixmap::fromImage(Train::mat2QImage(depthMat1)));
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

void WeightEstmation::estimate_weight(cv::Mat seg_depth_img) {
    // 加载估重模型
  torch::Device device(torch::kCPU);  //定义cpu设备
  // std::string img_path = "test.png";   //测试图片路径
  std::string model_path = "./models/mobilenet.pt";  //模型存储的路径
  auto module = torch::jit::load(model_path);
  module.to(
      device);  //模型加载到cpu上，这个模型就是一个分类的模型，我们暂时先输出类别预测的置信度。

  cv::Mat norm_img;
  cv::normalize(seg_depth_img, norm_img, 0, 255,
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
  tensor_image = tensor_image.toType(torch::kFloat);  //改变数据类型int-->float
  tensor_image = tensor_image.unsqueeze(0);           //扩展维数(B,C,H,W)
  tensor_image = tensor_image.to(device);  //将图片也加载到cpu上
  torch::Tensor output0 =
      module.forward({tensor_image}).toTensor();  //前向传播预测结果
  torch::Tensor output = torch::softmax(output0[0], -1);  // softmax处理
  torch::Tensor result =
      std::get<0>(torch::max(output, -1));  //返回一个预测概率值的最大
  std::vector<float> res(
      result.data_ptr<float>(),
      result.data_ptr<float>() + result.numel());  // tensor类型变为vector类型
  //输出float类型的值
  ui.label_weight->setText(QString::number(res[0] * 100)+"kg");

}


void WeightEstmation::estimate_weight(cv::Mat rgb_img, cv::Mat depth_img) {

   segment(rgb_img);

  // 加载估重模型
  torch::Device device(torch::kCPU);  //定义cpu设备
  // std::string img_path = "test.png";   //测试图片路径
  std::string model_path = "./models/mobilenet.pt";  //模型存储的路径
  auto module = torch::jit::load(model_path);
  module.to(
      device);  //模型加载到cpu上，这个模型就是一个分类的模型，我们暂时先输出类别预测的置信度。
  //auto test_image = cv::imread(fileName.toStdString(), -1);  //读取测试图片
  cv::Mat norm_img;
  cv::normalize(depth_img, norm_img, 0, 255,
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
  tensor_image = tensor_image.toType(torch::kFloat);  //改变数据类型int-->float
  tensor_image = tensor_image.unsqueeze(0);           //扩展维数(B,C,H,W)
  tensor_image = tensor_image.to(device);  //将图片也加载到cpu上
  torch::Tensor output0 =
      module.forward({tensor_image}).toTensor();  //前向传播预测结果
  torch::Tensor output = torch::softmax(output0[0], -1);  // softmax处理
  torch::Tensor result =
      std::get<0>(torch::max(output, -1));  //返回一个预测概率值的最大
  std::vector<float> res(
      result.data_ptr<float>(),
      result.data_ptr<float>() + result.numel());  // tensor类型变为vector类型
  //输出float类型的值
  ui.label_weight->setText(QString::number(res[0] * 100)+"kg");
}

void WeightEstmation::segment(cv::Mat img) {
  cv::Mat copy = img.clone();
  cv::Mat ret;
  std::vector<cv::Scalar> color;

  // 生成随机颜色
  /*srand(time(0));
  for (int i = 0; i < 1; i++) {
          int b = rand() % 256;
          int g = rand() % 256;
          int r = rand() % 256;
          color.push_back(Scalar(b, g, r));
  }*/

  color.push_back(cv::Scalar(0, 0, 255));
  std::vector<OutputSeg> result;  // 输出结果包括外包围框和mask

  if (task_segment.Detect(copy, net, result)) {
  ret = DrawPred(copy, result, task_segment._className,
                   color);  //将掩膜和包围框绘制在图片上
  }
  else {
      ret = img;
  }
  emit(show_img(ret));
}
void WeightEstmation::segment(cv::Mat rgb_img, cv::Mat depth_img) {


  cv::Mat ret;
  std::vector<cv::Scalar> color;

  color.push_back(cv::Scalar(0, 0, 255));
  std::vector<OutputSeg> result;  // 输出结果包括外包围框和mask

  if (task_segment.Detect(rgb_img, net, result)) {

      std::vector<cv::Mat> vec = DrawPred(rgb_img, depth_img, result, task_segment._className,
                   color);  //将掩膜和包围框绘制在图片上
      ret = vec.at(1);

      estimate_weight(vec.at(0));

  }
  else {
      ret = rgb_img;
  }
  emit(show_img(ret));

}
void WeightEstmation::showImg(cv::Mat img) {
        ui.label_depth->setPixmap(QPixmap::fromImage(Train::mat2QImage(img)));
    //QMessageBox msgBox;
    //test_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - test_time;

    //msgBox.setText(QString::fromStdString(std::to_string(test_time) + "ms"));
    //msgBox.exec();

}