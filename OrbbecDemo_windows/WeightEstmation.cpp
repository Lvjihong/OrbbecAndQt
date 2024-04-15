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

  // ��ȡRGB��������������ã��������ķֱ��ʣ�֡�ʣ��Լ�֡�ĸ�ʽ
  auto colorProfiles = pipe.getStreamProfileList(OB_SENSOR_COLOR);
  std::shared_ptr<ob::VideoStreamProfile> profile = nullptr;
  try {
    profile = colorProfiles->getVideoStreamProfile(1280, 0, OB_FORMAT_RGB, 30);
  } catch (const ob::Error&) {
    profile =
        std::const_pointer_cast<ob::StreamProfile>(colorProfiles->getProfile(0))
            ->as<ob::VideoStreamProfile>();
  }
  // ͨ������Config������PipelineҪ���û��߽�����Щ�������ｫ����RGB��
  config->enableStream(profile);
  // ��ȡ�����������������ã��������ķֱ��ʣ�֡�ʣ��Լ�֡�ĸ�ʽ
  auto depthProfiles = pipe.getStreamProfileList(OB_SENSOR_DEPTH);
  try {
    // ����ָ���ĸ�ʽ���Ҷ�Ӧ��Profile,���Ȳ���Y16��ʽ
    profile = depthProfiles->getVideoStreamProfile(1920, 0, OB_FORMAT_Y16, 30);
  } catch (const ob::Error&) {
    // û�ҵ�Y16��ʽ��ƥ���ʽ���Ҷ�Ӧ��Profile���п���
    profile =
        std::const_pointer_cast<ob::StreamProfile>(depthProfiles->getProfile(0))
            ->as<ob::VideoStreamProfile>();
  }
  // �������ͼ��RGBͼ�����
  config->setAlignMode(ALIGN_D2C_HW_MODE);
  // ͨ������Config������PipelineҪ���û��߽�����Щ�������ｫ���������
  config->enableStream(profile);
  // �ر�����ľ���ģʽ�����ж��豸�Ƿ��пɶ���д��Ȩ�ޣ��ٽ�������
  const auto& device = pipe.getDevice();
  if (device->isPropertySupported(OB_PROP_DEPTH_MIRROR_BOOL,
                                  OB_PERMISSION_WRITE)) {
    device->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, false);
  }
  if (device->isPropertySupported(OB_PROP_COLOR_MIRROR_BOOL,
                                  OB_PERMISSION_WRITE)) {
    device->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, false);
  }

  // ����ʵ���ָ�ģ��
  std::string seg_model_path = "./models/best.onnx";
  if (task_segment.ReadModel(net, seg_model_path, true)) {
    std::cout << "read net ok!" << std::endl;
  }

  connect(ui.btn_select, &QPushButton::clicked, [=]() {
    pipe.stop();
    QString rgb_file_path = QFileDialog::getOpenFileName(
        this, QString::fromLocal8Bit("��ũר��_ѡ����Ҫ���ƵĲ�ɫͼ��"), "./",
        tr("images(*.png *jpeg *jpg)"));
    if (rgb_file_path.isEmpty()) return;
    cv::Mat img_rgb = cv::imread(rgb_file_path.toStdString(), -1);

    QString depth_file_path = QFileDialog::getOpenFileName(
        this, QString::fromLocal8Bit("��ũר��_ѡ����Ҫ��Ӧ�����ͼ��"), "./",
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
      // �������ķ�ʽ�ȴ�һ֡���ݣ���֡��һ������֡��
      // ���������õ���������֡���ݶ��������frameSet�ڣ�
      // ������֡�ĵȴ���ʱʱ��Ϊ100ms
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
      //// �ɻ�ɫͼ��תΪα��ɫͼ��
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
    // ���ع���ģ��
  torch::Device device(torch::kCPU);  //����cpu�豸
  // std::string img_path = "test.png";   //����ͼƬ·��
  std::string model_path = "./models/mobilenet.pt";  //ģ�ʹ洢��·��
  auto module = torch::jit::load(model_path);
  module.to(
      device);  //ģ�ͼ��ص�cpu�ϣ����ģ�;���һ�������ģ�ͣ�������ʱ��������Ԥ������Ŷȡ�

  cv::Mat norm_img;
  cv::normalize(seg_depth_img, norm_img, 0, 255,
                cv::NORM_MINMAX);  //��ͼƬ��һ��
  cv::Mat input_img;
  cv::cvtColor(norm_img, input_img, cv::COLOR_GRAY2RGB);  //���RGB��ͨ��
  cv::Mat trans_img;
  cv::resize(input_img, trans_img,
             cv::Size(448, 448));  //ģ�������С�̶�Ϊ488*488
  torch::Tensor tensor_image =
      torch::from_blob(trans_img.data, {trans_img.rows, trans_img.cols, 3},
                       torch::kByte);              //ͼƬ��Ϊtensor����
  tensor_image = tensor_image.permute({2, 0, 1});  //ת�ã�H,W,C��-->(C,H,W)
  tensor_image = tensor_image.toType(torch::kFloat);  //�ı���������int-->float
  tensor_image = tensor_image.unsqueeze(0);           //��չά��(B,C,H,W)
  tensor_image = tensor_image.to(device);  //��ͼƬҲ���ص�cpu��
  torch::Tensor output0 =
      module.forward({tensor_image}).toTensor();  //ǰ�򴫲�Ԥ����
  torch::Tensor output = torch::softmax(output0[0], -1);  // softmax����
  torch::Tensor result =
      std::get<0>(torch::max(output, -1));  //����һ��Ԥ�����ֵ�����
  std::vector<float> res(
      result.data_ptr<float>(),
      result.data_ptr<float>() + result.numel());  // tensor���ͱ�Ϊvector����
  //���float���͵�ֵ
  ui.label_weight->setText(QString::number(res[0] * 100)+"kg");

}


void WeightEstmation::estimate_weight(cv::Mat rgb_img, cv::Mat depth_img) {

   segment(rgb_img);

  // ���ع���ģ��
  torch::Device device(torch::kCPU);  //����cpu�豸
  // std::string img_path = "test.png";   //����ͼƬ·��
  std::string model_path = "./models/mobilenet.pt";  //ģ�ʹ洢��·��
  auto module = torch::jit::load(model_path);
  module.to(
      device);  //ģ�ͼ��ص�cpu�ϣ����ģ�;���һ�������ģ�ͣ�������ʱ��������Ԥ������Ŷȡ�
  //auto test_image = cv::imread(fileName.toStdString(), -1);  //��ȡ����ͼƬ
  cv::Mat norm_img;
  cv::normalize(depth_img, norm_img, 0, 255,
                cv::NORM_MINMAX);  //��ͼƬ��һ��
  cv::Mat input_img;
  cv::cvtColor(norm_img, input_img, cv::COLOR_GRAY2RGB);  //���RGB��ͨ��
  cv::Mat trans_img;
  cv::resize(input_img, trans_img,
             cv::Size(448, 448));  //ģ�������С�̶�Ϊ488*488
  torch::Tensor tensor_image =
      torch::from_blob(trans_img.data, {trans_img.rows, trans_img.cols, 3},
                       torch::kByte);              //ͼƬ��Ϊtensor����
  tensor_image = tensor_image.permute({2, 0, 1});  //ת�ã�H,W,C��-->(C,H,W)
  tensor_image = tensor_image.toType(torch::kFloat);  //�ı���������int-->float
  tensor_image = tensor_image.unsqueeze(0);           //��չά��(B,C,H,W)
  tensor_image = tensor_image.to(device);  //��ͼƬҲ���ص�cpu��
  torch::Tensor output0 =
      module.forward({tensor_image}).toTensor();  //ǰ�򴫲�Ԥ����
  torch::Tensor output = torch::softmax(output0[0], -1);  // softmax����
  torch::Tensor result =
      std::get<0>(torch::max(output, -1));  //����һ��Ԥ�����ֵ�����
  std::vector<float> res(
      result.data_ptr<float>(),
      result.data_ptr<float>() + result.numel());  // tensor���ͱ�Ϊvector����
  //���float���͵�ֵ
  ui.label_weight->setText(QString::number(res[0] * 100)+"kg");
}

void WeightEstmation::segment(cv::Mat img) {
  cv::Mat copy = img.clone();
  cv::Mat ret;
  std::vector<cv::Scalar> color;

  // ���������ɫ
  /*srand(time(0));
  for (int i = 0; i < 1; i++) {
          int b = rand() % 256;
          int g = rand() % 256;
          int r = rand() % 256;
          color.push_back(Scalar(b, g, r));
  }*/

  color.push_back(cv::Scalar(0, 0, 255));
  std::vector<OutputSeg> result;  // �������������Χ���mask

  if (task_segment.Detect(copy, net, result)) {
  ret = DrawPred(copy, result, task_segment._className,
                   color);  //����Ĥ�Ͱ�Χ�������ͼƬ��
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
  std::vector<OutputSeg> result;  // �������������Χ���mask

  if (task_segment.Detect(rgb_img, net, result)) {

      std::vector<cv::Mat> vec = DrawPred(rgb_img, depth_img, result, task_segment._className,
                   color);  //����Ĥ�Ͱ�Χ�������ͼƬ��
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