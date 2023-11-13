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
  connect(ui.btn_select, &QPushButton::clicked, [=]() {
    QString dirpath = QFileDialog::getExistingDirectory(
        this, QString::fromLocal8Bit("��ũר��_ѡ����Ҫ���Ƶ�����"), "./",
        QFileDialog::ShowDirsOnly);

    QDir dir(dirpath);
    if (!dir.exists()) {
      dir.mkdir(dirpath);
    }

    // ����ģ��
    torch::Device device(torch::kCPU);  //����cpu�豸
    std::string img_path = "test.png";  //����ͼƬ·��
    std::string model_path = "mobilenet.pt";  //ģ�ʹ洢��·��
    auto module = torch::jit::load(model_path);
    module.to(
        device);  //ģ�ͼ��ص�cpu�ϣ����ģ�;���һ�������ģ�ͣ�������ʱ��������Ԥ������Ŷȡ�
    auto test_image = cv::imread(img_path, -1);  //��ȡ����ͼƬ
    cv::Mat norm_img;
    cv::normalize(test_image, norm_img, 0, 255,
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
    tensor_image =
        tensor_image.toType(torch::kFloat);    //�ı���������int-->float
    tensor_image = tensor_image.unsqueeze(0);  //��չά��(B,C,H,W)
    tensor_image = tensor_image.to(device);    //��ͼƬҲ���ص�cpu��
    torch::Tensor output0 =
        module.forward({tensor_image}).toTensor();  //ǰ�򴫲�Ԥ����
    torch::Tensor output = torch::softmax(output0[0], -1);  // softmax����
    torch::Tensor result =
        std::get<0>(torch::max(output, -1));  //����һ��Ԥ�����ֵ�����ֵ

    std::vector<float> res(
        result.data_ptr<float>(),
        result.data_ptr<float>() + result.numel());  // tensor���ͱ�Ϊvector����
    //���float���͵�ֵ
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
      // �������ķ�ʽ�ȴ�һ֡���ݣ���֡��һ������֡�����������õ���������֡���ݶ��������frameSet�ڣ�
      // ������֡�ĵȴ���ʱʱ��Ϊ100ms
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
      // �ɻ�ɫͼ��תΪα��ɫͼ��
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
