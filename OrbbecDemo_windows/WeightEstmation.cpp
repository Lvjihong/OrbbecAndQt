#include "WeightEstmation.h"

WeightEstmation::WeightEstmation(QWidget* parent)
    : QWidget(parent) {
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
      ui.label_depth->setPixmap(QPixmap::fromImage(Train::mat2QImage(depthMat1)));
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
