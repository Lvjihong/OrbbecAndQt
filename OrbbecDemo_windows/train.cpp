#include "Train.h"
#include <qdatetime.h>
#include <iostream>
#include <vector>

Train::Train(const QString rootDirPath, QWidget* parent) : QWidget(parent) {
  ui.setupUi(this);
  Train::rootDirPath = rootDirPath;

  // ����TreeView
  delete ui.treeView->model();
  updateTreeView();

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

  connect(ui.btn_start, &QPushButton::clicked, [=]() {
    ui.btn_start->setEnabled(false);
    QString subDirPath =
        Train::rootDirPath + "/pig_" +
        QString::number(QDateTime::currentDateTime().toTime_t());
    QDir dir(subDirPath);
    if (!dir.exists()) {
      dir.mkdir(subDirPath);
    }
    if (!dir.exists("Depth")) {
      dir.mkdir("Depth");
    }
    if (!dir.exists("Rgb")) {
      dir.mkdir("Rgb");
    }

    depthCount = 0;
    colorCount = 0;
    isSave = true;
    saveOrShowAll(isSave, subDirPath);
  });

  connect(ui.btn_exit, &QPushButton::clicked, [=]() {
    pipe.stop();
    close();
  });
}
Train::Train(const Train& trainWindow) {
  ui = trainWindow.ui;
  // pipe = trainWindow.pipe;
  // config = std::make_shared<ob::Config>(*trainWindow.config);
  depthCount = trainWindow.depthCount;
  colorCount = trainWindow.colorCount;
  rootDirPath = trainWindow.rootDirPath;
  isSave = trainWindow.isSave;
  // model = new QDirModel(*trainWindow.model);
}
Train::~Train() {
  if (model != nullptr) {
    delete model;
    model = nullptr;
  }
}
QImage Train::mat2QImage(cv::Mat cvImg) {
  QImage qImg;
  if (cvImg.channels() == 3)  // 3 channels color image
  {
    cv::cvtColor(cvImg, cvImg, cv::COLOR_BGR2RGB);
    qImg = QImage((const unsigned char*)(cvImg.data), cvImg.cols, cvImg.rows,
                  cvImg.cols * cvImg.channels(), QImage::Format_RGB888);
  } else if (cvImg.channels() == 1)  // grayscale image
  {
    qImg = QImage((const unsigned char*)(cvImg.data), cvImg.cols, cvImg.rows,
                  cvImg.cols * cvImg.channels(), QImage::Format_Indexed8);
  } else {
    qImg = QImage((const unsigned char*)(cvImg.data), cvImg.cols, cvImg.rows,
                  cvImg.cols * cvImg.channels(), QImage::Format_RGB888);
  }
  return qImg;
}

std::vector<cv::Mat> Train::frame2Mat(
  const std::shared_ptr<ob::VideoFrame>& frame) {
  const int data_size = static_cast<int>(frame->dataSize());
  if (frame == nullptr || data_size < 1024) {
    return {};
  }
  const OBFrameType frame_type = frame->type();  // ֡���ͣ���ɫ/���/���⣩
  const OBFormat frame_format = frame->format();               // ͼ���ʽ
  const int frame_height = static_cast<int>(frame->height());  // ͼ��߶�
  const int frame_width = static_cast<int>(frame->width());    // ͼ����
  void* const frame_data = frame->data();  // ֡ԭʼ����
  cv::Mat result_mat;
  std::vector<cv::Mat> ret;
  if (frame_type == OB_FRAME_COLOR) {
    // Color image
    if (frame_format == OB_FORMAT_MJPG) {
      const cv::Mat raw_mat(1, data_size, CV_8UC1, frame_data);
      result_mat = cv::imdecode(raw_mat, 1);
    } else if (frame_format == OB_FORMAT_NV21) {
      const cv::Mat raw_mat(frame_height * 3 / 2, frame_width, CV_8UC1,
                            frame_data);
      cv::cvtColor(raw_mat, result_mat, cv::COLOR_YUV2BGR_NV21);
    } else if (frame_format == OB_FORMAT_YUYV ||
               frame_format == OB_FORMAT_YUY2) {
      const cv::Mat raw_mat(frame_height, frame_width, CV_8UC2, frame_data);
      cv::cvtColor(raw_mat, result_mat, cv::COLOR_YUV2BGR_YUY2);
    } else if (frame_format == OB_FORMAT_RGB888) {
      const cv::Mat raw_mat(frame_height, frame_width, CV_8UC3, frame_data);
      cv::cvtColor(raw_mat, result_mat, cv::COLOR_RGB2BGR);
    } else if (frame_format == OB_FORMAT_UYVY) {
      const cv::Mat raw_mat(frame_height, frame_width, CV_8UC2, frame_data);
      cv::cvtColor(raw_mat, result_mat, cv::COLOR_YUV2BGR_UYVY);
    }
    ret.push_back(result_mat);
  } else if (frame_format == OB_FORMAT_Y16 || frame_format == OB_FORMAT_YUYV ||
             frame_format == OB_FORMAT_YUY2) {
    // IR or depth image
    const cv::Mat raw_mat(frame_height, frame_width, CV_16UC1, frame_data);
    const double scale =
        1 / pow(2, frame->pixelAvailableBitSize() -
                       (frame_type == OB_FRAME_DEPTH ? 10 : 8));
    //result_mat = raw_mat;
    cv::convertScaleAbs(raw_mat, result_mat, scale);
    ret.push_back(result_mat);
    ret.push_back(raw_mat);
  } else if (frame_type == OB_FRAME_IR) {
    // IR image
    if (frame_format == OB_FORMAT_Y8) {
      result_mat = cv::Mat(frame_height, frame_width, CV_8UC1, frame_data);
    } else if (frame_format == OB_FORMAT_MJPG) {
      const cv::Mat raw_mat(1, data_size, CV_8UC1, frame_data);
      result_mat = cv::imdecode(raw_mat, 1);
    }
  }
  return ret;
}

// Save the depth map in png format
void saveDepthPng(std::shared_ptr<ob::DepthFrame> depthFrame,
                  int index,
                  std::string dirPath) {
  std::vector<int> compression_params;
  compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(0);
  compression_params.push_back(cv::IMWRITE_PNG_STRATEGY);
  compression_params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
  std::string depthName =
      dirPath + "\\depth\\Depth_" + std::to_string(depthFrame->width()) + "x" +
      std::to_string(depthFrame->height()) + "_" + std::to_string(index) + "_" +
      std::to_string(depthFrame->timeStamp()) + "ms.png";
  cv::Mat depthMat = Train::frame2Mat(depthFrame).at(1);
  cv::imwrite(depthName, depthMat, compression_params);
}

// Save the color image in png format
void saveColorPng(std::shared_ptr<ob::ColorFrame> colorFrame,
                  int index,
                  std::string dirPath) {
  std::vector<int> compression_params;
  compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(0);
  compression_params.push_back(cv::IMWRITE_PNG_STRATEGY);
  compression_params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
  std::string colorName =
      dirPath + "\\rgb\\Color_" + std::to_string(colorFrame->width()) + "x" +
      std::to_string(colorFrame->height()) + "_" + std::to_string(index) + "_" +
      std::to_string(colorFrame->timeStamp()) + "ms.png";
  cv::Mat colorRawMat(colorFrame->height(), colorFrame->width(), CV_8UC3,
                      colorFrame->data());
  cv::imwrite(colorName, colorRawMat, compression_params);
}

void Train::saveOrShowAll(bool flag, QString dataPathDir) {
  try {
    pipe.stop();
    pipe.enableFrameSync();
    pipe.start(config);
    dataPathDir.replace("/", "\\");
    while (cv::waitKey() != 27 && this->isVisible()) {
      // �������ķ�ʽ�ȴ�һ֡���ݣ���֡��һ������֡�����������õ���������֡���ݶ��������frameSet�ڣ�
      // ������֡�ĵȴ���ʱʱ��Ϊ100ms
      auto frame_set = pipe.waitForFrames(100);
      if (frame_set == nullptr) {
        continue;
      }
      ob::FormatConvertFilter formatConvertFilter;
      cv::Mat imgDepth;
      cv::Mat imgRgb;
      if (!frame_set->depthFrame()) {
        continue;
      }
      if (!frame_set->colorFrame()) {
        continue;
      }
      cv::Mat depthMat1;
      // �ɻ�ɫͼ��תΪα��ɫͼ��
      imgDepth = frame2Mat(frame_set->depthFrame()).at(0);
      cv::applyColorMap(imgDepth, depthMat1, cv::COLORMAP_JET);
      if (depthCount < 30 && flag) {
        std::thread t1([=]() mutable {
          saveDepthPng(frame_set->depthFrame(), depthCount,
                       dataPathDir.toStdString());
        });
        t1.detach();
        depthCount++;
      } else {
      }
      ui.label_depth->setPixmap(
          QPixmap::fromImage(mat2QImage(depthMat1)));

      auto colorFrame = frame_set->colorFrame();
      imgRgb = frame2Mat(colorFrame).at(0);
      
      if (colorCount < 30 && flag) {
        std::thread t2([=]() mutable {
          if (colorFrame->format() != OB_FORMAT_RGB) {
            if (colorFrame->format() == OB_FORMAT_MJPG) {
              formatConvertFilter.setFormatConvertType(FORMAT_MJPG_TO_RGB888);
            } else if (colorFrame->format() == OB_FORMAT_UYVY) {
              formatConvertFilter.setFormatConvertType(FORMAT_UYVY_TO_RGB888);
            } else if (colorFrame->format() == OB_FORMAT_YUYV) {
              formatConvertFilter.setFormatConvertType(FORMAT_YUYV_TO_RGB888);
            } else {
              std::cout << "Color format is not support!" << std::endl;
            }
            colorFrame =
                formatConvertFilter.process(colorFrame)->as<ob::ColorFrame>();
          }
          formatConvertFilter.setFormatConvertType(FORMAT_RGB888_TO_BGR);
          colorFrame =
              formatConvertFilter.process(colorFrame)->as<ob::ColorFrame>();
          saveColorPng(colorFrame, colorCount, dataPathDir.toStdString());
        });
        t2.detach();
        colorCount++;
      } else if (colorCount > 29 && flag) {
        flag = false;
        ui.btn_start->setEnabled(true);

        InputWeightDialog* dialog = new InputWeightDialog(dataPathDir, this);
        dialog->setWindowFlags(Qt::Window);
        dialog->setWindowModality(Qt::ApplicationModal);
        dialog->setWindowFlags((windowFlags() & ~Qt::WindowCloseButtonHint));
        connect(dialog, &InputWeightDialog::inputOver, this,
                &Train::updateTreeView);
        dialog->show();

        break;
      }
      ui.label_rgb->setPixmap(QPixmap::fromImage(mat2QImage(imgRgb)));
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

void Train::updateTreeView() {
  model = new QDirModel;
  model->setReadOnly(false);
  model->setSorting(QDir::DirsFirst | QDir::IgnoreCase | QDir::Name);

  ui.treeView->setModel(model);
  ui.treeView->header()->setStretchLastSection(true);
  ui.treeView->header()->setSortIndicator(0, Qt::AscendingOrder);
  ui.treeView->header()->setSortIndicatorShown(true);

  QModelIndex index = model->index(rootDirPath);
  ui.treeView->setRootIndex(index);
  ui.treeView->expand(index);
  ui.treeView->scrollTo(index);
  ui.treeView->resizeColumnToContents(0);
  ui.treeView->allColumnsShowFocus();
}

void Train::closeEvent(QCloseEvent* e) {
  pipe.stop();
  e->accept();
}

