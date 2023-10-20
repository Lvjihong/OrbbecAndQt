#include "MainWindow.h"

#include <iostream>
#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Frame.hpp>
#include "libobsensor/hpp/Error.hpp"
#include "libobsensor/hpp/StreamProfile.hpp"

OrbbecDemo::OrbbecDemo(QWidget* parent) : QMainWindow(parent) {
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
  // 打开相机的镜像模式，先判断设备是否有可读可写的权限，再进行设置
  const auto& device = pipe.getDevice();
  if (device->isPropertySupported(OB_PROP_DEPTH_MIRROR_BOOL,
                                  OB_PERMISSION_WRITE)) {
    device->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, true);
  }
  if (device->isPropertySupported(OB_PROP_COLOR_MIRROR_BOOL,
                                  OB_PERMISSION_WRITE)) {
    device->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, true);
  }
  connect(ui.btn_depth, &QPushButton::clicked, [=]() {
    pipe.stop();
    showDepth();
  });
  connect(ui.btn_color, &QPushButton::clicked, [=]() {
    pipe.stop();
    showRGB();
  });
  connect(ui.btn_save_depth_png, &QPushButton::clicked,
          [=]() { startDepth(); });
  connect(ui.btn_stop_depth, &QPushButton::clicked, [=]() { stopDepth(); });
  connect(ui.btn_save_color_png, &QPushButton::clicked, [=]() { startRGB(); });
  connect(ui.btn_stop_color, &QPushButton::clicked, [=]() { stopRGB(); });
  connect(ui.btn_save_color_avi, &QPushButton::clicked,
          [=]() { saveOrShowAll(); });
}

OrbbecDemo::~OrbbecDemo() {
  pipe.stop();
}

cv::Mat frame2Mat(const std::shared_ptr<ob::VideoFrame>& frame) {
  const int data_size = static_cast<int>(frame->dataSize());
  if (frame == nullptr || data_size < 1024) {
    return {};
  }
  const OBFrameType frame_type = frame->type();  // 帧类型（彩色/深度/红外）
  const OBFormat frame_format = frame->format();               // 图像格式
  const int frame_height = static_cast<int>(frame->height());  // 图像高度
  const int frame_width = static_cast<int>(frame->width());    // 图像宽度
  void* const frame_data = frame->data();  // 帧原始数据
  cv::Mat result_mat;
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
  } else if (frame_format == OB_FORMAT_Y16 || frame_format == OB_FORMAT_YUYV ||
             frame_format == OB_FORMAT_YUY2) {
    // IR or depth image
    const cv::Mat raw_mat(frame_height, frame_width, CV_16UC1, frame_data);
    const double scale =
        1 / pow(2, frame->pixelAvailableBitSize() -
                       (frame_type == OB_FRAME_DEPTH ? 10 : 8));
    cv::convertScaleAbs(raw_mat, result_mat, scale);
  } else if (frame_type == OB_FRAME_IR) {
    // IR image
    if (frame_format == OB_FORMAT_Y8) {
      result_mat = cv::Mat(frame_height, frame_width, CV_8UC1, frame_data);
    } else if (frame_format == OB_FORMAT_MJPG) {
      const cv::Mat raw_mat(1, data_size, CV_8UC1, frame_data);
      result_mat = cv::imdecode(raw_mat, 1);
    }
  }
  return result_mat;
}

QImage mat2QImage(cv::Mat cvImg) {
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

// Save the depth map in png format
void saveDepthPng(std::shared_ptr<ob::DepthFrame> depthFrame, int index) {
  if (index < 30) {
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
    std::string depthName =
        "F://imgs/depth/Depth_" + std::to_string(depthFrame->width()) + "x" +
        std::to_string(depthFrame->height()) + "_" + std::to_string(index) +
        "_" + std::to_string(depthFrame->timeStamp()) + "ms.png";
    cv::Mat depthMat(depthFrame->height(), depthFrame->width(), CV_16UC1,
                     depthFrame->data());
    cv::imwrite(depthName, depthMat, compression_params);
    std::cout << "Depth saved:" << depthName << std::endl;
  }
}

// Save the color image in png format
void saveColorPng(std::shared_ptr<ob::ColorFrame> colorFrame, int index) {
  if (index < 300) {
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
    std::string colorName =
        "F://imgs/rgb/Color_" + std::to_string(colorFrame->width()) + "x" +
        std::to_string(colorFrame->height()) + "_" + std::to_string(index) +
        "_" + std::to_string(colorFrame->timeStamp()) + "ms.png";
    cv::Mat colorRawMat(colorFrame->height(), colorFrame->width(), CV_8UC3,
                        colorFrame->data());
    cv::imwrite(colorName, colorRawMat, compression_params);
    std::cout << "Color saved:" << colorName << std::endl;
  }
}

void OrbbecDemo::saveOrShowAll() {
  depthCount = 0;
  colorCount = 0;
  try {
    pipe.stop();
    pipe.enableFrameSync();
    pipe.start(config);
    cv::VideoWriter depthWriter("F://imgs/depth/record.avi",
                                cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30,
                                cv::Size(1280, 800), false);
    cv::VideoWriter rgbWriter("F://imgs/rgb/record.avi",
                              cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30,
                              cv::Size(1920, 1080));
    while (cv::waitKey() != 27) {
      // 以阻塞的方式等待一帧数据，该帧是一个复合帧，配置里启用的所有流的帧数据都会包含在frameSet内，
      // 并设置帧的等待超时时间为100ms
      auto frame_set = pipe.waitForFrames(100);
      if (frame_set == nullptr) {
        continue;
      }
      cv::Mat img;
      if (!frame_set->depthFrame()) {
        continue;
      }
      if (!frame_set->colorFrame()) {
        continue;
      }
      img = frame2Mat(frame_set->depthFrame());
      if (depthCount < 30) {
        depthWriter.write(img);
        saveDepthPng(frame_set->depthFrame(), depthCount++);
      } else {
        depthWriter.release();
      }
      ui.label_depth->setPixmap(QPixmap::fromImage(mat2QImage(img)));
      img = frame2Mat(frame_set->colorFrame());
      if (colorCount < 30) {
        saveColorPng(frame_set->colorFrame(), colorCount++);
        rgbWriter.write(img);
      } else {
        rgbWriter.release();
      }
      ui.label_color->setPixmap(QPixmap::fromImage(mat2QImage(img)));
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

void OrbbecDemo::showDepth(bool isRecord) {
  try {
    pipe.start(config);
    cv::VideoWriter writer("F://imgs/depth/record.avi",
                           cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30,
                           cv::Size(640, 400), false);
    while (cv::waitKey() != 27) {
      // 以阻塞的方式等待一帧数据，该帧是一个复合帧，配置里启用的所有流的帧数据都会包含在frameSet内，
      // 并设置帧的等待超时时间为100ms
      auto frame_set = pipe.waitForFrames(100);
      if (frame_set == nullptr) {
        continue;
      }
      if (!frame_set->depthFrame()) {
        continue;
      }
      cv::Mat img;
      img = frame2Mat(frame_set->depthFrame());
      if (isRecord && depthCount < 30) {
        writer.write(img);
        saveDepthPng(frame_set->depthFrame(), depthCount++);
      }
      if (depthCount >= 30 || !isRecord) {
        writer.release();
      }
      ui.label_depth->setPixmap(QPixmap::fromImage(mat2QImage(img)));
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

void OrbbecDemo::showRGB(bool isRecord) {
  try {
    pipe.start(config);
    cv::VideoWriter writer("F://imgs/rgb/record.avi",
                           cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30,
                           cv::Size(640, 480));
    while (cv::waitKey() != 27) {
      // 以阻塞的方式等待一帧数据，该帧是一个复合帧，配置里启用的所有流的帧数据都会包含在frameSet内，
      // 并设置帧的等待超时时间为100ms
      auto frame_set = pipe.waitForFrames(100);
      if (frame_set == nullptr) {
        continue;
      }
      // Create a format conversion Filter
      ob::FormatConvertFilter formatConvertFilter;
      cv::Mat img;
      if (!frame_set->colorFrame()) {
        continue;
      }
      img = frame2Mat(frame_set->colorFrame());
      auto colorFrame = frame_set->colorFrame();
      if (isRecord && colorCount < 300) {
        if (colorFrame->format() != OB_FORMAT_RGB) {
          if (colorFrame->format() == OB_FORMAT_MJPG) {
            formatConvertFilter.setFormatConvertType(FORMAT_MJPG_TO_RGB888);
          } else if (colorFrame->format() == OB_FORMAT_UYVY) {
            formatConvertFilter.setFormatConvertType(FORMAT_UYVY_TO_RGB888);
          } else if (colorFrame->format() == OB_FORMAT_YUYV) {
            formatConvertFilter.setFormatConvertType(FORMAT_YUYV_TO_RGB888);
          } else {
            std::cout << "Color format is not support!" << std::endl;
            continue;
          }
          colorFrame =
              formatConvertFilter.process(colorFrame)->as<ob::ColorFrame>();
        }
        formatConvertFilter.setFormatConvertType(FORMAT_RGB888_TO_BGR);
        colorFrame =
            formatConvertFilter.process(colorFrame)->as<ob::ColorFrame>();
        saveColorPng(colorFrame, colorCount++);
        writer.write(img);
      } else {
        writer.release();
      }
      ui.label_color->setPixmap(QPixmap::fromImage(mat2QImage(img)));
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

void OrbbecDemo::startDepth() {
  pipe.stop();
  depthCount = 0;
  showDepth(true);
}

void OrbbecDemo::stopDepth() {
  pipe.stop();
}

void OrbbecDemo::startRGB() {
  pipe.stop();
  colorCount = 0;
  showRGB(true);
}

void OrbbecDemo::stopRGB() {
  pipe.stop();
}
