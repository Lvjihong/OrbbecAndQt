#include "WeightEstimation.h"

//��ɫ����ڲ�
const float color_cx = 646.308350;
const float color_cy = 359.501068;
const float color_fx = 690.507568;
const float color_fy = 690.749939;

const float new_cx = color_cx - 280;
WeightEstimation::WeightEstimation(QWidget* parent) : QWidget(parent) {
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

  init_models();

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

    test_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count();

    estimate_weight_by_img_plus(img_rgb, img_depth);
  });

  connect(ui.btn_exit, &QPushButton::clicked, [=]() {
    pipe.stop();
    close();
  });

  connect(ui.btn_start, &QPushButton::clicked, [=]() { showDepth(); });

  connect(this, &WeightEstimation::show_img, this, &WeightEstimation::showImg);
}

WeightEstimation::WeightEstimation(const WeightEstimation& window) {
  ui = window.ui;
}

WeightEstimation::~WeightEstimation() {}
void WeightEstimation::init_models() {
  // ����ʵ���ָ�ģ��
  std::string seg_model_path = "./models/segment.onnx";
  if (task_segment.ReadModel(seg_net, seg_model_path, true)) {
    std::cout << "read net ok!" << std::endl;
  }

  // ���ؼ��ͷβ��ģ��
  std::string detect_model_path = "./models/detect.onnx";
  if (task_detect.ReadModel(detect_net, detect_model_path, true)) {
    std::cout << "read net ok!" << std::endl;
  }

  // ���ع���ģ��
  std::string estimate_model_path = "./models/estimate.onnx";  //ģ�ʹ洢��·��
  estimate_net = cv::dnn::readNetFromONNX(estimate_model_path);
}
void WeightEstimation::showDepth() {
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
      // segment(Train::frame2Mat(frame_set->colorFrame())[0]);

      // QMessageBox msgBox;
      // test_time =
      // std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()
      // - test_time;
      // msgBox.setText(QString::fromStdString(std::to_string(Train::frame2Mat(frame_set->depthFrame())[1].depth())));
      // msgBox.exec();

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

void WeightEstimation::segment(cv::Mat rgb_img, cv::Mat depth_img) {
  // �ָ�
  cv::Mat img_show;
  std::vector<cv::Scalar> color_seg;
  std::vector<OutputSeg> result;  // �������������Χ���mask

  color_seg.push_back(cv::Scalar(0, 0, 255));
  if (task_segment.Detect(rgb_img, seg_net, result)) {
    std::vector<cv::Mat> vec =
        DrawPred(rgb_img, depth_img, result, task_segment._className,
                 color_seg);  //����Ĥ�Ͱ�Χ�������ͼƬ��
    img_show = vec.at(1);

    // ����Ԥ����
    cv::resize(depth_img, depth_img, cv::Size(224, 224));
    depth_img.convertTo(depth_img, CV_32FC1);
    PreProcess(depth_img, depth_img);
    // ��������
    cv::Mat inputBlob = cv::dnn::blobFromImage(
        depth_img, 1.0, cv::Size(224, 224), cv::Scalar(0, 0, 0), false, true);
    estimate_net.setInput(inputBlob);
    cv::Mat prob = estimate_net.forward();
    ui.label_weight->setText(QString::number(prob.at<float>(0, 0)) + "kg");
  } else {
    // ���ԼӸ���ʾ���棬û�м�⵽ͼƬ�е���

    img_show = rgb_img;
  }
  emit(show_img(img_show));
}
void WeightEstimation::showImg(cv::Mat img) {
  ui.label_depth->setPixmap(QPixmap::fromImage(Train::mat2QImage(img)));
  // QMessageBox msgBox;
  // test_time =
  // std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()
  // - test_time;

  // msgBox.setText(QString::fromStdString(std::to_string(test_time) + "ms"));
  // msgBox.exec();
}

std::vector<std::vector<int>> get_point(cv::Mat rgb_img, cv::Mat depth_img,
                                        std::vector<OutputSeg> boxes) {
  int head = -1;
  int body = -1;
  int hip = -1;
  int direction = -1;
  for (int i = 0; i < boxes.size(); i++) {
    auto box = boxes[i];
    if (box.id == 1) {
      if (head != -1 && boxes[head].confidence < boxes[i].confidence) head = i;
      if (head == -1) head = i;
    } else if (box.id == 0) {
      if (body != -1 && boxes[body].confidence < boxes[i].confidence) body = i;
      if (body == -1) body = i;
    } else {
      if (hip != -1 && boxes[hip].confidence < boxes[i].confidence) hip = i;
      if (hip == -1) hip = i;
    }
  }

  if (body == -1) {
    std::cout << "No body region..." << std::endl;
    exit(1);
  }
  std::vector<int> body_center = {
      boxes[body].box.x + cvRound(boxes[body].box.width / 2.0),
      boxes[body].box.y + cvRound(boxes[body].box.height / 2.0)};
  std::vector<int> head_center = {
      boxes[head].box.x + cvRound(boxes[head].box.width / 2.0),
      boxes[head].box.y + cvRound(boxes[head].box.height / 2.0)};
  std::vector<int> hip_center = {
      boxes[hip].box.x + cvRound(boxes[hip].box.width / 2.0),
      boxes[hip].box.y + cvRound(boxes[hip].box.height / 2.0)};

  std::vector<int> pre_center;

  if (head != -1 && hip != -1) {
    if (boxes[head].confidence > boxes[hip].confidence) {
      std::vector<int> v1 = {head_center[0] - body_center[0],
                             head_center[1] - body_center[1]};
      int v1_norm = sqrt(v1[0] * v1[0] + v1[1] * v1[1]);
      std::vector<int> pre_point = {
          body_center[0] + 30 * v1[0] / v1_norm,
          body_center[1] + 30 * v1[1] / v1_norm,
      };
      pre_center.emplace_back(pre_point[0]);
      pre_center.emplace_back(pre_point[1]);
    } else {
      std::vector<int> v1 = {body_center[0] - hip_center[0],
                             body_center[1] - hip_center[1]};
      int v1_norm = sqrt(v1[0] * v1[0] + v1[1] * v1[1]);
      std::vector<int> pre_point = {
          body_center[0] + 30 * v1[0] / v1_norm,
          body_center[1] + 30 * v1[1] / v1_norm,
      };
      pre_center.emplace_back(pre_point[0]);
      pre_center.emplace_back(pre_point[1]);
    }
  } else if (head == -1 && hip != -1) {
    std::vector<int> v1 = {body_center[0] - hip_center[0],
                           body_center[1] - hip_center[1]};
    int v1_norm = sqrt(v1[0] * v1[0] + v1[1] * v1[1]);
    std::vector<int> pre_point = {
        body_center[0] + 30 * v1[0] / v1_norm,
        body_center[1] + 30 * v1[1] / v1_norm,
    };
    pre_center.emplace_back(pre_point[0]);
    pre_center.emplace_back(pre_point[1]);
  } else if (head != -1 && hip == -1) {
    std::vector<int> v1 = {head_center[0] - body_center[0],
                           head_center[1] - body_center[1]};
    int v1_norm = sqrt(v1[0] * v1[0] + v1[1] * v1[1]);
    std::vector<int> pre_point = {
        body_center[0] + 30 * v1[0] / v1_norm,
        body_center[1] + 30 * v1[1] / v1_norm,
    };
    pre_center.emplace_back(pre_point[0]);
    pre_center.emplace_back(pre_point[1]);

  } else {
    std::cout << "No head region and No hip region..." << std::endl;
    exit(1);
  }

  std::vector<int> v0 = {pre_center[0] - body_center[0],
                         pre_center[1] - body_center[1]};
  int v0_norm = sqrt(v0[0] * v0[0] + v0[1] * v0[1]);

  while (depth_img.at<unsigned short>(body_center[0], body_center[1]) == 0) {
    body_center[0] -= 2 * v0[0] / v0_norm;
    body_center[1] -= 2 * v0[1] / v0_norm;
  }
  while (depth_img.at<unsigned short>(pre_center[0], pre_center[1]) == 0) {
    pre_center[0] += 2 * v0[0] / v0_norm;
    pre_center[1] += 2 * v0[1] / v0_norm;
  }

  std::vector<std::vector<int>> ret;
  ret.emplace_back(pre_center);
  ret.emplace_back(body_center);
  return ret;
}
bool cmp_max(pcl::PointIndices& x, pcl::PointIndices& y) {
  return x.indices.size() > y.indices.size();
}
//��Ĥ�ָ�����
pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg_pig_cloud(const cv::Mat& depth,
                                                     const cv::Mat& rgb,
                                                     cv::Mat mask) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  for (int m = 0; m < depth.rows; m++)
    for (int n = 0; n < depth.cols; n++) {
      if (mask.at<UINT8>(m, n) != 0) {
        ushort d = depth.ptr<ushort>(m)[n];
        if (d == 0) continue;
        pcl::PointXYZRGB p;
        p.z = double(d) / 1000;
        p.x = (n - color_cx) * p.z / color_fx;
        p.y = (m - color_cy) * p.z / color_fy;
        p.b = rgb.ptr<uchar>(m)[n * 3];
        p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
        p.r = rgb.ptr<uchar>(m)[n * 3 + 2];
        cloud->points.emplace_back(p);
      }
    }
  cloud->height = 1;
  cloud->width = cloud->points.size();
  cloud->is_dense = false;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_result(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(100);
  sor.setStddevMulThresh(1.7);
  sor.setNegative(false);
  sor.filter(*cloud_result);

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud_result);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(0.02);
  ec.setMinClusterSize(3000);
  ec.setMaxClusterSize(200000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_result);
  ec.extract(cluster_indices);
  std::sort(cluster_indices.begin(), cluster_indices.end(), cmp_max);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_seg(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*cloud_result, cluster_indices[0], *cloud_seg);
  cloud->points.clear();
  cloud_result->points.clear();
  std::cout << "PointCloud representing the Cluster:"
            << cloud_seg->points.size() << std::endl;
  return cloud_seg;
}

// runcode3:�ú���Ϊʵ���ָ�+��̬��һ�����
cv::Mat runcode3(int pixel_x_head, int pixel_y_head, int pixel_x_body,
                 int pixel_y_body, Eigen::Affine3f scene_sensor_camera,
                 cv::Mat depth_img, cv::Mat rgb_img, cv::Mat mask) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  for (int m = 0; m < depth_img.rows; m++)
    for (int n = 0; n < depth_img.cols; n++) {
      ushort d = depth_img.ptr<ushort>(m)[n];
      if (d == 0) continue;
      pcl::PointXYZRGB tempp;
      tempp.z = double(d) / 1000;
      tempp.x = (n - color_cx) * tempp.z / color_fx;
      tempp.y = (m - color_cy) * tempp.z / color_fy;
      tempp.b = rgb_img.ptr<uchar>(m)[n * 3];
      tempp.g = rgb_img.ptr<uchar>(m)[n * 3 + 1];
      tempp.r = rgb_img.ptr<uchar>(m)[n * 3 + 2];
      cloud_ptr->points.push_back(tempp);
    }
  cloud_ptr->height = 1;
  cloud_ptr->width = cloud_ptr->points.size();

  Eigen::Affine3f camera_to_Pixel_matrix(Eigen::Affine3f::Identity());
  Eigen::Affine3f pixel_to_world_matrix(Eigen::Affine3f::Identity());
  camera_to_Pixel_matrix(0, 0) = color_fx;
  camera_to_Pixel_matrix(1, 1) = color_fy;
  camera_to_Pixel_matrix(0, 2) = color_cx;
  camera_to_Pixel_matrix(1, 2) = color_cy;
  pixel_to_world_matrix = camera_to_Pixel_matrix.inverse();
  //�������峯��
  Eigen::Vector3f head_point(pixel_x_head, pixel_y_head,
                             depth_img.ptr<ushort>(pixel_y_head)[pixel_x_head]);
  Eigen::Vector3i head_color(
      rgb_img.ptr<uchar>(pixel_y_head)[pixel_x_head * 3 + 2],
      rgb_img.ptr<uchar>(pixel_y_head)[pixel_x_head * 3 + 1],
      rgb_img.ptr<uchar>(pixel_y_head)[pixel_x_head * 3]);
  Eigen::Vector3f body_point(pixel_x_body, pixel_y_body,
                             depth_img.ptr<ushort>(pixel_y_body)[pixel_x_body]);
  Eigen::Vector3i body_color(
      rgb_img.ptr<uchar>(pixel_y_body)[pixel_x_body * 3 + 2],
      rgb_img.ptr<uchar>(pixel_y_body)[pixel_x_body * 3 + 1],
      rgb_img.ptr<uchar>(pixel_y_body)[pixel_x_body * 3]);

  PigHeadDirectionEstimation pig(cloud_ptr, scene_sensor_camera);

  pig.setmatrix(camera_to_Pixel_matrix, pixel_to_world_matrix, 720,
                1280);  //���ñ任��������ͼ����
  pig.caculateHeadDirection2(head_point, head_color, body_point, body_color);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr single_point_1(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  single_point_1->points.push_back(pig.getHeadPoint());
  single_point_1->points.push_back(pig.getbodyPoint());
  single_point_1->height = 1;
  single_point_1->width = single_point_1->points.size();
  //�ָ�������㷨��
  SegementationPig S_pig(cloud_ptr, scene_sensor_camera);

  S_pig.create_ground_plane_vector(cloud_ptr, pig.getbodyPoint());

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ss =
      seg_pig_cloud(depth_img, rgb_img, mask);

  pcl::ModelCoefficients plane_coeff_af_mirror;
  plane_coeff_af_mirror.values.resize(4);
  plane_coeff_af_mirror.values[0] = S_pig.up_ground_vec_1[0];
  plane_coeff_af_mirror.values[1] = S_pig.up_ground_vec_1[1];
  plane_coeff_af_mirror.values[2] = S_pig.up_ground_vec_1[2];
  plane_coeff_af_mirror.values[3] = S_pig.up_ground_vec_1[3];

  //����Գ�ƽ��
  SymmetricalPlane S_plane;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ss_1(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  for (int i = 0; i < ss->size(); i++) {
    pcl::PointXYZRGBA p;
    p.x = ss->points[i].x;
    p.y = ss->points[i].y;
    p.z = ss->points[i].z;
    p.r = ss->points[i].r;
    p.g = ss->points[i].g;
    p.b = ss->points[i].b;
    p.a = 0;
    ss_1->points.push_back(p);
  }
  ss_1->width = ss_1->points.size();
  ss_1->height = 1;
  S_plane.Reset();
  S_plane.SetSearchRadius(0.1);
  S_plane.SetUpVector(S_pig.up_ground_vec_1);
  S_plane.SetBiggestProfile(ss_1);
  S_plane.Do_Computing_thread();

  //λ�˹�һ������
  PoseNormalization poseN;
  poseN.Reset();
  poseN.SetInputcloud(ss_1);
  poseN.Set_gd_plane(S_pig.up_ground_vec_1);
  Eigen::Vector3f head_dir(pig.getHeadPoint().x - pig.getbodyPoint().x,
                           pig.getHeadPoint().y - pig.getbodyPoint().y,
                           pig.getHeadPoint().z - pig.getbodyPoint().z);
  poseN.Set_head_direction(head_dir);

  poseN.Set_symmetry_plane(S_plane.get_estimated_plane());  //�趨�Գ�ƽ��
  poseN.Computing_thread();
  pcl::PointCloud<pcl::PointXYZRGBA> ssm = poseN.Get_normalizated_data();
  //��ͶӰ
  cv::Mat result_dep = cv::Mat::zeros(720, 720, CV_16UC1);
  for (int j = 0; j < ssm.points.size(); j++) {
    pcl::PointXYZRGBA temp_p = ssm.points[j];
    double dep = (temp_p.z + 4.5) * 1000;
    int c = (temp_p.x * color_fx) / (temp_p.z + 4.5) + new_cx;
    int r = (temp_p.y * color_fy) / (temp_p.z + 4.5) + color_cy;
    if (c > 719)
      c = 719;
    else if (c < 0)
      c = 0;
    if (r > 719)
      r = 719;
    else if (r < 0)
      r = 0;
    result_dep.at<ushort>(r, c) = (ushort)dep;
  }
  return result_dep;
}

void WeightEstimation::PreProcess(const cv::Mat& image, cv::Mat& image_blob) {
  cv::Mat input;
  image.copyTo(input);
  std::vector<cv::Mat> channel_p;
  cv::Mat x1, x2, x3;
  for (int row = 0; row < input.rows; row++) {
    for (int col = 0; col < input.cols; col++) {
      float pixelValue = input.at<float>(row, col);
      if (pixelValue != 0) {
        input.at<float>(row, col) = (pixelValue - 2100) / 4300;
      }
    }
  }
  x1 = (input - 0.485) / 0.229;
  x2 = (input - 0.456) / 0.224;
  x3 = (input - 0.406) / 0.225;
  channel_p.push_back(x1);
  channel_p.push_back(x2);
  channel_p.push_back(x3);

  cv::Mat outt;
  merge(channel_p, outt);
  image_blob = outt;
}
void WeightEstimation::estimate_weight_by_img_plus(cv::Mat rgb_img,
                                                   cv::Mat depth_img) {
  // 1.��ȡͷβ���Լ�mask
  // 1.1 ��ȡmask
  cv::Mat img_show;
  cv::Mat mask;
  cv::Mat rgb_clone = rgb_img.clone();
  cv::Mat depth_clone = depth_img.clone();
  std::vector<cv::Scalar> color_seg;
  std::vector<OutputSeg> result;  // �������������Χ���mask

  color_seg.push_back(cv::Scalar(0, 0, 255));
  if (task_segment.Detect(rgb_img, seg_net, result)) {
    std::vector<cv::Mat> vec =
        DrawPred(rgb_img, depth_img, result, task_segment._className,
                 color_seg);  //����Ĥ�Ͱ�Χ�������ͼƬ��

    img_show = vec.at(1);
    mask = vec.at(0);

      // 1.2 ��ȡͷβ���
      std::vector<cv::Scalar> color_det;
      color_det.push_back(cv::Scalar(0, 0, 255));
      color_det.push_back(cv::Scalar(255, 0, 0));
      color_det.push_back(cv::Scalar(0, 255, 0));
      std::vector<OutputSeg> boxes;
      if (!task_detect.Detect(rgb_clone, detect_net, boxes)) {
        std::cout << "Detect Failed!" << std::endl;
      }

      std::vector<std::vector<int>> points =
          get_point(rgb_clone, depth_clone, boxes);

      // 2.��̬��һ��
      Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());  //��λ��
      cv::Mat final_depth =
          runcode3(points[0][0], points[0][1], points[1][0], points[1][1],
                   scene_sensor_pose, depth_clone, rgb_clone, mask);

      // 3.��������

      cv::resize(final_depth, final_depth, cv::Size(224, 224));
      final_depth.convertTo(final_depth, CV_32FC1);
      PreProcess(final_depth, final_depth);
      cv::Mat inputBlob =
          cv::dnn::blobFromImage(final_depth, 1.0, cv::Size(224, 224),
                                 cv::Scalar(0, 0, 0), false, true);

      estimate_net.setInput(inputBlob);
      cv::Mat prob = estimate_net.forward();
      ui.label_weight->setText(QString::number(prob.at<float>(0, 0)) + "kg");

    } else {
      // ���ԼӸ���ʾ���棬û�м�⵽ͼƬ�е���

      img_show = rgb_img;
    }
    emit(show_img(img_show));
  }