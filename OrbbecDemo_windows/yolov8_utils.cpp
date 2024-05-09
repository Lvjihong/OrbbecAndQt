#pragma once
#include "yolov8_utils.h"
using namespace cv;
using namespace std;

bool CheckParams(int netHeight, int netWidth, const int* netStride,
                 int strideSize) {
  if (netHeight % netStride[strideSize - 1] != 0 ||
      netWidth % netStride[strideSize - 1] != 0) {
    cout << "Error:_netHeight and _netWidth must be multiple of max stride "
         << netStride[strideSize - 1] << "!" << endl;
    return false;
  }
  return true;
}

bool CheckModelPath(std::string modelPath) {
  if (0 != _access(modelPath.c_str(), 0)) {
    cout << "Model path does not exist,  please check " << modelPath << endl;
    return false;
  } else
    return true;
}

void LetterBox(const cv::Mat& image, cv::Mat& outImage, cv::Vec4d& params,
               const cv::Size& newShape, bool autoShape, bool scaleFill,
               bool scaleUp, int stride,
               const cv::Scalar& color)  //�ı�ͼƬ�Ĵ�С��ʹ֮�������������
{
  if (false) {
    int maxLen = MAX(image.rows, image.cols);
    outImage = Mat::zeros(Size(maxLen, maxLen), CV_8UC3);
    image.copyTo(outImage(Rect(0, 0, image.cols, image.rows)));
    params[0] = 1;
    params[1] = 1;
    params[3] = 0;
    params[2] = 0;
  }
  cv::Size shape = image.size();
  float r = std::min((float)newShape.height / (float)shape.height,
                     (float)newShape.width / (float)shape.width);
  if (!scaleUp) r = std::min(r, 1.0f);

  float ratio[2]{r, r};
  int new_un_pad[2] = {(int)std::round((float)shape.width * r),
                       (int)std::round((float)shape.height * r)};

  auto dw = (float)(newShape.width - new_un_pad[0]);
  auto dh = (float)(newShape.height - new_un_pad[1]);

  if (autoShape) {
    dw = (float)((int)dw % stride);
    dh = (float)((int)dh % stride);
  } else if (scaleFill) {
    dw = 0.0f;
    dh = 0.0f;
    new_un_pad[0] = newShape.width;
    new_un_pad[1] = newShape.height;
    ratio[0] = (float)newShape.width / (float)shape.width;
    ratio[1] = (float)newShape.height / (float)shape.height;
  }

  dw /= 2.0f;
  dh /= 2.0f;

  if (shape.width != new_un_pad[0] && shape.height != new_un_pad[1]) {
    cv::resize(image, outImage, cv::Size(new_un_pad[0], new_un_pad[1]));
  } else {
    outImage = image.clone();
  }

  int top = int(std::round(dh - 0.1f));
  int bottom = int(std::round(dh + 0.1f));
  int left = int(std::round(dw - 0.1f));
  int right = int(std::round(dw + 0.1f));
  params[0] = ratio[0];
  params[1] = ratio[1];
  params[2] = left;
  params[3] = top;
  cv::copyMakeBorder(outImage, outImage, top, bottom, left, right,
                     cv::BORDER_CONSTANT, color);
}
void GetMask(const cv::Mat& maskProposals, const cv::Mat& maskProtos,
             std::vector<OutputSeg>& output, const MaskParams& maskParams) {
  // cout << maskProtos.size << endl;

  int net_width = maskParams.netWidth;
  int net_height = maskParams.netHeight;
  int seg_channels = maskProtos.size[1];
  int seg_height = maskProtos.size[2];
  int seg_width = maskProtos.size[3];
  float mask_threshold = maskParams.maskThreshold;
  Vec4f params = maskParams.params;
  Size src_img_shape = maskParams.srcImgShape;

  Mat protos = maskProtos.reshape(0, {seg_channels, seg_width * seg_height});

  Mat matmul_res = (maskProposals * protos).t();
  Mat masks = matmul_res.reshape(output.size(), {seg_width, seg_height});
  vector<Mat> maskChannels;
  split(masks, maskChannels);
  for (int i = 0; i < output.size(); ++i) {
    Mat dest, mask;
    // sigmoid
    cv::exp(-maskChannels[i], dest);
    dest = 1.0 / (1.0 + dest);

    Rect roi(int(params[2] / net_width * seg_width),
             int(params[3] / net_height * seg_height),
             int(seg_width - params[2] / 2), int(seg_height - params[3] / 2));
    dest = dest(roi);
    resize(dest, mask, src_img_shape, INTER_NEAREST);

    // crop
    Rect temp_rect = output[i].box;
    mask = mask(temp_rect) > mask_threshold;
    output[i].boxMask = mask;
  }
}

void GetMask2(const Mat& maskProposals, const Mat& maskProtos,
              OutputSeg& output, const MaskParams& maskParams) {
  int net_width = maskParams.netWidth;
  int net_height = maskParams.netHeight;
  int seg_channels = maskProtos.size[1];
  int seg_height = maskProtos.size[2];
  int seg_width = maskProtos.size[3];
  float mask_threshold = maskParams.maskThreshold;
  Vec4f params = maskParams.params;
  Size src_img_shape = maskParams.srcImgShape;

  Rect temp_rect = output.box;
  // crop from mask_protos
  int rang_x =
      floor((temp_rect.x * params[0] + params[2]) / net_width * seg_width);
  int rang_y =
      floor((temp_rect.y * params[1] + params[3]) / net_height * seg_height);
  int rang_w = ceil(((temp_rect.x + temp_rect.width) * params[0] + params[2]) /
                    net_width * seg_width) -
               rang_x;
  int rang_h = ceil(((temp_rect.y + temp_rect.height) * params[1] + params[3]) /
                    net_height * seg_height) -
               rang_y;

  //��������
  // mask_protos(roi_rangs).clone()λ�ñ���˵�����output.box���ݲ��ԣ����߾��ο��1�����صģ����������ע�Ͳ��ַ�ֹ����
  rang_w = MAX(rang_w, 1);
  rang_h = MAX(rang_h, 1);
  if (rang_x + rang_w > seg_width) {
    if (seg_width - rang_x > 0)
      rang_w = seg_width - rang_x;
    else
      rang_x -= 1;
  }
  if (rang_y + rang_h > seg_height) {
    if (seg_height - rang_y > 0)
      rang_h = seg_height - rang_y;
    else
      rang_y -= 1;
  }

  vector<Range> roi_rangs;
  roi_rangs.push_back(Range(0, 1));
  roi_rangs.push_back(Range::all());
  roi_rangs.push_back(Range(rang_y, rang_h + rang_y));
  roi_rangs.push_back(Range(rang_x, rang_w + rang_x));

  // crop
  Mat temp_mask_protos = maskProtos(roi_rangs).clone();
  Mat protos = temp_mask_protos.reshape(0, {seg_channels, rang_w * rang_h});
  Mat matmul_res = (maskProposals * protos).t();
  Mat masks_feature = matmul_res.reshape(1, {rang_h, rang_w});
  Mat dest, mask;

  // sigmoid
  cv::exp(-masks_feature, dest);
  dest = 1.0 / (1.0 + dest);

  int left = floor((net_width / seg_width * rang_x - params[2]) / params[0]);
  int top = floor((net_height / seg_height * rang_y - params[3]) / params[1]);
  int width = ceil(net_width / seg_width * rang_w / params[0]);
  int height = ceil(net_height / seg_height * rang_h / params[1]);

  resize(dest, mask, Size(width, height), INTER_NEAREST);
  Rect mask_rect = temp_rect - Point(left, top);
  mask_rect &= Rect(0, 0, width, height);
  mask = mask(mask_rect) > mask_threshold;
  if (mask.rows != temp_rect.height ||
      mask.cols !=
          temp_rect
              .width) {  // https://github.com/UNeedCryDear/yolov8-opencv-onnxruntime-cpp/pull/30
    resize(mask, mask, temp_rect.size(), INTER_NEAREST);
  }
  output.boxMask = mask;
}

// ԭʼ���mask
Mat DrawPred(Mat& img, vector<OutputSeg> result,
             std::vector<std::string> classNames, vector<Scalar> color,
             bool isVideo) {
  Mat mask = img.clone();

  // ��ͼ������
  double img_center_x = img.cols / 2;
  double img_center_y = img.rows / 2;

  pair<int, double> min_point(-1, std::numeric_limits<double>::max());
  int left, top;
  for (int i = 0; i < result.size(); i++) {
    left = result[i].box.x;
    top = result[i].box.y;
    //��Ŀ������
    int center_x = left + result[i].box.width / 2;
    int center_y = top + result[i].box.height / 2;
    double offset =
        pow(center_x - img_center_x, 2) + pow(center_y - img_center_y, 2);
    if (offset < min_point.second) {
      min_point.first = i;
      min_point.second = offset;
    }
  }

  int target_num = min_point.first;
  rectangle(img, result[target_num].box, color[result[target_num].id], 2, 8);
  cv::circle(
      img,
      cv::Point(result[target_num].box.x + result[target_num].box.width / 2,
                result[target_num].box.y + result[target_num].box.height / 2),
      5, cv::Scalar(255, 0, 0), -1);
  if (result[target_num].boxMask.rows && result[target_num].boxMask.cols > 0)
    mask(result[target_num].box)
        .setTo(color[result[target_num].id],
               result[target_num].boxMask);  //�ü�+�ɰ洦��
  string label = classNames[result[target_num].id] + ":" +
                 to_string(result[target_num].confidence);
  int baseLine;
  Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
  top = max(result[target_num].box.y, labelSize.height);

  putText(img, label, Point(result[target_num].box.x, result[target_num].box.y),
          FONT_HERSHEY_SIMPLEX, 1, color[result[target_num].id], 2);

  addWeighted(img, 0.5, mask, 0.5, 0, img);  // add mask to src

  return img;
}

// �Ľ����mask
vector<Mat> DrawPred(Mat& img, Mat depth_img, vector<OutputSeg> result,
                     std::vector<std::string> classNames, vector<Scalar> color,
                     bool isVideo) {
  vector<Mat> ret;
  double threshold = 40.0;
  // ��¡����ͼ���Դ�������ͼ������ͼ��
  Mat mask = img.clone();

  // ��������ͼ�����������
  double img_center_x = img.cols / 2;
  double img_center_y = img.rows / 2;

  // ��ʼ��һ�ԣ����ڸ�����С���������ƫ����
  pair<int, double> min_point(-1, std::numeric_limits<double>::max());
  int left, top;

  cv::Mat test_mask = cv::Mat::zeros(depth_img.rows, depth_img.cols, CV_8UC1);

  // ѭ���������
  for (int i = 0; i < result.size(); i++) {
    // ��ȡ�߽������Ͻ�����
    left = result[i].box.x;
    top = result[i].box.y;

    // ���㵱ǰ�߽�����������
    int center_x = left + result[i].box.width / 2;
    int center_y = top + result[i].box.height / 2;

    // ���������ͼ�����ĵ�ƫ������������Ҫʱ���� min_point
    double offset =
        pow(center_x - img_center_x, 2) + pow(center_y - img_center_y, 2);
    if (offset < min_point.second) {
      min_point.first = i;
      min_point.second = offset;
    }
  }

  // ��ȡ������Сƫ�Ƶı߽�������
  int target_index = min_point.first;

  // ������ͼ���ϻ���Χ�ƶ���ľ���
  rectangle(img, result[target_index].box, color[result[target_index].id], 2,
            8);

  // ������ͼ���϶�������Ļ���ԲȦ
  cv::circle(
      img,
      cv::Point(
          result[target_index].box.x + result[target_index].box.width / 2,
          result[target_index].box.y + result[target_index].box.height / 2),
      5, cv::Scalar(255, 0, 0), -1);

  // ��ȡ���ͼ��� ROI����������ֵ�Ĳ��֣�
  Mat depth_roi = depth_img(result[target_index].box);

  depth_roi.setTo(Scalar(0),
                  result[target_index].boxMask == 0);  // ����Ч������Ϊ0
  // ����һ�����룬��ʶ���ͼ�з���ֵ��λ��
  cv::Mat depth_nroi = (depth_roi != 0);

  // ʹ���������ƽ�����
  cv::Scalar meanDepth = cv::mean(depth_roi, depth_nroi);

  double avgDepth = meanDepth[0];

  // ����ƽ���������ֵ�Ƚϣ�ѡ���ɰ���ɫ
  Scalar maskColor =
      (avgDepth / 4 / 10 > threshold) ? Scalar(0, 0, 255) : Scalar(0, 255, 0);

  // ���ݱ߽����ɰ��ڿ�¡������ͼ����Ӧ����ɫ�ɰ�
  if (result[target_index].boxMask.rows &&
      result[target_index].boxMask.cols > 0)
    mask(result[target_index].box)
        .setTo(maskColor,
               result[target_index].boxMask);  // �ü� + �ɰ����

  test_mask(result[target_index].box);

  test_mask(result[target_index].box)
      .setTo(Scalar(255, 255, 255), result[target_index].boxMask);
  ret.emplace_back(test_mask);

  // �����������������Ŷȵı�ǩ
  string label = classNames[result[target_index].id] + ":" +
                 to_string(result[target_index].confidence);

  // ������ñ�ǩ��λ��
  int baseLine;
  Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
  top = max(result[target_index].box.y, labelSize.height);

  // ������ͼ���Ϸ��ñ�ǩ
  putText(img, label,
          Point(result[target_index].box.x, result[target_index].box.y),
          FONT_HERSHEY_SIMPLEX, 1, color[result[target_index].id], 2);

  // ������ͼ������ͼ���ϵ�����ͼ�����Դ����������ͼ��
  addWeighted(img, 0.5, mask, 0.5, 0, img);  // ��������ӵ�Դͼ��

  // �����޸ĺ��ͼ��
  ret.emplace_back(img);
  return ret;
}

// ��ͷβ�����������ְ�Χ��ǵ�rgb��
void DrawPred_box(Mat& img, vector<OutputSeg> result,
                  std::vector<std::string> classNames, vector<Scalar> color,
                  bool isVideo) {
  Mat mask = img.clone();
  for (int i = 0; i < result.size(); i++) {
    int left, top;
    left = result[i].box.x;
    top = result[i].box.y;
    int color_num = i;
    rectangle(img, result[i].box, color[result[i].id], 2, 8);
    if (result[i].boxMask.rows && result[i].boxMask.cols > 0)
      mask(result[i].box).setTo(color[result[i].id], result[i].boxMask);
    string label =
        classNames[result[i].id] + ":" + to_string(result[i].confidence);
    int baseLine;
    Size labelSize =
        getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = max(top, labelSize.height);
    // rectangle(frame, Point(left, top - int(1.5 * labelSize.height)),
    // Point(left + int(1.5 * labelSize.width), top + baseLine), Scalar(0, 255,
    // 0), FILLED);
    putText(img, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 1,
            color[result[i].id], 2);
  }
  // addWeighted(img, 0.5, mask, 0.5, 0, img); //add mask to src
  // imshow("1", img);
  // if (!isVideo)
  // waitKey();
  // destroyAllWindows();
}
