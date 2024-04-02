#include"yolov8_seg.h"
using namespace std;
using namespace cv;
using namespace cv::dnn;

bool Yolov8Seg::ReadModel(Net& net, string& netPath, bool isCuda = false) {
	try {
		if (!CheckModelPath(netPath))
			return false;
		net = readNet(netPath);//��ȡ����ṹ

//#if CV_VERSION_MAJOR==4 &&CV_VERSION_MINOR==7&&CV_VERSION_REVISION==0
//		net.enableWinograd(false);  //bug of opencv4.7.x in AVX only platform ,https://github.com/opencv/opencv/pull/23112 and https://github.com/opencv/opencv/issues/23080 
//		//net.enableWinograd(true);		//If your CPU supports AVX2, you can set it true to speed up
//#endif

	}
	catch (const std::exception&) {
		return false;
	}

	if (isCuda) {
		//cuda
		net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
		net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA); //or DNN_TARGET_CUDA_FP16
	}
	else {
		//cpu
		cout << "Inference device: CPU" << endl;
		net.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
		net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
	}
	return true;
}

bool Yolov8Seg::Detect(Mat& srcImg, Net& net, vector<OutputSeg>& output) {
	Mat blob;//������ͼƬ
	output.clear();
	int col = srcImg.cols;
	int row = srcImg.rows;
	Mat netInputImg;
	Vec4d params;
	LetterBox(srcImg, netInputImg, params, cv::Size(_netWidth, _netHeight));//��������ͼƬ�ߴ�
	blobFromImage(netInputImg, blob, 1 / 255.0, cv::Size(_netWidth, _netHeight), cv::Scalar(0, 0, 0), true, false);//��һ��ͼƬԤ����
	//**************************************************************************************************************************************************/
	//�������������û�����������µ��ǽ��ƫ��ܴ󣬿��Գ������������������
	// If there is no problem with other settings, but results are a lot different from  Python-onnx , you can try to use the following two sentences
	// 
	//$ blobFromImage(netInputImg, blob, 1 / 255.0, cv::Size(_netWidth, _netHeight), cv::Scalar(104, 117, 123), true, false);
	//$ blobFromImage(netInputImg, blob, 1 / 255.0, cv::Size(_netWidth, _netHeight), cv::Scalar(114, 114,114), true, false);
	//****************************************************************************************************************************************************/
	net.setInput(blob);
	std::vector<cv::Mat> net_output_img;//����output0��output1
	vector<string> output_layer_names{ "output0","output1" };
	net.forward(net_output_img, output_layer_names); //get outputs
	std::vector<int> class_ids;// res-class_id
	std::vector<float> confidences;// res-conf ���Ŷ�
	std::vector<cv::Rect> boxes;// res-box
	std::vector<vector<float>> picked_proposals;  //output0[:,:, 4 + _className.size():net_width]===> for mask
	Mat output0 = Mat(Size(net_output_img[0].size[2], net_output_img[0].size[1]), CV_32F, (float*)net_output_img[0].data).t();  //[bs,116,8400]=>[bs,8400,116]
	int rows = output0.rows;//8400
	int net_width = output0.cols;//116
	int socre_array_length = net_width - 4 - net_output_img[1].size[1];//�õ����Ŷ���Ϣ�ĳ���
	float* pdata = (float*)output0.data;

	for (int r = 0; r < rows; ++r) {
		cv::Mat scores(1, socre_array_length, CV_32FC1, pdata + 4);//���Ŷ�
		Point classIdPoint;
		double max_class_socre;
		minMaxLoc(scores, 0, &max_class_socre, 0, &classIdPoint);
		max_class_socre = (float)max_class_socre;
		if (max_class_socre >= _classThreshold) {
			vector<float> temp_proto(pdata + 4 + socre_array_length, pdata + net_width);//mask���
			picked_proposals.push_back(temp_proto);
			//rect [x,y,w,h]
			float x = (pdata[0] - params[2]) / params[0];
			float y = (pdata[1] - params[3]) / params[1];
			float w = pdata[2] / params[0];
			float h = pdata[3] / params[1];
			int left = MAX(int(x - 0.5 * w + 0.5), 0);
			int top = MAX(int(y - 0.5 * h + 0.5), 0);
			class_ids.push_back(classIdPoint.x);
			confidences.push_back(max_class_socre);
			boxes.push_back(Rect(left, top, int(w + 0.5), int(h + 0.5)));
		}
		pdata += net_width;//next line
	}
	//NMS
	vector<int> nms_result;
	NMSBoxes(boxes, confidences, _classThreshold, _nmsThreshold, nms_result);
	std::vector<vector<float>> temp_mask_proposals;
	Rect holeImgRect(0, 0, srcImg.cols, srcImg.rows);
	for (int i = 0; i < nms_result.size(); ++i) {

		int idx = nms_result[i];
		OutputSeg result;
		result.id = class_ids[idx];
		result.confidence = confidences[idx];
		result.box = boxes[idx] & holeImgRect;
		temp_mask_proposals.push_back(picked_proposals[idx]);
		output.push_back(result);
	}
	MaskParams mask_params;
	mask_params.params = params;
	mask_params.srcImgShape = srcImg.size();
	mask_params.netHeight = _netHeight;
	mask_params.netWidth = _netWidth;
	mask_params.maskThreshold = _maskThreshold;
	for (int i = 0; i < temp_mask_proposals.size(); ++i) {
		GetMask2(Mat(temp_mask_proposals[i]).t(), net_output_img[1], output[i], mask_params);
	}


	//******************** ****************
	// �ϰ汾�ķ�������������ڿ�����ע�͵Ĳ���֮��һֱ��������ʹ�������
	 //If the GetMask2() still reports errors , it is recommended to use GetMask().
	//Mat mask_proposals;
	//for (int i = 0; i < temp_mask_proposals.size(); ++i) {
	//	mask_proposals.push_back(Mat(temp_mask_proposals[i]).t());
	//}
	//GetMask(mask_proposals, net_output_img[1], output, mask_params);
	//*****************************************************/


	if (output.size())
		return true;
	else
		return false;
}