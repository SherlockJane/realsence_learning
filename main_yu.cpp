#define main_yu
#ifdef main_yu
#pragma comment(lib, "legacy_stdio_definitions.lib")
#include "windows.h"
#include <iostream>
#include<fstream>
#include <string>
#include<Python.h>
#include<map>
using namespace std;
//camera
#include "pxcsensemanager.h"
#include <pxcsession.h>    
#include "util_render.h"   

// OpenCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

#define WIDTH 640    
#define HEIGHT 480  

// 相机内参

const double camera_factor = 1000;
const double camera_cx = 311.704;
const double camera_cy = 245.82;
const double camera_fx = 474.055;
const double camera_fy = 474.055;

Mat output;

void ConvertPXCImageToOpenCVMat(PXCImage *inImg, Mat *outImg) {

	int cvDataType;

	int cvDataWidth;





	PXCImage::ImageData data;

	inImg->AcquireAccess(PXCImage::ACCESS_READ, &data);

	PXCImage::ImageInfo imgInfo = inImg->QueryInfo();



	switch (data.format) {

		/* STREAM_TYPE_COLOR */

	case PXCImage::PIXEL_FORMAT_YUY2: /* YUY2 image  */

	case PXCImage::PIXEL_FORMAT_NV12: /* NV12 image */

		throw(0); // Not implemented

	case PXCImage::PIXEL_FORMAT_RGB32: /* BGRA layout on a little-endian machine */

		cvDataType = CV_8UC4;

		cvDataWidth = 4;

		break;

	case PXCImage::PIXEL_FORMAT_RGB24: /* BGR layout on a little-endian machine */

		cvDataType = CV_8UC3;

		cvDataWidth = 3;

		break;

	case PXCImage::PIXEL_FORMAT_Y8:  /* 8-Bit Gray Image, or IR 8-bit */

		cvDataType = CV_8U;

		cvDataWidth = 1;

		break;



		/* STREAM_TYPE_DEPTH */

	case PXCImage::PIXEL_FORMAT_DEPTH: /* 16-bit unsigned integer with precision mm. */

	case PXCImage::PIXEL_FORMAT_DEPTH_RAW: /* 16-bit unsigned integer with device specific precision (call device->QueryDepthUnit()) */

		cvDataType = CV_16U;

		cvDataWidth = 2;

		break;

	case PXCImage::PIXEL_FORMAT_DEPTH_F32: /* 32-bit float-point with precision mm. */

		cvDataType = CV_32F;

		cvDataWidth = 4;

		break;



		/* STREAM_TYPE_IR */

	case PXCImage::PIXEL_FORMAT_Y16:          /* 16-Bit Gray Image */

		cvDataType = CV_16U;

		cvDataWidth = 2;

		break;

	case PXCImage::PIXEL_FORMAT_Y8_IR_RELATIVE:    /* Relative IR Image */

		cvDataType = CV_8U;

		cvDataWidth = 1;

		break;

	}



	// suppose that no other planes

	if (data.planes[1] != NULL) throw(0); // not implemented

	// suppose that no sub pixel padding needed

	if (data.pitches[0] % cvDataWidth != 0) throw(0); // not implemented



	outImg->create(imgInfo.height, data.pitches[0] / cvDataWidth, cvDataType);



	memcpy(outImg->data, data.planes[0], imgInfo.height*imgInfo.width*cvDataWidth * sizeof(pxcBYTE));



	inImg->ReleaseAccess(&data);

}


void f_gray2color(Mat gray_mat, Mat color_mat)

{

	uchar* p = nullptr;

	Vec3b* q = nullptr;

	int height = gray_mat.rows;

	int width = gray_mat.cols;

	for (int i = 0; i < height; i++)

	{

		for (int j = 0; j < width; j++)

		{

			p = gray_mat.ptr<uchar>(i);

			q = color_mat.ptr<Vec3b>(i);

			if (p[j] == 0)

			{
				q[j][0] = 0; q[j][1] = 0; q[j][2] = 0;
			}

			else

			{

				q[j][0] = 255 - p[j];

				q[j][1] = 128 - abs(128 - p[j]);

				q[j][2] = p[j];

			}

		}

	}

	return;

}

int camera()
{
	int k = 0;
	pxcCHAR titlr1 = (wchar_t)L"COLOR_STREAM";
	pxcCHAR titlr2 = (wchar_t)L"DEPTH_STREAM";
	//UtilRender *renderColor = new UtilRender(&titlr1);//L"COLOR_STREAM"
	//UtilRender *renderDepth = new UtilRender(&titlr2);//L"DEPTH_STREAM"


	PXCSenseManager *psm = 0;
	psm = PXCSenseManager::CreateInstance();
	if (!psm)
	{
		wprintf_s(L"Unabel to create the PXCSenseManager\n");
		return 1;
	}
	pxcStatus sts;

	psm->EnableStream(PXCCapture::STREAM_TYPE_COLOR, WIDTH, HEIGHT);

	psm->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, WIDTH, HEIGHT);

	sts = psm->Init();
	if (sts != PXC_STATUS_NO_ERROR)
	{
		wprintf_s(L"Unabel to Initializes the pipeline\n");
		return 2;
	}

	PXCImage *colorIm, *depthIm;
	PXCImage::ImageData depth_data, color_data;
	PXCImage::ImageInfo depth_info, color_info;
	while (psm->AcquireFrame(true) >= PXC_STATUS_NO_ERROR)

	{
		if (psm->AcquireFrame(true) < PXC_STATUS_NO_ERROR) break;

		waitKey(60);
		PXCCapture::Sample *sample = psm->QuerySample();

		colorIm = sample->color;
		depthIm = sample->depth;

		if (colorIm->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB24, &color_data) < PXC_STATUS_NO_ERROR)
			wprintf_s(L"未正常获取彩色图\n");
		if (depthIm->AcquireAccess(PXCImage::ACCESS_READ, &depth_data) < PXC_STATUS_NO_ERROR)
			wprintf_s(L"未正常获取深度图\n");



		//Mat dst;
		//ConvertPXCImageToOpenCVMat(depthIm, dst);
		depth_info = sample->depth->QueryInfo();
		color_info = sample->color->QueryInfo();

		Mat depth(Size(depth_info.width, depth_info.height), CV_8UC1, (void*)depth_data.planes[0], depth_data.pitches[0] / sizeof(uchar));
		Mat color(Size(color_info.width, color_info.height), CV_8UC3, (void*)color_data.planes[0], color_data.pitches[0] / sizeof(uchar));

		IplImage *dst = 0;
		Mat dst3(color_info.height, color_info.width, CV_16UC1);
		ushort* p;
		dst = cvCreateImage(Size(depth_info.width, depth_info.height), 8, 1);
		short* depth__data = (short*)depth_data.planes[0];
		int dpitch = depth_data.pitches[0] / sizeof(ushort);
		for (int y = 0; y < depth_info.height; y++)
		{
			for (int x = 0; x < depth_info.width; x++)
			{
				//dst->imageData[y * depth_info.height + x] = depth__data[y * depth_info.height + x];
				ushort d = depth__data[y*dpitch + x];

				p = dst3.ptr<ushort>(y);

				//距离在0.2m至1.2m之间

				if (d > 0)

					p[x] = 255 - 0.255*(d - 200);

				else

					p[x] = 0;
			}
		}

		//imshow("dst3", dst3);
		/*Mat output;*/

		dst3.convertTo(output, CV_8UC1, 1);

		imshow("depth picture", output);

		for (int y = 0; y < depth_info.width; y++)
		{
			for (int x = 0; x < depth_info.height; x++)
			{
				dst->imageData[y * depth_info.height + x] = depth__data[y * depth_info.height + x];
			}
		}
		Mat dst_;
		dst_ = cvarrToMat(dst);
		//imshow("dst", dst_);

		depthIm->ReleaseAccess(&depth_data);
		colorIm->ReleaseAccess(&color_data);

		//if (!renderColor->RenderFrame(colorIm)) break;
		//if (!renderDepth->RenderFrame(depthIm)) break;

		psm->ReleaseFrame();

		string Img_Name = "image" + to_string(k) + ".jpg";
		string Img_Name_Deep = "deep" + to_string(k) + ".jpg";

		imshow("color", color);

		if (k == 20)
		{
			imwrite(Img_Name, color);
			imwrite(Img_Name_Deep, output);
			return 0;
		}
		cv::destroyWindow("color");

		k++;
	}
	psm->Release();
}
void compare()
{
	//读取识别结果
	fstream in("file.txt");//识别结果路径
	vector<string> obj;
	vector<int> loc;
	string cur;
	int locx_min, locy_min, locx_max, locy_max;
	int co = 0;
	while (!in.eof())
	{
		in >> cur;
		in >> locx_min;
		in >> locy_min;
		in >> locx_max;
		in >> locy_max;
		obj.push_back(cur);
		loc.push_back(locx_min);
		loc.push_back(locy_min);
		loc.push_back(locx_max);
		loc.push_back(locy_max);
		co++;
		cout << "cur:" << cur << "  locx:" << locx_min << "  locy:" << locy_min << "  locx:" << locx_max << "  locy:" << locy_max << endl;
	}
	//检查物体是否真实存在，删去平面上的物体
	auto it = obj.begin();
	//int n = 0;
	//while (it < obj.end())
	//{
	//	if (!check(loc[4 * n], loc[4 * n + 1], loc[4 * n + 2], loc[4 * n + 3]))//这里的参数可能需要改（指向被删除点云和深度图的指针）
	//		obj.erase(it);
	//	else
	//		it++;
	//	n++;
	//}
	//for (auto i = obj.begin(); i < obj.end(); i++)
	//	cout << "name:" << *i << endl;
	//整理
	sort(obj.begin(), obj.end());
	map<string, int>ans;
	it = obj.begin();
	while (it < obj.end())
	{
		ans[*it]++;
		it++;
	}
	//输出
	auto map_it = ans.begin();
	while (map_it != ans.end())
	{
		cout << "Goal_ID=" << map_it->first << ";";
		cout << "Num=" << map_it->second << endl;
		++map_it;
	}

}
void discrimination()
{
	Py_Initialize();

	PyRun_SimpleString("import sys");

	PyRun_SimpleString("sys.path.append('./')");

	PyObject * pModule = NULL; //shengmingbianliang

	pModule = PyImport_ImportModule("demo1");

	system("pause");

	Py_Finalize();
}
int main()
{
	float time1 = 0, time2 = 0, time = 0;
	time1 = cv::getTickCount();

	camera();
	waitKey(10);

	//识别
	discrimination();

	//判断
	compare();

	//计时
	time2 = cv::getTickCount();
	time = (time2 - time1) / double(cv::getTickFrequency());
	cout << "time:" << time << endl;
	system("pause");

	return 0;

}
#endif