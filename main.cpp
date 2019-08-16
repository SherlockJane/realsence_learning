//#define main_
#ifdef main_
#pragma comment(lib, "legacy_stdio_definitions.lib")
#include "windows.h"
#include <iostream>
#include <string>
#include<Python.h>

//camera
#include "pxcsensemanager.h"
#include <pxcsession.h>    
#include "util_render.h"   

// OpenCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl\visualization\cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

using namespace std;
using namespace cv;

#define WIDTH 640    
#define HEIGHT 480  
 
// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

typedef pcl::PointXYZ PointT_next;
typedef pcl::PointCloud<PointT_next> PointCloud_next;

//相机参数
/*     
color intrinsics: 640  480  308.783  237.412  620.252  620.252
coeffs: 0  0  0  0  0
distortion model: None

depth intrinsics: 640  480  311.704  245.82  474.055  474.055
coeffs: 0.125268  0.107347  0.00599028  0.00498074  0.0363283
distortion model: Inverse Brown Conrady
*/

// 相机内参

const double camera_factor = 1000;
const double camera_cx = 311.704;
const double camera_cy = 245.82;
const double camera_fx = 474.055;
const double camera_fy = 474.055;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
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

void deep_to_point()
{
	// 图像矩阵
	cv::Mat rgb, depth;
	// 使用cv::imread()来读取图像

	// rgb 图像是8UC3的彩色图像
	// depth 是16UC1的单通道图像，注意flags设置-1,表示读取原始数据不做任何修改
	depth = cv::imread("deep20.jpg", -1);
	int channels = depth.channels();
	printf("w,h:%d,%d,%d\n", depth.rows, depth.cols, channels);//480,640

	cv::Mat reverse(depth.rows, depth.cols, CV_8UC1);
	uchar* Data_;
	for (int i = 0; i < depth.rows; i++)
	{
		Data_ = reverse.ptr<uchar>(i);
		for (int j = 0; j < depth.cols; j++)
		{
			Data_[j] = 0;
		}
	}

	// 点云变量
	// 使用智能指针，创建一个空点云。这种指针用完会自动释放。
	PointCloud::Ptr cloud(new PointCloud);
	// 遍历深度图
	int qaq = 0;
	for (int m = 0; m < depth.rows; m++)
	{
		for (int n = 0; n < depth.cols; n++)
		{
			// 获取深度图中(m,n)处的值
			uchar d = depth.ptr<uchar>(m)[n];
			// d 可能没有值，若如此，跳过此点
			if (d == 0)
				continue;
			// d 存在值，则向点云增加一个点
			PointT p;
			// 计算这个点的空间坐标/ camera_factor
			p.z = double(d);
			p.x = (n - camera_cx) * p.z / camera_fx;
			p.y = (m - camera_cy) * p.z / camera_fy;
			// 从rgb图像中获取它的颜色
			// rgb是三通道的BGR格式图，所以按下面的顺序获取颜色

			// 把p加入到点云中
			cloud->points.push_back(p);
		}
	}

	// 设置并保存点云
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cout << "point cloud size = " << cloud->points.size() << endl;
	cloud->is_dense = false;
	pcl::io::savePCDFile("15.pcd", *cloud);
	//system("pause");

	//转化回2D
	/*
	int k = cloud->points.size();
	uchar d_ = 0;
	float m_ = 0.0, n_ = 0.0;
	int mm = 0, nn = 0;
	for (int i = 0; i < k; i++)
	{
		d_ = uchar(cloud->points[i].z);
		if (cloud->points[i].z != 0)
		{
			m_ = cloud->points[i].y*camera_fy / cloud->points[i].z + camera_cy;
			n_ = cloud->points[i].x*camera_fx / cloud->points[i].z + camera_cx;
			mm = int(m_);
			nn = int(n_);
			if (d_ < 0)
				d_ = 0;
			if (d_ > 255)
				d_ = 255;
			//if (mm >= 0 && mm < 480 && nn >= 0 && nn < 640 )
			{
				reverse.at<uchar>(mm, nn) = d_;
				printf("d_:%d\n", d_);
			}
		}
	}
	imshow("reverse", reverse);
	cvWaitKey(10);
	system("pause");
	*/
	// 清除数据并退出
	cloud->points.clear();
	cout << "Point cloud saved." << endl;
}

void pcl_delicate()
{
	pcl::PCDReader reader;
	PointCloud_next::Ptr cloud_next(new PointCloud_next);
	reader.read("15.pcd", *cloud_next);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(1);
	seg.setInputCloud(cloud_next);
	seg.segment(*inliers, *coefficients);
	//取出平面里的点
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);*/
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud_next);
	extract.setIndices(inliers);
	//extract.filter(*cloud_filtered);

	// 提取除地面外的物体
	//extract.setNegative(true);
	extract.filter(*cloud_filtered);
	seg.setInputCloud(cloud_filtered);
	seg.segment(*inliers, *coefficients);

	//显示点云

	int v1(0);
	int v2(1);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_next, 0, 255, 0); // green
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_(cloud_filtered, 255, 0, 0); // green
	viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, single_color_, "sample cloud_filtered", v1);
	viewer->addPointCloud<pcl::PointXYZ>(cloud_next, single_color, "sample cloud", v2);
	viewer->addCoordinateSystem(1.0);
	viewer->spin();

	// 清除数据并退出
	cloud_next->points.clear();
	cout << "Point cloud_next dealed." << endl;

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

bool check(int x_min, int y_min, int x_max, int y_max)   //检查点是否在点云中
{

	PointT p;
	//计算方法
	int dealt_x = x_max - x_min;
	int dealt_y = y_max - y_min;
	vector<int>locx = { x_min + dealt_x / 2,x_min + dealt_x / 2,x_min + dealt_x / 3,x_min + dealt_x * 2 / 3 };
	vector<int>locy = { y_min + dealt_y / 2, y_min + dealt_y / 3,y_min + dealt_y / 2,y_min + dealt_y / 2 };
	int count = 0;
	for (int i = 0; i < 4; i++)
	{
		p.z = output.at<uchar>(locy[i], locx[i]);
		p.x = (locx[i] - camera_cx) * p.z / camera_fx;
		p.y = (locy[i] - camera_cy) * p.z / camera_fy;
		auto pp = cloud_filtered->begin();
		while (pp != cloud_filtered->end())
		{
			if (abs(p.z - pp->z) < 3)
			{
				if (abs(p.x - pp->x) < 3)
				{
					if (abs(p.y - pp->y) < 3)
					{
						count++;
						break;
					}
				}

			}
			pp++;
		}
	}
	if (count > 2)
		return false;
	else
		return true;
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
	int n = 0;
	while (it < obj.end())
	{
		if (!check(loc[4 * n], loc[4 * n + 1], loc[4 * n + 2], loc[4 * n + 3]))//这里的参数可能需要改（指向被删除点云和深度图的指针）
			obj.erase(it);
		else
			it++;
		n++;
	}
	for (auto i = obj.begin(); i < obj.end(); i++)
		cout << "name:" << *i << endl;
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

int main()
{
	float time1 = 0, time2 = 0, time = 0;
	time1 = cv::getTickCount();

	camera();
	waitKey(10);

	//将深度图转化为点云
	deep_to_point();

	//去平面处理
	pcl_delicate();

	//识别
	discrimination();

	//判断
	compare();

	//计时
	time2 = cv::getTickCount();
	time = (time2-time1) / double(cv::getTickFrequency());
	cout << "time:" << time << endl;
	system("pause");
	
	return 0;

}
#endif