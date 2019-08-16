//#define llj_
#ifdef llj_
//https://blog.csdn.net/qq_18941713/article/details/84647887//原理
#include <iostream>
#include <string>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include<opencv2/imgproc.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl\visualization\cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

using namespace std;
using namespace cv;

// 定义点云类型
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参
const double camera_factor = 1000;
const double camera_cx = 344.1270;
const double camera_cy = 247.7587;
const double camera_fx = 474.055;
const double camera_fy = 474.055;

int main()
{
	// 读取深度图，并转化为点云
	Mat  depth;		 // 图像矩阵
	// depth 是16UC1的单通道图像，注意flags设置-1,表示读取原始数据不做任何修改
	depth = cv::imread("652.jpg", 0);
	// 点云变量
	// 使用智能指针，创建一个空点云。这种指针用完会自动释放。
	pcl::PCDReader reader;
	PointCloud::Ptr cloud(new PointCloud);
	reader.read("151.pcd", *cloud);
	
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(5);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);
	//取出平面里的点
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	//extract.filter(*cloud_filtered);
	// 提取除地面外的物体
	extract.setNegative(true);
	extract.filter(*cloud_filtered);
	seg.setInputCloud(cloud_filtered);
	seg.segment(*inliers, *coefficients);
	
	//显示点云

	int v1(0);
	int v2(1);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0); // green
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_(cloud_filtered, 255, 0, 0); // green
	viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, single_color_, "sample cloud_filtered", v1);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud", v2);
	viewer->addCoordinateSystem(1.0);
	viewer->spin();

	//点云转换回深度图
	Mat result(depth.size(), CV_8UC1);
	uchar* Data_;
	for (int i = 0; i < depth.rows; i++)
	{
		Data_ = result.ptr<uchar>(i);
		for (int j = 0; j < depth.cols; j++)
		{
			Data_[j] = 0;
		}
	}
	cout << depth.size() << endl;
	cout << result.size() << endl;
	int k = cloud_filtered->points.size();
	cout << k;
	int i = 0;
	for (; i < k; i++)
	{
		result.at<uchar>((cloud_filtered->points[i].y) / 100, (cloud_filtered->points[i].x) / 50) = (cloud_filtered->points[i].z);
		imshow("11", result);
		waitKey(5);
	}
	Mat element = getStructuringElement(MORPH_RECT, Size(10, 10));
	dilate(result, result, element);
	
	//显示
	imshow("11", result);
	waitKey();
	// 清除数据并退出
	cloud->points.clear();
	cout << "Point cloud saved." << endl;
	return 0;

}
#endif
//#define llj
#ifdef llj
//https://blog.csdn.net/qq_18941713/article/details/84647887//原理
#include <iostream>
#include <string>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include<opencv2/imgproc.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl\visualization\cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


using namespace std;
using namespace cv;
using namespace pcl;
using namespace io;
// 定义点云类型
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参
const double camera_factor = 1000;
const double camera_cx = 344.1270;
const double camera_cy = 247.7587;
const double camera_fx = 474.055;
const double camera_fy = 474.055;

int main()
{

		// 读取深度图，并转化为点云
		Mat  depth;		 // 图像矩阵
		// depth 是16UC1的单通道图像，注意flags设置-1,表示读取原始数据不做任何修改
		depth = cv::imread("deep.png", 0);
		// 点云变量
		// 使用智能指针，创建一个空点云。这种指针用完会自动释放。
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		 //遍历深度图
		for (int m = 0; m < depth.rows; m++)
		{
			for (int n = 0; n < depth.cols/2; n++)
			{
				// 获取深度图中(m,n)处的值
				ushort d = depth.ptr<ushort>(m)[n];
				// d 可能没有值，若如此，跳过此点
				if (d == 0)
					continue;
				// d 存在值，则向点云增加一个点
				PointT p;

				// 计算这个点的空间坐标
				/*
				p.z = double(d) / camera_factor;
				p.x = (n - camera_cx) * p.z / camera_fx;
				p.y = (m - camera_cy) * p.z / camera_fy;
				*/ 
				p.z = d ;
				p.x = n * 100;
				p.y = m * 100;
				/*
				// 从rgb图像中获取它的颜色
				// rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
				p.b = rgb.ptr<uchar>(m)[n * 3];
				p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
				p.r = rgb.ptr<uchar>(m)[n * 3 + 2];
				*/
				// 把p加入到点云中
				cloud->points.push_back(p);
			}
		}
		if (io::loadPCDFile("15.pcd", *cloud) == -1) {

			cerr << "can't read file rabbit.pcd" << endl;

			return -1;

		}
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients(true);
		// Mandatory
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(350);
		seg.setInputCloud(cloud);//导入原图
		seg.segment(*inliers, *coefficients);//寻找平面，由inliers记录信息
		//取出平面里的点
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);//收集除平面外的点
		pcl::ExtractIndices<pcl::PointXYZ> extract;//收集平面里的点
		extract.setInputCloud(cloud);//导入原图
		extract.setIndices(inliers);//处理成只剩平面
		//extract.filter(*cloud_filtered);
		// 提取除地面外的物体
		extract.setNegative(true);
		extract.filter(*cloud_filtered);
		
		//显示点云
		int v1(0);
		int v2(1);
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0); // green 原图
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_(cloud_filtered, 255, 0, 0); // red 处理后的
		viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, single_color_, "sample cloud_filtered",v1);
		viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud",v2);
		viewer->addCoordinateSystem(1.0);
		viewer->spin();
		
		//点云转换回深度图
		Mat result(depth.size(),CV_8UC1);
		cout << depth.size() << endl;
		cout << result.size() << endl;
		int k = cloud_filtered->points.size();
		cout << k;
		int i = 0;
		for (; i < k; i++)
		{
				result.at<uchar>((cloud_filtered->points[i].y) / 100, (cloud_filtered->points[i].x) / 50) = (cloud_filtered->points[i].z);
		}
		Mat element =getStructuringElement(MORPH_RECT, Size(5, 9));
		morphologyEx(result, result, MORPH_OPEN, element);
		//threshold(result, result,200, 255, THRESH_BINARY_INV);
		/*		// 设置并保存点云
		cloud->height = 1;
		cloud->width = cloud->points.size();
		cout << "point cloud size = " << cloud->points.size() << endl;
		cloud->is_dense = false;
		*/
		//pcl::io::savePCDFile("test.pcd", *cloud);
		//显示
		imshow("11", result);
		waitKey(20);
		system("pause");
		// 清除数据并退出
		cloud->points.clear();
		cout << "Point cloud saved." << endl;
		return 0;
		
}
#endif //llj