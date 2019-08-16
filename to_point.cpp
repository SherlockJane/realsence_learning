//#define to_point
#ifdef to_point
//#include <QtCore/QCoreApplication>

#include <iostream>


#include <string>

using namespace std;



// OpenCV 库

#include <opencv2/core/core.hpp>

#include <opencv2/highgui/highgui.hpp>



// PCL 库

#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>



// 定义点云类型

typedef pcl::PointXYZRGBA PointT;

typedef pcl::PointCloud<PointT> PointCloud;

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

int main()
{
	printf("1");
	//QCoreApplication a(argc, argv);

	// 读取./data/rgb.png和./data/depth.png，并转化为点云



	// 图像矩阵
	
	cv::Mat rgb, depth;
	
	// 使用cv::imread()来读取图像
	

	rgb = cv::imread("color86.jpg");

	// rgb 图像是8UC3的彩色图像

	// depth 是16UC1的单通道图像，注意flags设置-1,表示读取原始数据不做任何修改

	depth = cv::imread("deep86.jpg", -1);
	int channels = depth.channels();
	printf("w,h:%d,%d,%d\n", depth.rows, depth.cols,channels);//480,640
	system("pause");
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

		for (int n = 0; n < depth.cols; n++)

		{

			// 获取深度图中(m,n)处的值

			uchar d = depth.ptr<uchar>(m)[n];
			//printf("d:%d\n", d);

			// d 可能没有值，若如此，跳过此点

			if (d == 0)

				continue;

			// d 存在值，则向点云增加一个点

			PointT p;



			// 计算这个点的空间坐标/ camera_factor

			p.z = double(d) ;
			p.x = (n - camera_cx) * p.z / camera_fx;
			p.y = (m - camera_cy) * p.z / camera_fy;
			
			qaq++;
			


			// 从rgb图像中获取它的颜色

			// rgb是三通道的BGR格式图，所以按下面的顺序获取颜色

			p.b = rgb.ptr<uchar>(m)[n * 3];

			p.g = rgb.ptr<uchar>(m)[n * 3 + 1];

			p.r = rgb.ptr<uchar>(m)[n * 3 + 2];



			// 把p加入到点云中

			cloud->points.push_back(p);
			
			//system("pause");

		}

	// 设置并保存点云

	cloud->height = 1;

	cloud->width = cloud->points.size();

	cout << "point cloud size = " << cloud->points.size() << endl;

	cloud->is_dense = false;

	pcl::io::savePCDFile("151.pcd", *cloud);
	system("pause");
	//转化回2D
	int k = cloud->points.size();
	uchar d_=0;
	float m_=0.0, n_=0.0;
	int mm=0, nn=0;
	//int array_[640*480]= { 0 };
	for (int i = 0; i < k; i++)
	{
		
		d_ = uchar(cloud->points[i].z);
		//array_[nn][mm] = d_;*camera_factor
		//printf("cloud->points[0].z:%f\n", cloud->points[i].x);
		//printf("m_:%f\n", m_);
		if (cloud->points[i].z != 0)
		{
			m_ = cloud->points[i].y*camera_fy/ cloud->points[i].z + camera_cy;
			n_ = cloud->points[i].x*camera_fx/ cloud->points[i].z + camera_cx;
			mm = int(m_);
			nn = int(n_);
			if (d_ < 0)
				d_ = 0;
			if (d_ > 255)
				d_ = 255;
			
			//if (mm >= 0 && mm < 480 && nn >= 0 && nn < 640 )
			{
				reverse.at<uchar>(mm,nn) = d_;
				/*cvWaitKey(5);
				imshow("reverse", reverse);*/
				printf("d_:%d\n", d_);
			}
		}		
	}
	imshow("reverse", reverse);
	cvWaitKey(200);
	//uchar* Data_;
	//for (int i = 0; i < depth.rows; i++)
	//{
	//	Data_ = reverse.ptr<uchar>(i);
	//	for (int j = 0; j < depth.cols; j++)
	//	{
	//		Data_[j] = array_[i][j];
	//	}
	//}
	//imshow("reverse", reverse);
	system("pause");
	// 清除数据并退出

	cloud->points.clear();

	cout << "Point cloud saved." << endl;

	return 0;

}
#endif
