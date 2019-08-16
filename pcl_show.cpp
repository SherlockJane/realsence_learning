//#define pcl_show
#ifdef pcl_show

#include <iostream>

#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>



using namespace std;

using namespace pcl;

using namespace io;



int main() {

	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);



	if (io::loadPCDFile("15.pcd", *cloud) == -1) {

		cerr << "can't read file rabbit.pcd" << endl;

		return -1;

	}

	/* 创建视窗对象，并给标题栏定义一个名称"3D viewer"。viewer的类型为boost::shared_ptr只能共享指针，

	 * 这样做可以保证该指针在整个程序中全局使用，而不引起内存错误。

	 */

	boost::shared_ptr<visualization::PCLVisualizer> viewer(new visualization::PCLVisualizer("3D viewer"));

	/*设置窗口viewer的背景颜色*/

	viewer->setBackgroundColor(0, 0, 0);

	/*将点云数据添加到视窗中，并为其定义一个唯一的字符串作为ID号，利用此ID号保证其他成员方法也能表示该点云。

	 * 多次调用addPointCloud()可以实现多个点云的叠加，每调用一次就创建一个新的ID号。如果想要更新一个已经

	 * 显示的点云，用户必须先调用removePointCloud()，并提供新的ID号。（在PCL1.1版本之后直接调用updatePointCloud()

	 * 就可以了，不必手动调用removePointCloud()就可实现点云更新）

	 */

	viewer->addPointCloud<PointXYZ>(cloud, "sample cloud");

	/*修改现实点云的尺寸。用户可通过该方法控制点云在视窗中的显示方式*/

	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");

	/*设置XYZ三个坐标轴的大小和长度，该值也可以缺省

	 *查看复杂的点云图像会让用户没有方向感，为了让用户保持正确的方向判断，需要显示坐标轴。三个坐标轴X（R，红色）

	 * Y（G，绿色）Z（B，蓝色）分别用三种不同颜色的圆柱体代替。

	 */

	viewer->addCoordinateSystem(1.0);

	/*通过设置相机参数是用户从默认的角度和方向观察点*/

	viewer->initCameraParameters();



	/*此while循环保持窗口一直处于打开状态，并且按照规定时间刷新窗口。

	 * wasStopped()判断显示窗口是否已经被关闭，spinOnce()叫消息回调函数，作用其实是设置更新屏幕的时间

	 * this_thread::sleep()在县城中调用sleep()。抱歉，我还不知道这句话的作用

	 */

	while (!viewer->wasStopped()) {

		viewer->spinOnce(100);

		boost::this_thread::sleep(boost::posix_time::microseconds(1000));

	}



	return 0;

}
#endif