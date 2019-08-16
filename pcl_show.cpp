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

	/* �����Ӵ����󣬲�������������һ������"3D viewer"��viewer������Ϊboost::shared_ptrֻ�ܹ���ָ�룬

	 * ���������Ա�֤��ָ��������������ȫ��ʹ�ã����������ڴ����

	 */

	boost::shared_ptr<visualization::PCLVisualizer> viewer(new visualization::PCLVisualizer("3D viewer"));

	/*���ô���viewer�ı�����ɫ*/

	viewer->setBackgroundColor(0, 0, 0);

	/*������������ӵ��Ӵ��У���Ϊ�䶨��һ��Ψһ���ַ�����ΪID�ţ����ô�ID�ű�֤������Ա����Ҳ�ܱ�ʾ�õ��ơ�

	 * ��ε���addPointCloud()����ʵ�ֶ�����Ƶĵ��ӣ�ÿ����һ�ξʹ���һ���µ�ID�š������Ҫ����һ���Ѿ�

	 * ��ʾ�ĵ��ƣ��û������ȵ���removePointCloud()�����ṩ�µ�ID�š�����PCL1.1�汾֮��ֱ�ӵ���updatePointCloud()

	 * �Ϳ����ˣ������ֶ�����removePointCloud()�Ϳ�ʵ�ֵ��Ƹ��£�

	 */

	viewer->addPointCloud<PointXYZ>(cloud, "sample cloud");

	/*�޸���ʵ���Ƶĳߴ硣�û���ͨ���÷������Ƶ������Ӵ��е���ʾ��ʽ*/

	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");

	/*����XYZ����������Ĵ�С�ͳ��ȣ���ֵҲ����ȱʡ

	 *�鿴���ӵĵ���ͼ������û�û�з���У�Ϊ�����û�������ȷ�ķ����жϣ���Ҫ��ʾ�����ᡣ����������X��R����ɫ��

	 * Y��G����ɫ��Z��B����ɫ���ֱ������ֲ�ͬ��ɫ��Բ������档

	 */

	viewer->addCoordinateSystem(1.0);

	/*ͨ����������������û���Ĭ�ϵĽǶȺͷ���۲��*/

	viewer->initCameraParameters();



	/*��whileѭ�����ִ���һֱ���ڴ�״̬�����Ұ��չ涨ʱ��ˢ�´��ڡ�

	 * wasStopped()�ж���ʾ�����Ƿ��Ѿ����رգ�spinOnce()����Ϣ�ص�������������ʵ�����ø�����Ļ��ʱ��

	 * this_thread::sleep()���س��е���sleep()����Ǹ���һ���֪����仰������

	 */

	while (!viewer->wasStopped()) {

		viewer->spinOnce(100);

		boost::this_thread::sleep(boost::posix_time::microseconds(1000));

	}



	return 0;

}
#endif