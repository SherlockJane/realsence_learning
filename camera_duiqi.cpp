#define camera 
#ifdef camera 
#include <iostream>
using namespace std;

#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include<string>

#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
using namespace cv;

#include<librealsense2/rs.hpp>
#include<librealsense2/rsutil.h>

//��ȡ������ض�Ӧ���ȵ�λ���ף��Ļ������
float get_depth_scale(rs2::device dev)
{
	// Go over the device's sensors
	for (rs2::sensor& sensor : dev.query_sensors())
	{
		// Check if the sensor if a depth sensor
		if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
		{
			return dpt.get_depth_scale();
		}
	}
	throw std::runtime_error("Device does not have a depth sensor");
}

//���ͼ���뵽��ɫͼ����
Mat align_Depth2Color(Mat depth, Mat color, rs2::pipeline_profile profile) 
{
	//����������
	auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
	auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
	//��ȡ�ڲ�
	const auto intrinDepth = depth_stream.get_intrinsics();
	const auto intrinColor = color_stream.get_intrinsics();
	//ֱ�ӻ�ȡ���������ͷ����ϵ����ɫ����ͷ����ϵ��ŷʽ�任����
	//auto  extrinDepth2Color=depth_stream.get_extrinsics_to(color_stream);
	rs2_extrinsics  extrinDepth2Color;
	rs2_error *error;
	rs2_get_extrinsics(depth_stream, color_stream, &extrinDepth2Color, &error);
	//ƽ��㶨��
	float pd_uv[2], pc_uv[2];
	//�ռ�㶨��
	float Pdc3[3], Pcc3[3];
	//��ȡ�����������ʵ��λ������D435Ĭ��1���ף�SR300Ϊ0.125��
	float depth_scale = get_depth_scale(profile.get_device());
	int y = 0, x = 0;
	//��ʼ�����
	//Mat result=Mat(color.rows,color.cols,CV_8UC3,Scalar(0,0,0));
	Mat result = Mat(color.rows, color.cols, CV_16U, Scalar(0));
	//�����ͼ�����
	for (int row = 0; row < depth.rows; row++) 
	{
		for (int col = 0; col < depth.cols; col++) 
		{
			//����ǰ��(x,y)��������pd_uv����ʾ��ǰ���ͼ�ĵ�
			pd_uv[0] = col;
			pd_uv[1] = row;
			//ȡ��ǰ���Ӧ�����ֵ
			uint16_t depth_value = depth.at<uint16_t>(row, col);
			//���㵽��
			float depth_m = depth_value * depth_scale;
			//�����ͼ�����ص�����ڲ�ת�����������ͷ����ϵ�µ���ά��
			rs2_deproject_pixel_to_point(Pdc3, &intrinDepth, pd_uv, depth_m);
			//���������ͷ����ϵ����ά��ת������ɫ����ͷ����ϵ��
			rs2_transform_point_to_point(Pcc3, &extrinDepth2Color, Pdc3);
			//����ɫ����ͷ����ϵ�µ������ά��ӳ�䵽��άƽ����
			rs2_project_point_to_pixel(pc_uv, &intrinColor, Pcc3);
			//ȡ��ӳ���ģ�u,v)
			x = (int)pc_uv[0];
			y = (int)pc_uv[1];
			if(x<0||x>color.cols)
			    continue;
			if(y<0||y>color.rows)
			    continue;
			//��ֵ�޶�
			x = x < 0 ? 0 : x;
			x = x > depth.cols - 1 ? depth.cols - 1 : x;
			y = y < 0 ? 0 : y;
			y = y > depth.rows - 1 ? depth.rows - 1 : y;
			result.at<uint16_t>(y, x) = depth_value;
		}
	}
	//����һ�����ɫͼ�����˵������Ϣͼ��
	return result;
}



void measure_distance(Mat &color, Mat depth, cv::Size range, rs2::pipeline_profile profile)

{

	//��ȡ�����������ʵ��λ������D435Ĭ��1���ף�

	float depth_scale = get_depth_scale(profile.get_device());

	//����ͼ�����ĵ�

	cv::Point center(color.cols / 2, color.rows / 2);

	//����������ķ�Χ

	cv::Rect RectRange(center.x - range.width / 2, center.y - range.height / 2, range.width, range.height);

	//�����÷�Χ

	float distance_sum = 0;

	int effective_pixel = 0;

	for (int y = RectRange.y; y < RectRange.y + RectRange.height; y++) {

		for (int x = RectRange.x; x < RectRange.x + RectRange.width; x++) {

			//������ͼ�¸õ����ز�Ϊ0����ʾ�о�����Ϣ

			if (depth.at<uint16_t>(y, x)) {

				distance_sum += depth_scale * depth.at<uint16_t>(y, x);

				effective_pixel++;

			}

		}

	}

	cout << "������ɣ���Ч���ص�:" << effective_pixel << endl;

	float effective_distance = distance_sum / effective_pixel;

	cout << "Ŀ����룺" << effective_distance << " m" << endl;

	char distance_str[30];

	sprintf_s(distance_str, "the distance is:%f m", effective_distance);

	cv::rectangle(color, RectRange, Scalar(0, 0, 255), 2, 8);

	cv::putText(color, (string)distance_str, cv::Point(color.cols*0.02, color.rows*0.05),

		cv::FONT_HERSHEY_PLAIN, 2, Scalar(0, 255, 0), 2, 8);

}



int main()
{
	const char* depth_win = "depth_Image";
	namedWindow(depth_win, WINDOW_AUTOSIZE);
	const char* color_win = "color_Image";
	namedWindow(color_win, WINDOW_AUTOSIZE);

	//���ͼ����ɫmap
	rs2::colorizer c;                          // Helper to colorize depth images

	//�������ݹܵ�
	rs2::pipeline pipe;
	rs2::config pipe_config;
	pipe_config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	pipe_config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);



	//start()�����������ݹܵ���profile

	rs2::pipeline_profile profile = pipe.start(pipe_config);



	//����һ������ȥת����ȵ�����

	float depth_clipping_distance = 1.f;



	//����������

	auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();

	auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();



	//��ȡ�ڲ�

	auto intrinDepth = depth_stream.get_intrinsics();

	auto intrinColor = color_stream.get_intrinsics();



	//ֱ�ӻ�ȡ���������ͷ����ϵ����ɫ����ͷ����ϵ��ŷʽ�任����

	auto  extrinDepth2Color = depth_stream.get_extrinsics_to(color_stream);



	while (cvGetWindowHandle(depth_win) && cvGetWindowHandle(color_win)) // Application still alive?

	{

		//��������ֱ���µ�һ֡����

		rs2::frameset frameset = pipe.wait_for_frames();

		//ȡ���ͼ�Ͳ�ɫͼ

		rs2::frame color_frame = frameset.get_color_frame();//processed.first(align_to);

		rs2::frame depth_frame = frameset.get_depth_frame();

		rs2::frame depth_frame_4_show = frameset.get_depth_frame().apply_filter(c);

		//��ȡ���

		const int depth_w = depth_frame.as<rs2::video_frame>().get_width();

		const int depth_h = depth_frame.as<rs2::video_frame>().get_height();

		const int color_w = color_frame.as<rs2::video_frame>().get_width();

		const int color_h = color_frame.as<rs2::video_frame>().get_height();



		//����OPENCV���� ����������

		Mat depth_image(Size(depth_w, depth_h),

			CV_16U, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

		Mat depth_image_4_show(Size(depth_w, depth_h),

			CV_8UC3, (void*)depth_frame_4_show.get_data(), Mat::AUTO_STEP);

		Mat color_image(Size(color_w, color_h),

			CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

		//ʵ�����ͼ���뵽��ɫͼ

		Mat result = align_Depth2Color(depth_image, color_image, profile);

		measure_distance(color_image, result, cv::Size(20, 20), profile);

		//��ʾ

		imshow(depth_win, depth_image_4_show);

		imshow(color_win, color_image);

		//imshow("result",result);

		waitKey(1);

	}

	return 0;

}
#endif

