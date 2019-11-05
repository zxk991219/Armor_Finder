#pragma once

// #ifdef USE_NEW_CODE
// #undef USE_NEW_CODE
// #endif

// #ifdef DEBUG
// #undef DEBUG
// #endif




// 开始define

// #define DEBUG //在程序中用 #ifdef DEBUG 与 #endif 将debug代码块框起来,实现debug输出 
// #define USE_NEW_CODE //如果保留这一行,则使用新代码; 如果注释掉这一行,则使用旧代码

// #define SHOW_LIGHT
// #define SHOW_ARMOR
#define SHOW_ARMOR_WHOLE

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "proportion_thresh.hpp"
#include <algorithm>

#ifdef USE_NEW_CODE //新代码在下面

namespace sp
{
	
float iou(const cv::Rect& lhs, const cv::Rect& rhs)
{
	const int lt_x = std::max(lhs.x, rhs.x);
	const int lt_y = std::max(lhs.y, rhs.y);
	const int rd_x = std::min(lhs.x + lhs.width, rhs.x + rhs.width);
	const int rd_y = std::min(lhs.y + lhs.height, rhs.y + rhs.height);
	
	const int inner_w = std::max(0, rd_x - lt_x + 1);
	const int inner_h = std::max(0, rd_y - lt_y + 1);
	const int inner_area = inner_h * inner_w;
	
	return static_cast<float>(inner_area) / (lhs.area() + rhs.area() - inner_area);
}

bool bboxes_light_is_ok(const cv::Mat& in, double max_val, double proportion, int thresh_value)
{
	cv::Mat in2;
	int rows = in.rows;
	int cols = in.cols;
	
	// cv::cvtColor(in, in, CV_RGB2GRAY);//不知道为什么在这里灰度图像会报错

	if(in.isContinuous())
	{
		cols *= rows;
		rows = 1;
	}
	std::vector<uchar> color_value(rows*cols);
	int pos = 0;
	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;j++)
		{
			color_value[pos++] = in.at<uchar>(i,j);
		}
	}
	std::nth_element(color_value.begin(), color_value.end()-rows*cols*proportion, color_value.end());
	auto thre_iterator = color_value.end()-rows*cols*proportion;
	uchar threshold = *thre_iterator;

	int threshold_int = (int)threshold;

	#ifdef DEBUG
	std::cout << "threshold=" << threshold_int << std::endl; //打印计算得出的threshold
	#endif

	if(threshold_int>=thresh_value) // 筛亮度
	{
		// hsv筛颜色
		cv::cvtColor(in, in2, CV_RGB2HSV);
		CvScalar scalar;

		// 获得bboxes_light的中点坐标和中点hsv亮度
		IplImage* ipl_in2 = cvCreateImage(cvSize(rows,cols), IPL_DEPTH_8U, 3);
		*ipl_in2 = IplImage(in2);
		scalar = cvGet2D(ipl_in2, rows/2, cols/2);

		if((int)scalar.val[1]>254
			// && (int)scalar.val[1]<30
			&& (int)scalar.val[2]>254)
		{
			#ifdef DEBUG
			std::cout << "饱和度" << (int)scalar.val[1] << std::endl;
			std::cout << "明度值" << (int)scalar.val[2] << std::endl;
			#endif

			return true;
		}
		else
		{
			#ifdef DEBUG
			std::cout << "未获取HSV" << std::endl;
			#endif
		}
		
		#ifdef DEBUG
		std::cout << "true" << std::endl;
		#endif
	}
	else 
	{
		return false;
		#ifdef DEBUG
		std::cout << "false" << std::endl;
		#endif
	}
	
}

bool bboxes_armor_isok(const cv::Rect& rect_l, const cv::Rect& rect_r)
{
	const double egde_l = std::min(rect_l.x, rect_r.x);
	const double egde_r = std::max(rect_l.x+rect_l.width, rect_r.x+rect_r.width);
	// const double egde_d = std::min(rect_l.y-rect_l.height, rect_r.y-rect_r.height);
	// const double egde_u = std::max(rect_l.y, rect_r.y);
	const double egde_u = std::max(rect_l.y+rect_l.height, rect_r.y+rect_r.height);
	const double egde_d = std::min(rect_l.y, rect_r.y);
	const double bbox_light_width = std::max(rect_l.width, rect_r.width);
	const double bbox_light_height = std::max(rect_l.height, rect_r.height);
	
	const double bbox_armor_width = egde_r - egde_l;
	const double bbox_armor_height = egde_u - egde_d;

	// 判断是否为装甲板
	
	// 添加装甲板和灯条框的高度相比：<1.2 && >0.9

	if
	(bbox_armor_height/bbox_armor_width<0.5
	&& bbox_armor_height/bbox_armor_width>0.2
	&& bbox_armor_height/bbox_light_height>0.9
	&& bbox_armor_height/bbox_light_height<1.5
	// && rect_r.width/rect_l.width>0.7
	// && rect_r.width/rect_l.width<1.3
	)
	{
		return true;
	}
	// else if (bbox_armor_width/bbox_armor_height<1.2 
	// && bbox_armor_width/bbox_armor_height>0.3)
	// {
	// 	return true;
	// }
	else return false;
}

cv::Rect get_armor(const cv::Rect& rect_l, const cv::Rect& rect_r)
{
	const int egde_l = std::min(rect_l.x, rect_r.x);
	const int egde_r = std::max(rect_l.x+rect_l.width, rect_r.x+rect_r.width);
	// const int egde_d = std::min(rect_l.y-rect_l.height, rect_r.y-rect_r.height);
	// const int egde_u = std::max(rect_l.y, rect_r.y);
	const double egde_u = std::max(rect_l.y+rect_l.height, rect_r.y+rect_r.height);
	const double egde_d = std::min(rect_l.y, rect_r.y);
	
	const int bbox_armor_width = egde_r - egde_l;
	const int bbox_armor_height = egde_u - egde_d;

	cv::Rect armor(egde_l, egde_d, bbox_armor_width, bbox_armor_height);
	return armor;
}

cv::Mat& mser(cv::Mat& mat, cv::Mat& mat_real)
{
	cv::Mat img_show;
	
	// cv::resize(mat, mat, {640, 480});
	
	double thresh_binar = 0.02; //二值化取thresh_binar最亮部分
	double bbox_proportion_thresh_max = 0.1; //设定bbox的宽高比上阈值
	double bbox_proportion_thresh_min = 0.0; //设定bbox的宽高比下阈值
	constexpr float thresh_iou = 0.8; //IOU的阈值
	int thresh_value = 250; // bboxes_light的色度阈值

	sp::proportion_thresh(mat, mat, 255, thresh_binar); //二值化图像


	auto mser = cv::MSER::create(	0.1, // _delta 灰度值的变化量
									40, //_min_area 检测到的组块⾯积的范围
									14400, //_max_area 检测到的组块⾯积的范围
									0.25, //_max_variation 最⼤的变化率
									0.2, // _min_diversity 稳定区域的最⼩变换量
									200, //  _max_evolution 对彩⾊图像的MSER检测
									1.01, // _area_threshold 
									0.003, // _min_margin 
									5 // _edge_blur_size 
									); 

	// auto mser = cv::MSER::create(	5, // _delta 灰度值的变化量
	// 								60, //_min_area 检测到的组块⾯积的范围
	// 								14400, //_max_area 检测到的组块⾯积的范围
	// 								0.25, //_max_variation 最⼤的变化率
	// 								0.2, // _min_diversity 稳定区域的最⼩变换量
	// 								200, // _max_evolution 对彩⾊图像的MSER检测
	// 								1.01, // _area_threshold 
	// 								0.003, // _min_margin 
	// 								5 //  _edge_blur_size 
	// 								); 
									
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Rect>               bboxes; // 所有图块矩形
	std::vector<cv::Rect>               bboxes_light; // 灯条矩形
	std::vector<cv::Rect>               bboxes_armor; // 灯条匹配矩形
	std::vector<cv::Rect>               bboxes_armor_selected; // 筛选后的灯条匹配矩形
	std::vector<cv::Rect>               bboxes_armor_whole; // 装甲板矩形


	mser->detectRegions(mat, contours, bboxes); // 检测边缘

	#ifdef DEBUG
	std::cout << "All: " << bboxes.size() << " bboxes" << '\n';
	#endif

	std::vector<int> drawed_rects;
	drawed_rects.reserve(bboxes.size() / 4);
	bboxes_light.reserve(drawed_rects.size());

	// 筛选bbox并将bbox画在图像上

	int cnt = 0;
	if (!bboxes.empty())
	{
		++cnt;

		//筛选bboxes
		if(bboxes.front().width/bboxes.front().height > bbox_proportion_thresh_max)
		{
			#ifdef DEBUG
			std::cout << "out1" << std::endl;
			#endif
		}
		else if(bboxes.front().width/bboxes.front().height < bbox_proportion_thresh_min)
		{
			#ifdef DEBUG
			std::cout << "out2" << std::endl;
			#endif
		}
		else
		{
			// 画灯条矩形
			#ifdef SHOW_LIGHT

			#ifdef DEBUG
			cv::rectangle(mat, bboxes.front(), {255}, 2);
			#endif

			cv::rectangle(mat_real, bboxes.front(), {0,255,0}, 2);
			#endif

			drawed_rects.push_back(0);
		}	
	}

	for (int i = 1; i < bboxes.size(); ++i)
	{
		bool skip = false;
		for (auto&& index : drawed_rects)
			if (skip = (sp::iou(bboxes[i], bboxes[index]) > thresh_iou))
				break;
		if (skip)
			continue;
		
		//筛选bboxes
		if(bboxes[i].width/bboxes[i].height > bbox_proportion_thresh_max)
		{
			#ifdef DEBUG
			std::cout << "out3" << std::endl;
			#endif
		}
		else if(bboxes[i].width/bboxes[i].height < bbox_proportion_thresh_min)
		{
			#ifdef DEBUG
			std::cout << "out4" << std::endl;
			#endif
		}
		else
		{
			cv::Mat imagePart=mat_real(bboxes[i]); //抠图

			if(bboxes_light_is_ok(imagePart, 255, thresh_binar, thresh_value))
			{
				#ifdef DEBUG
				cv::imshow("imagePart",imagePart);
				#endif

				// 画灯条矩形
				#ifdef SHOW_LIGHT

				#ifdef DEBUG				
				cv::rectangle(mat, bboxes[i], {255}, 2);
				#endif

				cv::rectangle(mat_real, bboxes[i], {0,255,0}, 2);
				#endif
				
				drawed_rects.push_back(i);
				++cnt;
				bboxes_light.push_back(bboxes[i]);
			}
		}		
	}

	for (int k = 0; k < bboxes_light.size(); k++)
	{
		bool skip_light = false;
		for (int l=0;l<bboxes_light.size();l++)
			if (skip_light = sp::bboxes_armor_isok(bboxes_light[k], bboxes_light[l]) && l!=k)// 筛选条件
			{
				bboxes_armor.push_back(sp::get_armor(bboxes_light[k], bboxes_light[l]));
			}
	}

	// bboxes_armor去重
	for (int m = 0; m < bboxes_armor.size(); m++)
	{
		bool skip = false;
		for (int n=m;n<bboxes_armor.size();n++)
			if (skip = (sp::iou(bboxes_armor[m], bboxes_armor[n]) > thresh_iou) && m!=n)
				break;
		if (skip)
			continue;
		bboxes_armor_selected.push_back(bboxes_armor[m]);

		#ifdef DEBUG
		std::cout<<"pushed";
		#endif
	}

	for (int p=0;p<bboxes_armor_selected.size();p++)
	{
		// 画装甲板矩形
		#ifdef SHOW_ARMOR

		#ifdef DEBUG
		cv::rectangle(mat, bboxes_armor_selected[p], {255}, 2);
		#endif

		cv::rectangle(mat_real, bboxes_armor_selected[p], {0,0,255}, 2);
		#endif
	}

	for (int i = 0; i < bboxes_armor_selected.size(); i++)
	{
		int rect_armor_width = bboxes_armor_selected[i].width;
		int rect_armor_height = bboxes_armor_selected[i].width*0.6;
		int rect_armor_x = bboxes_armor_selected[i].x;
		int rect_armor_y = (bboxes_armor_selected[i].y+bboxes_armor_selected[i].height/2.0)-rect_armor_height/2.0;
		cv::Rect rect_armor(rect_armor_x, rect_armor_y, rect_armor_width, rect_armor_height);

		#ifdef SHOW_ARMOR_WHOLE

		#ifdef DEBUG
		cv::rectangle(mat, rect_armor, {255}, 2);
		#endif

		cv::rectangle(mat_real, rect_armor, {255,0,0}, 2);
		#endif
	}
	

	#ifdef DEBUG
	std::string cnt_str = std::to_string(cnt);	
	std::cout << "Answer: "+cnt_str+" bboxes" << '\n';
	#endif

	return mat;
}

}













#else //旧代码在下面

namespace sp
{
	
float iou(const cv::Rect& lhs, const cv::Rect& rhs)
{
	const int lt_x = std::max(lhs.x, rhs.x);
	const int lt_y = std::max(lhs.y, rhs.y);
	const int rd_x = std::min(lhs.x + lhs.width, rhs.x + rhs.width);
	const int rd_y = std::min(lhs.y + lhs.height, rhs.y + rhs.height);
	
	const int inner_w = std::max(0, rd_x - lt_x + 1);
	const int inner_h = std::max(0, rd_y - lt_y + 1);
	const int inner_area = inner_h * inner_w;
	
	return static_cast<float>(inner_area) / (lhs.area() + rhs.area() - inner_area);
}

bool bboxes_light_is_ok(const cv::Mat& in, double max_val, double proportion, int thresh_value)
{
	cv::Mat in2;
	int rows = in.rows;
	int cols = in.cols;
	
	// cv::cvtColor(in, in, CV_RGB2GRAY);//不知道为什么在这里灰度图像会报错

	if(in.isContinuous())
	{
		cols *= rows;
		rows = 1;
	}
	std::vector<uchar> color_value(rows*cols);
	int pos = 0;
	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;j++)
		{
			color_value[pos++] = in.at<uchar>(i,j);
		}
	}
	std::nth_element(color_value.begin(), color_value.end()-rows*cols*proportion, color_value.end());
	auto thre_iterator = color_value.end()-rows*cols*proportion;
	uchar threshold = *thre_iterator;

	int threshold_int = (int)threshold;

	#ifdef DEBUG
	std::cout << "threshold=" << threshold_int << std::endl; //打印计算得出的threshold
	#endif

	if(threshold_int>=thresh_value) // 筛亮度
	{
		// hsv筛颜色
		cv::cvtColor(in, in2, CV_RGB2HSV);
		CvScalar scalar;

		// 获得bboxes_light的中点坐标和中点hsv亮度
		IplImage* ipl_in2 = cvCreateImage(cvSize(rows,cols), IPL_DEPTH_8U, 3);
		*ipl_in2 = IplImage(in2);
		scalar = cvGet2D(ipl_in2, rows/2, cols/2);

		if((int)scalar.val[1]>254
			// && (int)scalar.val[1]<30
			&& (int)scalar.val[2]>254)
		{
			#ifdef DEBUG
			std::cout << "饱和度" << (int)scalar.val[1] << std::endl;
			std::cout << "明度值" << (int)scalar.val[2] << std::endl;
			#endif

			return true;
		}
		else
		{
			#ifdef DEBUG
			std::cout << "未获取HSV" << std::endl;
			#endif
		}
		
		#ifdef DEBUG
		std::cout << "true" << std::endl;
		#endif
	}
	else 
	{
		return false;
		#ifdef DEBUG
		std::cout << "false" << std::endl;
		#endif
	}
	
}

bool bboxes_armor_isok(const cv::Rect& rect_l, const cv::Rect& rect_r)
{
	const double egde_l = std::min(rect_l.x, rect_r.x);
	const double egde_r = std::max(rect_l.x+rect_l.width, rect_r.x+rect_r.width);
	// const double egde_d = std::min(rect_l.y-rect_l.height, rect_r.y-rect_r.height);
	// const double egde_u = std::max(rect_l.y, rect_r.y);
	const double egde_u = std::max(rect_l.y+rect_l.height, rect_r.y+rect_r.height);
	const double egde_d = std::min(rect_l.y, rect_r.y);
	const double bbox_light_width = std::max(rect_l.width, rect_r.width);
	const double bbox_light_height = std::max(rect_l.height, rect_r.height);
	
	const double bbox_armor_width = egde_r - egde_l;
	const double bbox_armor_height = egde_u - egde_d;

	// 判断是否为装甲板
	
	// 添加装甲板和灯条框的高度相比：<1.2 && >0.9

	if
	(bbox_armor_height/bbox_armor_width<0.5
	&& bbox_armor_height/bbox_armor_width>0.2
	&& bbox_armor_height/bbox_light_height>0.9
	&& bbox_armor_height/bbox_light_height<1.5
	// && rect_r.width/rect_l.width>0.7
	// && rect_r.width/rect_l.width<1.3
	)
	{
		return true;
	}
	// else if (bbox_armor_width/bbox_armor_height<1.2 
	// && bbox_armor_width/bbox_armor_height>0.3)
	// {
	// 	return true;
	// }
	else return false;
}

cv::Rect get_armor(const cv::Rect& rect_l, const cv::Rect& rect_r)
{
	const int egde_l = std::min(rect_l.x, rect_r.x);
	const int egde_r = std::max(rect_l.x+rect_l.width, rect_r.x+rect_r.width);
	// const int egde_d = std::min(rect_l.y-rect_l.height, rect_r.y-rect_r.height);
	// const int egde_u = std::max(rect_l.y, rect_r.y);
	const double egde_u = std::max(rect_l.y+rect_l.height, rect_r.y+rect_r.height);
	const double egde_d = std::min(rect_l.y, rect_r.y);
	
	const int bbox_armor_width = egde_r - egde_l;
	const int bbox_armor_height = egde_u - egde_d;

	cv::Rect armor(egde_l, egde_d, bbox_armor_width, bbox_armor_height);
	return armor;
}

cv::Mat& mser(cv::Mat& mat, cv::Mat& mat_real)
{
	cv::Mat img_show;
	
	// cv::resize(mat, mat, {640, 480});
	
	double thresh_binar = 0.02; //二值化取thresh_binar最亮部分
	double bbox_proportion_thresh_max = 0.3; //设定bbox的宽高比上阈值
	double bbox_proportion_thresh_min = 0.0; //设定bbox的宽高比下阈值
	constexpr float thresh_iou = 0.5; //IOU的阈值
	int thresh_value = 250; // bboxes_light的色度阈值


	sp::proportion_thresh(mat, mat, 255, thresh_binar); //二值化图像


	auto mser = cv::MSER::create(	5, // _delta 灰度值的变化量
									50, //_min_area 检测到的组块⾯积的范围
									14400, //_max_area 检测到的组块⾯积的范围
									0.25, //_max_variation 最⼤的变化率
									0.2, // _min_diversity 稳定区域的最⼩变换量
									200, //  _max_evolution 对彩⾊图像的MSER检测
									1.01, // _area_threshold 
									0.003, // _min_margin 
									5 // _edge_blur_size 
									); 

	// auto mser = cv::MSER::create(	5, // _delta 灰度值的变化量
	// 								60, //_min_area 检测到的组块⾯积的范围
	// 								14400, //_max_area 检测到的组块⾯积的范围
	// 								0.25, //_max_variation 最⼤的变化率
	// 								0.2, // _min_diversity 稳定区域的最⼩变换量
	// 								200, // _max_evolution 对彩⾊图像的MSER检测
	// 								1.01, // _area_threshold 
	// 								0.003, // _min_margin 
	// 								5 //  _edge_blur_size 
	// 								); 
									
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Rect>               bboxes;
	std::vector<cv::Rect>               bboxes_light;
	std::vector<cv::Rect>               bboxes_armor;
	std::vector<cv::Rect>               bboxes_armor_selected;

	mser->detectRegions(mat, contours, bboxes); // 检测边缘

	#ifdef DEBUG
	std::cout << "All: " << bboxes.size() << " bboxes" << '\n';
	#endif

	std::vector<int> drawed_rects;
	drawed_rects.reserve(bboxes.size() / 4);
	bboxes_light.reserve(drawed_rects.size());

	// 筛选bbox并将bbox画在图像上

	int cnt = 0;
	if (!bboxes.empty())
	{
		++cnt;

		//筛选bboxes
		if(bboxes.front().width/bboxes.front().height < bbox_proportion_thresh_max
		&& bboxes.front().width/bboxes.front().height > bbox_proportion_thresh_min)
		{
			// 画灯条矩形
			#ifdef SHOW_LIGHT

			#ifdef DEBUG
			cv::rectangle(mat, bboxes.front(), {255}, 2);
			#endif

			cv::rectangle(mat_real, bboxes.front(), {0,255,0}, 2);
			#endif

			drawed_rects.push_back(0);
		}
	}

	for (int i = 1; i < bboxes.size(); ++i)
	{
		bool skip = false;
		for (auto&& index : drawed_rects)
			if (skip = (sp::iou(bboxes[i], bboxes[index]) > thresh_iou))
				break;
		if (skip)
			continue;
		
		//筛选bboxes
		if(bboxes.front().width/bboxes.front().height < bbox_proportion_thresh_max
		&& bboxes.front().width/bboxes.front().height > bbox_proportion_thresh_min)
		{

			cv::Mat imagePart=mat_real(bboxes[i]); //抠图
			// IplImage* ipl_in;
			// ipl_in = &IplImage(imagePart);

			// IplImage *ipl_in = (IplImage *) &IplImage(imagePart);

			if(bboxes_light_is_ok(imagePart, 255, thresh_binar, thresh_value))
			{
				#ifdef DEBUG
				cv::imshow("imagePart",imagePart);
				#endif

				// 画灯条矩形
				#ifdef SHOW_LIGHT

				#ifdef DEBUG				
				cv::rectangle(mat, bboxes[i], {255}, 2);
				#endif

				cv::rectangle(mat_real, bboxes[i], {0,255,0}, 2);
				#endif
				
				drawed_rects.push_back(i);
				++cnt;
				bboxes_light.push_back(bboxes[i]);
			}
		}		
	}

	for (int k = 0; k < bboxes_light.size(); k++)
	{
		bool skip_light = false;
		for (int l=0;l<bboxes_light.size();l++)
			if (skip_light = sp::bboxes_armor_isok(bboxes_light[k], bboxes_light[l]) && l!=k)// 筛选条件
			{
				bboxes_armor.push_back(sp::get_armor(bboxes_light[k], bboxes_light[l]));
			}
	}

	// bboxes_armor去重
	for (int m = 0; m < bboxes_armor.size(); m++)
	{
		bool skip = false;
		for (int n=m;n<bboxes_armor.size();n++)
			if (skip = (sp::iou(bboxes_armor[m], bboxes_armor[n]) > thresh_iou) && m!=n)
				break;
		if (skip)
			continue;
		bboxes_armor_selected.push_back(bboxes_armor[m]);

		#ifdef DEBUG
		std::cout<<"pushed";
		#endif
	}

	for (int p=0;p<bboxes_armor_selected.size();p++)
	{
		// 画装甲板矩形
		#ifdef SHOW_ARMOR

		#ifdef DEBUG
		cv::rectangle(mat, bboxes_armor_selected[p], {255}, 2);
		#endif

		cv::rectangle(mat_real, bboxes_armor_selected[p], {0,0,255}, 2);
		#endif
	}

	#ifdef DEBUG
	std::string cnt_str = std::to_string(cnt);	
	std::cout << "Answer: "+cnt_str+" bboxes" << '\n';
	#endif

	return mat;
}

}
#endif
