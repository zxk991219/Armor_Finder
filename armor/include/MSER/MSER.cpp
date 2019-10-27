
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "proportion_thresh.hpp"
#include "other/include/timer.hpp"

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

}

int main()
{   
	sp::timer timer;
	timer.reset();

	auto mat = cv::imread("../image/test_image.jpg", cv::IMREAD_GRAYSCALE);
	cv::resize(mat, mat, {1280, 720});
	sp::proportion_thresh(mat, mat, 255, 0.014); //二值化图像

	auto mser = cv::MSER::create();
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Rect>               bboxes;

	mser->detectRegions(mat, contours, bboxes);
	std::cout << "All: " << bboxes.size() << " bboxes" << '\n';
	
	//输出二值化后的图
	// std::string bboxes_size_str = std::to_string(bboxes.size());
	// cv::imshow("Origin: "+bboxes_size_str+" bboxes", mat);
	// cv::waitKey(0);
	
	// std::cin.get(); 

	//重新读入彩色图像
	auto mat_multicolor = cv::imread("../image/test_image.jpg");
	cv::resize(mat_multicolor, mat_multicolor, {1280, 720});

	std::vector<int> drawed_rects;
	drawed_rects.reserve(bboxes.size() / 4);

	double bbox_proportion_thresh_max = 5.0; //设定bbox的高宽比上阈值
	double bbox_proportion_thresh_min = 1.0; //设定bbox的高宽比下阈值
	// double bbox_proportion_thresh_min = 1.00000000000000012; //(多一点不行，少一点也不行）设定bbox的高宽比上阈值

	//画矩形	
	int cnt = 0;
	if (!bboxes.empty())
	{
		++cnt;

		//筛选bboxes
		if(bboxes.front().width/bboxes.front().height > bbox_proportion_thresh_max);
		// else if(bboxes.front().height/bboxes.front().width > bbox_proportion_thresh_max);
		// else if(bboxes.front().width/bboxes.front().height < bbox_proportion_thresh_min);
		else if(bboxes.front().height/bboxes.front().width < bbox_proportion_thresh_min);
		else
		{
			cv::rectangle(mat_multicolor, bboxes.front(), {0, 0, 255}, 2);
			drawed_rects.push_back(0);
		}	
	}

	constexpr float thresh = 0.0001; //IOU的阈值

	for (int i = 1; i < bboxes.size(); ++i)
	{
		bool skip = false;
		for (auto&& index : drawed_rects)
			if (skip = (sp::iou(bboxes[i], bboxes[index]) > thresh))
				break;
		if (skip)
			continue;
		
		//筛选bboxes
		if(bboxes[i].width/bboxes[i].height > bbox_proportion_thresh_max);
		// else if(bboxes[i].height/bboxes[i].width > bbox_proportion_thresh_max);
		// else if(bboxes[i].width/bboxes[i].height < bbox_proportion_thresh_min);
		else if(bboxes[i].height/bboxes[i].width < bbox_proportion_thresh_min);
		else
		{
			cv::rectangle(mat_multicolor, bboxes[i], {0, 0, 255}, 2);
			drawed_rects.push_back(i);
			++cnt;
		}		
	}

	std::string cnt_str = std::to_string(cnt);	
	std::cout << "Answer: "+cnt_str+" bboxes" << '\n';

	// std::cin.get();
	cv::imwrite("../image/ans.jpg", mat_multicolor);

	//输出带有bboxes的彩色原图
	// cv::imshow("Answer: "+cnt_str+" bboxes", mat_multicolor);
	// cv::waitKey(0);

	std::cout << "程序运行时间：" << timer.get() << std::endl;
}
