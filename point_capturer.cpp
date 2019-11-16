#include "point_capturer.h"

#include <iostream>
#include <fstream>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


/* Capture tool */
int cur_mx = 0;
int cur_my = 0;
int circle_radius = 15;

bool left_click = false;
int clk_mx = 0;
int clk_my = 0;
void MouseEventCallback(int event, int x, int y, int flags, void* userdata) {
	if (event == cv::EVENT_LBUTTONDOWN) {
		left_click = true;

		clk_mx = x;
		clk_my = y;
	}
	else if (event == cv::EVENT_MOUSEMOVE) {
		cur_mx = x;
		cur_my = y;
	}
	else if (event == cv::EVENT_MOUSEWHEEL) {
		if (cv::getMouseWheelDelta(flags) > 0) {
			circle_radius++;
		}
		else {
			circle_radius--;
			if (circle_radius <= 0)
				circle_radius = 1;
		}
	}

	//if (flags & CV_EVENT_FLAG_CTRLKEY) {
	//	std::cout << "\tCV_EVENT_FLAG_CTRLKEY" << std::endl;
	//}
}

PointCapturer::PointCapturer() //:
	//rs_(new RealsenseStreamer())
{
	//rs_->InitialzeStreamer();
}

PointCapturer::~PointCapturer() {
	//delete rs_;
}

void PointCapturer::ImageCapture(cv::Mat& img) {
	cv::namedWindow("Capture tool", 1);
	cv::setMouseCallback("Capture tool", MouseEventCallback, NULL);
	
	cv::Mat color = cv::Mat::zeros(img.size(), CV_8UC3);

	pressed_key_ = -1;
	while (pressed_key_ == -1 || pressed_key_ != 'q') {
		img.copyTo(color);

		for (int i = 0; i < points_.size(); ++i) {
			cv::circle(color, points_.at(i).first, points_.at(i).second, cv::Scalar(0, 255, 0), 1);
			cv::drawMarker(color, points_.at(i).first, cv::Scalar(0, 255, 0), cv::MarkerTypes::MARKER_CROSS, 2 * points_.at(i).second, 1);
		}

		cv::circle(color, cv::Point(cur_mx, cur_my), circle_radius, cv::Scalar(0, 0, 255), 1);
		cv::drawMarker(color, cv::Point(cur_mx, cur_my), cv::Scalar(0, 0, 255), cv::MarkerTypes::MARKER_CROSS, 2 * circle_radius, 1);

		cv::imshow("Capture tool", color);
		pressed_key_ = cv::waitKey(1);

		if (left_click) {
			points_.emplace_back(std::make_pair(cv::Point2i(clk_mx, clk_my), circle_radius));
			std::cout << "[Info] x = " << clk_mx << ", y = " << clk_my << ", # of points = " << points_.size() << std::endl;

			left_click = false;
		}

		if (pressed_key_ == 'r') {
			if (!points_.empty()) {
				points_.pop_back();
				std::cout << "[Info] Removed the latest point" << std::endl;
			}
		}
	}

	cv::destroyAllWindows();
}

void PointCapturer::SavePoints(std::string path) {
	std::ofstream out(path);

	out << points_.size() << std::endl;
	for (int i = 0; i < points_.size(); ++i) {
		out << points_.at(i).first.x << ", " << points_.at(i).first.y << std::endl;
	}

	out.close();
}

int* PointCapturer::points() {
	std::vector<int> points;

	for (int i = 0; i < points_.size(); ++i) {
		points.emplace_back(points_.at(i).first.x);
		points.emplace_back(points_.at(i).first.y);
	}

	return points.data();
}
