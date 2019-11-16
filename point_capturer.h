#pragma once

#include <string>
#include <vector>
#include <utility>

#include <opencv2/core.hpp>

class Streamer;
class PointCapturer {
public:
	PointCapturer();
	~PointCapturer();

public:
	void ImageCapture(cv::Mat& img);
	void SavePoints(std::string path);

	int* points();

private:

private:
	Streamer* rs_ = nullptr;

	std::vector<std::pair<cv::Point2i, int>> points_;

	int pressed_key_;
};

