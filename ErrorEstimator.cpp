#include "ErrorEstimator.h"
using namespace ee;

#include <iostream>
#include <algorithm>
#include <map>
#include <vector>
#include <filesystem>
namespace fs = std::filesystem;

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>

#include <realsense_streamer.h>
#include <ir_tracker.h>

#include <helper.h>

#include "matrix_helper.h"
#include "point_capturer.h"
#include "common.h"


std::string filepath_marker_xy_list = "marker_position.txt";
const static int kWorldSceneId = 2;
static helpers::arcball arc_ball;

static void mouseCallbackFunc(int event, int x, int y, int flags, void* userdata) {
	if (event == cv::EVENT_LBUTTONDOWN) {
		// Arcball
		vzm::CameraParameters cam_params;
		vzm::GetCameraParameters(kWorldSceneId, cam_params);
		
		helpers::cam_pose arc_cam_pose;
		glm::fvec3 pos = __cv3__ arc_cam_pose.pos = __cv3__ cam_params.pos;
		__cv3__ arc_cam_pose.up = __cv3__ cam_params.up;
		__cv3__ arc_cam_pose.view = __cv3__ cam_params.view;

		arc_ball.intializer((float*)&glm::fvec3(), glm::length(pos));
		arc_ball.start((int*)&glm::ivec2(x, y), (float*)&glm::fvec2(cam_params.w / 2, cam_params.h / 2), arc_cam_pose);
	}

	if (event == cv::EVENT_MOUSEMOVE &&  flags & cv::EVENT_FLAG_LBUTTON) {
		helpers::cam_pose arc_cam_pose;
		arc_ball.move((int*)&glm::ivec2(x, y), arc_cam_pose);
		vzm::CameraParameters cam_params_ov;
		vzm::GetCameraParameters(kWorldSceneId, cam_params_ov);
		__cv3__ cam_params_ov.pos = __cv3__ arc_cam_pose.pos;
		__cv3__ cam_params_ov.up = __cv3__ arc_cam_pose.up;
		__cv3__ cam_params_ov.view = __cv3__ arc_cam_pose.view;
		vzm::SetCameraParameters(kWorldSceneId, cam_params_ov);
	}

	if (event == cv::EVENT_MOUSEWHEEL) {
		vzm::CameraParameters cam_params_ov;
		vzm::GetCameraParameters(kWorldSceneId, cam_params_ov);
		__cv3__ cam_params_ov.pos = __cv3__ cam_params_ov.pos + (__cv3__ cam_params_ov.view) * 0.002f * (float)cv::getMouseWheelDelta(flags);
		vzm::SetCameraParameters(kWorldSceneId, cam_params_ov);
	}
}

ErrorEstimator::ErrorEstimator(
	std::string path_rs_configuration,
	std::string path_profile,
	std::string path_optitrack_calib
)
	:camera_(new RealsenseStreamer(path_rs_configuration))
{
	tracker_ = new IrTracker(
					std::map<std::string, int>{
					std::make_pair("realsense", kMarker::CAMERA)},
					path_profile,
					path_optitrack_calib);

	projected_uv_vec.resize(0);
}

ErrorEstimator::~ErrorEstimator(){
	if (camera_)
		delete camera_;

	if (tracker_)
		delete tracker_;

	vzm::DeinitEngineLib();
}

bool ee::ErrorEstimator::initialize(){

	if (!camera_->InitialzeStreamer()) {
		std::cout << "[Error] Initialization failed" << std::endl;
		return false;
	}

	if (!tracker_->Initialize()) {
		std::cout << "[Error] ir tracker initialization failed" << std::endl;
		return false;
	}

	/* Rendering Engine */
	if (!vzm::InitEngineLib()) {
		std::cout << "[Error] vzm initialization failed" << std::endl;
		return false;
	}

	if (!initializeScene()) {
		std::cout << "[Error] Loading object failed" << std::endl;
		return false;
	}

	if (!listUpCalibrationFiles()) {
		std::cout << "[Error] No Calibration files" << std::endl;
		return false;
	} else {
		calcMatrixFromCalibFile(path_calibration_.front());
		calc_result_ = true;
	}

	return true;
}



void ee::ErrorEstimator::run(){
	cv::Mat camera_rgb = cv::Mat::zeros(camera_->color_height(), camera_->color_width(), CV_8UC3);
	cv::Mat world_scene = cv::Mat(world_cam_params_.h, world_cam_params_.w, CV_8UC4);

	assert(camera_->color_height() == cam_params_.h && camera_->color_width() == cam_params_.w);

	// Calculate Point for Drawing Line
#pragma region CalcLinePosition
	int div_height = 5;
	int div_width = 5;
	std::vector < std::pair<std::pair<int, int>, std::pair<int, int>> > line_start2end(div_height + div_width - 2);
	int stride_height = camera_->color_height() / div_height;
	int stride_width = camera_->color_width() / div_width;
	for (int i = 1; i < div_height; i++) {
		std::pair<int, int> start = { i*stride_height, 0 };
		std::pair<int, int> end = { i*stride_height , camera_->color_width() - 1 };
		line_start2end.push_back({ start,end });
	}
	for (int i = 1; i < div_width; i++) {
		std::pair<int, int> start = { 0, i*stride_width };
		std::pair<int, int> end = { camera_->color_height() - 1 , i*stride_width };
		line_start2end.push_back({ start,end });
	}
#pragma endregion

	// Marker Set
	std::vector<int> detected_markers_id(30);
	for (int i = 0; i < detected_markers_id.size(); ++i) {
		float xyzr[4] = { 0, 0, 0, 0.01f };
		float color[3] = { 1.f, 1.f, 1.f };
		vzm::GenerateSpheresObject((float*)&xyzr[0], (float*)&color[0], 1, detected_markers_id.at(i));
	}

	int key_pressed = -1;
	while (key_pressed == -1/*no input*/ || key_pressed != 'q') {
		camera_->GetFrame(camera_rgb.data);
		cv::cvtColor(camera_rgb, camera_rgb, cv::COLOR_RGB2BGR);

		// Capture 2D(u,v) Position
		if (key_pressed == 32/*space bar*/) {
			PointCapturer capture_tool;
			capture_tool.ImageCapture(camera_rgb);
			capture_tool.SavePoints(filepath_marker_xy_list);
		}

		// Draw Line at RealSense RGB Image
		for (int i = 0; i < line_start2end.size(); i++) {
			cv::line(camera_rgb,
				cv::Point(line_start2end.at(i).first.second, line_start2end.at(i).first.first),
				cv::Point(line_start2end.at(i).second.second, line_start2end.at(i).second.first),
				cv::Scalar(0, 0, 0),
				3);
		}
		
		// Detect All World Markers
		std::vector<float> marker_xyz_all;
		tracker_->TrackMarkers();
		tracker_->markerXyzAll(&marker_xyz_all);

		// Generate Sphere At World Scene
		int num_markers = (int)marker_xyz_all.size() / 3;
		if (num_markers > 0) {
			for (int i = 0; i < num_markers; ++i) {							// Detecting된 애들을 World에 위치시켜준다.
				vzm::ObjStates obj_states;
				__cm4__ obj_states.os2ws = glm::translate(glm::fvec3(marker_xyz_all[3 * i + 0], marker_xyz_all[3 * i + 1], marker_xyz_all[3 * i + 2]));
				vzm::ReplaceOrAddSceneObject(kWorldSceneId, detected_markers_id.at(i), obj_states);
			}
			for (int i = num_markers; i < detected_markers_id.size(); ++i) { // 이전에 detecting 되었다가 이번에 안된 애들은 invisible로 바꾸어 준다.
				vzm::ObjStates obj_states;
				obj_states.is_visible = false;
				vzm::ReplaceOrAddSceneObject(kWorldSceneId, detected_markers_id.at(i), obj_states);
			}
		}
		
		// Camera
		if (double* d_os2ws = tracker_->os2wsAt(kMarker::CAMERA)) {
			/* Set camera position */
			glm::fmat4x4 rb2ws = glm::fmat4x4(glm::make_mat4x4(d_os2ws));
			helper::drawCameraViewport(kWorldSceneId, rb2ws * glm::inverse(rb2cs_), hmd_cam_viewport_, "HMD CAM");
		}

		if (calc_result_) {
			if (double* d_os2ws = tracker_->os2wsAt(kMarker::CAMERA)) { // Camera가 Detecting이 될 때 projection을 계산한다. // 안그러면 identity mat이 rb2ws에 들어간다.
				glm::fmat4x4 rb2ws = glm::fmat4x4(glm::make_mat4x4(d_os2ws));

				// 카메라 rigid body 제외한 마커 위치
				std::vector<glm::fvec3> testing_marker_pos;
				glm::fvec3 cam_pos = tr_pt(rb2ws, glm::fvec3(0, 0, 0)); // cam rigid body center position
				for (int i = 0; i < num_markers; ++i) {
					glm::fvec3 marker_pos(marker_xyz_all[3 * i + 0], marker_xyz_all[3 * i + 1], marker_xyz_all[3 * i + 2]);
					float dist = glm::distance(cam_pos, marker_pos);
					if (dist > 0.15/*15cm*/) // cam rigid body center와 15cm 이내의 marker는 cam rigid body의 marker로 판별
						testing_marker_pos.emplace_back(marker_pos);
				}

				// indexing markers; z축(+ -> -) 먼저, x축(+ -> -) 다음
				int n_row = 3;
				int n_col = 3;
				// z축
				std::sort(testing_marker_pos.begin(), testing_marker_pos.end(),
					[](const glm::fvec3& a, const glm::fvec3& b) { return a.z > b.z; });

				int cur_row = 0;
				std::vector<std::vector<glm::fvec3>> each_rows(n_row);
				for (int i = 0; i < testing_marker_pos.size(); ++i) {
					if (each_rows.at(cur_row).size() >= n_col)
						cur_row++;

					each_rows.at(cur_row).push_back(testing_marker_pos.at(i));
				}

				// x축
				for (auto& r : each_rows) {
					std::sort(r.begin(), r.end(),
						[](const glm::fvec3& a, const glm::fvec3& b) { return a.x > b.x; });
				};

				projected_uv_vec.clear();
				for (int i = 0; i < each_rows.size(); i++) {
					if (each_rows.at(i).size() != 0) {
						for (int j = 0; j < each_rows.at(i).size(); j++) {
							glm::fvec3 world_xyz = glm::fvec3(each_rows.at(i).at(j).x, each_rows.at(i).at(j).y, each_rows.at(i).at(j).z);
							glm::fvec3 projected_uv = tr_pt(mat_ps2ss*mat_cs2ps*rb2cs_*glm::inverse(rb2ws), world_xyz);
							projected_uv_vec.emplace_back(projected_uv.x, projected_uv.y);
						}
					}
				}
				calc_result_ = false;
			}
		}

		// Draw Projected (u,v) at RealSense RGB Image
		for (int i = 0; i < projected_uv_vec.size(); i++) {
			cv::drawMarker(camera_rgb, cv::Point(projected_uv_vec.at(i).x, projected_uv_vec.at(i).y), cv::Scalar(0, 255, 255), cv::MARKER_TILTED_CROSS, 10);
			cv::putText(camera_rgb, std::to_string(i+1), cv::Point(projected_uv_vec.at(i).x, projected_uv_vec.at(i).y), 2, 1.2, cv::Scalar(255));
		}
		
		// Render RealSense Scene
		cv::imshow("camera", camera_rgb);

		// Render World scene
		vzm::RenderScene(kWorldSceneId);
		vzm::GetRenderBufferPtrs(kWorldSceneId, &world_scene.data, nullptr, nullptr, nullptr);
		cv::imshow("World View", world_scene);

		key_pressed = cv::waitKey(1);
		handleKeyPressed(key_pressed);
	}
}

bool ee::ErrorEstimator::initializeScene(){
	/* Camera Parameter */
	cam_params_.projection_mode = 3;
	cam_params_.is_rgba_write = true;
	tracker_->cam_params(cam_params_.fx, cam_params_.fy, cam_params_.sc, cam_params_.cx, cam_params_.cy, cam_params_.w, cam_params_.h);

	/* World Scene parameter */
	world_scn_env_params_.is_on_camera = true;
	world_scn_env_params_.is_pointlight = true;
	vzm::SetSceneEnvParameters(kWorldSceneId, world_scn_env_params_);

	/* World Camera parameter */
	__cv3__ world_cam_params_.pos = glm::fvec3(-0.5f, 1.0f, 1.0f);
	__cv3__ world_cam_params_.up = glm::fvec3(0, 1.f, 0);
	__cv3__ world_cam_params_.view = glm::fvec3(0.f, 0.6f, 0.f) - glm::fvec3(-0.5f, 1.0f, 1.0f);
	world_cam_params_.w = world_view_width_;
	world_cam_params_.h = world_view_height_;
	world_cam_params_.fov_y = 3.141592654f / 4.f;
	world_cam_params_.aspect_ratio = ((float)world_cam_params_.w / (float)world_cam_params_.h);
	world_cam_params_.projection_mode = 2;
	world_cam_params_.is_rgba_write = true;
	vzm::SetCameraParameters(kWorldSceneId, world_cam_params_);

	helper::drawWorldGrid(kWorldSceneId);

	cv::namedWindow("World View", 1);
	cv::setMouseCallback("World View", mouseCallbackFunc);

	return true;
}

bool ee::ErrorEstimator::calcMatrixFromCalibFile(const std::string& path_calib){
	glm::fvec3 cam_pos_rbframe, cam_up_rbframe, cam_view_rbframe;
	mat_helper::load_cam_calib(cam_pos_rbframe, cam_up_rbframe, cam_view_rbframe, cam_params_, path_calib);
	mat_helper::MatrixWS2CS(rb2cs_, cam_pos_rbframe, cam_up_rbframe, cam_view_rbframe);

	mat_int[0][0] = cam_params_.fx;
	mat_int[1][0] = cam_params_.sc;
	mat_int[2][0] = cam_params_.cx;
	mat_int[1][1] = cam_params_.fy;
	mat_int[2][1] = cam_params_.cy;

	mat_helper::computeDXProjectionMatrix((float*)&mat_cs2ps, (float*)&mat_ps2ss, (float*)&mat_int, 0, 0, (float)cam_params_.w, (float)cam_params_.h, 0.01f, 1000.f);

	return true;
}

bool ErrorEstimator::listUpCalibrationFiles() {
	std::string path = "./calib/";
	for (const auto & entry : fs::directory_iterator(path)) {
		if (!entry.path().compare(".") || !entry.path().compare(".."))
			continue;

		path_calibration_.push_back(entry.path().u8string());
	}

	if (path_calibration_.size() > 0)
		return true;
	return false;
}

void ee::ErrorEstimator::handleKeyPressed(int key){
	if (key == 'h' || key == 'H') {
		std::string path = "./calib/";
		int d_length = path.length(); 

		// path_calibration_에 "./calib/"가 붙어 있어서, "./calib/"의 길이만큼 제외한 string을 추출한다.
		int t_length = path_calibration_.front().length(); //"./calib/*.txt"
		std::string file_name = path_calibration_.front().substr(d_length, t_length-d_length); // "*.txt"

		std::string out_file = "./result/" + file_name;
		std::ofstream out(out_file);
		for (int i = 0; i < projected_uv_vec.size(); i++) {
			glm::fvec2 upvp = projected_uv_vec.at(i);
			out << upvp.x << ", " << upvp.y << std::endl;
		}

		if (path_calibration_.size() > 1) {
			path_calibration_.pop_front();
		
			calcMatrixFromCalibFile(path_calibration_.front());
			calc_result_ = true;
		} else {
			std::cout << "[Error] No more calibration files" << std::endl;
		}
	}
}
