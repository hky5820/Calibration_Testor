#pragma once

#include <string>
#include <vector>
#include <list>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <VisMtvApi.h>

#include <common.h>

class IArTracker;
class IStreamer;

namespace ee {
	class ErrorEstimator{
	public:
		ErrorEstimator(
			std::string path_rs_configuration,
			std::string path_profile,
			std::string path_optitrack_calib
		);
		~ErrorEstimator();

		bool initialize();
		void run();

	private:
		bool listUpCalibrationFiles();

		bool initializeScene();
		bool calcMatrixFromCalibFile(const std::string& path_calib);
		void handleKeyPressed(int key);

	private:
		IStreamer* camera_ = nullptr;
		IArTracker* tracker_ = nullptr;

		// World Only Object
		ViewportObjectData hmd_cam_viewport_;
		ViewportObjectData tracking_cam0_viewport_;
		ViewportObjectData tracking_cam1_viewport_;

		/* Rendering Camera */
		vzm::CameraParameters cam_params_;

		/* World Camera */
		//const int kWorldSceneId = 2; // static으로 선언..
		vzm::CameraParameters world_cam_params_;
		vzm::SceneEnvParameters world_scn_env_params_;
		// Arcball
		int world_view_width_ = 1280;
		int world_view_height_ = 720;

		std::string path_profile_;
		std::string path_optitrack_calib_;

		glm::fmat4x4 rb2cs_ = glm::fmat4x4(1.f);

		glm::fmat4x4 mat_cs2ps;
		glm::fmat4x4 mat_ps2ss;
		glm::fmat3x3 mat_int;

		std::vector<glm::fvec2> projected_uv_vec;

		std::list<std::string> path_calibration_;
		bool calc_result_ = false;
	};
}
