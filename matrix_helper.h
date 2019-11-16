#pragma once
#include <string>
#include <vector>
#include <fstream>

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/transform.hpp>

#include "VisMtvApi.h"

#define __cv3__ *(glm::fvec3*)
#define __cv4__ *(glm::fvec4*)
#define __cm4__ *(glm::fmat4x4*)

namespace mat_helper {

	std::vector<std::string> __split(const std::string& s, const std::string& delimiter)
	{
		size_t pos_start = 0, pos_end, delim_len = delimiter.length();
		std::string token;
		std::vector<std::string> res;

		while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
			token = s.substr(pos_start, pos_end - pos_start);
			pos_start = pos_end + delim_len;
			res.push_back(token);
		}

		res.push_back(s.substr(pos_start));
		return res;
	};

	bool load_cam_calib(glm::fvec3& cam_pos_rbframe, glm::fvec3& cam_up_rbframe, glm::fvec3& cam_view_rbframe, vzm::CameraParameters& cam_params, const std::string& calib_path){
		int count = 0;
		std::string line_text;
		std::ifstream cam_calib_info(calib_path);
		if (!cam_calib_info) {
			return false;
		}
		std::getline(cam_calib_info, line_text); // Camera Intrinsics
		std::getline(cam_calib_info, line_text);
		cam_params.fx = std::stof(line_text);
		std::getline(cam_calib_info, line_text);
		cam_params.fy = std::stof(line_text);
		std::getline(cam_calib_info, line_text);
		cam_params.sc = std::stof(line_text);
		std::getline(cam_calib_info, line_text);
		cam_params.cx = std::stof(line_text);
		std::getline(cam_calib_info, line_text);
		cam_params.cy = std::stof(line_text);
		std::getline(cam_calib_info, line_text);
		cam_params.w = std::stof(line_text);
		std::getline(cam_calib_info, line_text);
		cam_params.h = std::stof(line_text);
		std::getline(cam_calib_info, line_text); // Camera Extrinsics
		glm::fvec3* fv3_list[3] = { &cam_pos_rbframe, &cam_view_rbframe, &cam_up_rbframe };
		while (std::getline(cam_calib_info, line_text))
		{
			std::vector<std::string> xyz = __split(line_text, ",");
			if (xyz.size() == 3)
				*fv3_list[count++] = glm::fvec3(std::stof(xyz[0]), std::stof(xyz[1]), std::stof(xyz[2]));
		}
		cam_params.projection_mode = 3;

		return true;
	};

	void MatrixWS2CS(glm::fmat4x4 &mat, glm::fvec3& pos_eye, glm::fvec3& vec_up, glm::fvec3& vec_view)
	{
		using namespace glm;
		const dvec3& _pos_eye = pos_eye;
		const dvec3& _vec_up = vec_up;
		const dvec3& _vec_view = vec_view;

		dvec3 d3VecAxisZ = -_vec_view;
		d3VecAxisZ = glm::normalize(d3VecAxisZ);

		dvec3 d3VecAxisX = glm::cross(_vec_up, d3VecAxisZ);
		d3VecAxisX = glm::normalize(d3VecAxisX);

		dvec3 d3VecAxisY = glm::cross(d3VecAxisZ, d3VecAxisX);
		d3VecAxisY = glm::normalize(d3VecAxisY);

		fmat4x4& _mat = mat;

		_mat[0][0] = d3VecAxisX.x;
		_mat[0][1] = d3VecAxisY.x;
		_mat[0][2] = d3VecAxisZ.x;
		_mat[0][3] = 0;
		_mat[1][0] = d3VecAxisX.y;
		_mat[1][1] = d3VecAxisY.y;
		_mat[1][2] = d3VecAxisZ.y;
		_mat[1][3] = 0;
		_mat[2][0] = d3VecAxisX.z;
		_mat[2][1] = d3VecAxisY.z;
		_mat[2][2] = d3VecAxisZ.z;
		_mat[2][3] = 0;
		_mat[3][0] = -glm::dot(d3VecAxisX, _pos_eye);
		_mat[3][1] = -glm::dot(d3VecAxisY, _pos_eye);
		_mat[3][2] = -glm::dot(d3VecAxisZ, _pos_eye);
		_mat[3][3] = 1;
	};


	void computeDXProjectionMatrix(float* mat_cs2ps, float* mat_ps2ss, const float* mat_int3x3,
		const float x0, const float y0, const float width, const float height, const float znear, const float zfar)
	{
		float q = zfar / (znear - zfar);
		float qn = zfar * znear / (znear - zfar);

		glm::fmat4x4 mat_p;
		glm::fmat3x3 K = *(glm::mat3x3*)mat_int3x3;

		float fx = K[0][0];
		float fy = K[1][1];
		float sc = K[1][0];
		float cx = K[2][0];
		float cy = K[2][1];

		mat_p[0][0] = 2.f*fx / width;
		mat_p[1][0] = -2.f*sc / width;
		mat_p[2][0] = (width + 2.f * x0 - 2.f*cx) / width;
		mat_p[3][0] = 0;
		mat_p[0][1] = 0;
		mat_p[1][1] = 2.f*fy / height;
		mat_p[2][1] = -(height + 2.f*y0 - 2.f*cy) / height;
		mat_p[3][1] = 0;
		mat_p[0][2] = 0;
		mat_p[1][2] = 0;
		mat_p[2][2] = q;
		mat_p[3][2] = qn;
		mat_p[0][3] = 0;
		mat_p[1][3] = 0;
		mat_p[2][3] = -1.f;
		mat_p[3][3] = 0;

		__cm4__ mat_cs2ps = mat_p;

		glm::fmat4x4 matTranslate, matScale, matTransform, matTranslateSampleModel;
		matTranslate = glm::translate(glm::fvec3(1., -1., 0.));
		matScale = glm::scale(glm::fvec3(width*0.5, height*0.5, 1.));
		//matTranslateSampleModel = glm::translate(glm::fvec3(-0.5, 0.5, 0.));

		glm::fmat4x4 mat_s;
		mat_s = matTranslateSampleModel * (matScale * matTranslate);

		mat_s[0][1] *= -1.;
		mat_s[1][1] *= -1.;
		mat_s[2][1] *= -1.;
		mat_s[3][1] *= -1.;

		__cm4__ mat_ps2ss = mat_s;
	};
}