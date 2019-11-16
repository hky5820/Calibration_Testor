#pragma once

// Driver parameters
const static float rad = 47.0;
const static float tx = -99.0;
const static float ty = 8.0;
const static float tz = -163.5;

// Pin offset
static float glass_pin_tx = 0;
static float glass_pin_ty = 0;
static float glass_pin_tz = 0;
//static float glass_pin_tx = 90;
//static float glass_pin_ty = 10.f;
//static float glass_pin_tz = -130;
//static float glass_pin_tx = 69.67169f;
//static float glass_pin_tz = -145.271f;

enum kMarker {
	CAMERA = 0,
	AWL = 1,
	PEDICLEPROBE = 2,
	DRIVER = 3,
	SPINE = 4,
	HMD = 5
};