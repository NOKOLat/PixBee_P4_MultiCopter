#include <array>
#include <cmath>

#include "TWO_DOF_PID.h"
#include "SBUS_Handller.h"
#include "Quaternion/Quaternion.h"


auto *rollParam = new TWO_DOF_PID_PARAM<float>(0,0.020 ,0.003,0.00001,0.1,-0.1);
//auto *rollParam = new TWO_DOF_PID_PARAM<float>(0,0.0 ,0.000,0.000,0.2,-0.2);
//auto *rollParam = new TWO_DOF_PID_PARAM<float>(0,1,0.0000,0.0000,0.03,-0.03);
auto *yawRateParam = new TWO_DOF_PID_PARAM<float>(0.2,0.1,0.01,0.00001,0.05,-0.05);
auto *altitudeParam = new TWO_DOF_PID_PARAM<float>(0,0.1,0,0.05,0.2,-0.2);
multicopter::ALTITUDE_CONTROL_MODE initialAltitudeControl = multicopter::ALTITUDE_CONTROL_MODE::THROTTLE;
float initialBankAngleLim = 30*std::numbers::pi / 180.0;
float initialBankAcceleLim = 360*std::numbers::pi / 180.0;
float initialYawRateLim = 90*std::numbers::pi / 180.0;

//sbus calibration values
std::array<uint16_t, 18> center = {
    1500, 1500, 966, 966,
    1500, 1500, 1500, 1500,
    1500, 1500, 1500, 1500,
    1500, 1500, 1500, 1500,
    0, 0
};

std::array<uint16_t, 18> upper = {
	1680, 1680, 1680, 1680,
	1904, 1904, 1904, 1904,
	1904, 1904, 1500, 1500,
    1500, 1500, 1500, 1500,
    0, 0
};

std::array<uint16_t, 18> lower = {
	368, 368, 368, 368,
	144, 144, 144, 144,
	144, 144, 1500, 1500,
    1500, 1500, 1500, 1500,
    0, 0
};

Quaternion<float> imuFrame(1/std::sqrt(2),0,0,-1/std::sqrt(2));
