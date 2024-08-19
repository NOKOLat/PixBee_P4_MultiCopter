#include <array>
#include <cmath>

#include "TWO_DOF_PID.h"
#include "SBUS_Handller.h"
#include "Quaternion/Quaternion.h"


auto *rollParam = new TWO_DOF_PID_PARAM<float>(0,0.15,0.00001,0.1,0.1,-0.1);
auto *yawRateParam = new TWO_DOF_PID_PARAM<float>(0,0.1,0,0,0.1,-0.1);
auto *altitudeParam = new TWO_DOF_PID_PARAM<float>(0,0.1,0,0.05,0.2,-0.2);
multicopter::ALTITUDE_CONTROL_MODE initialAltitudeControl = multicopter::ALTITUDE_CONTROL_MODE::THROTTLE;
float initialBankAngleLim = 20*std::numbers::pi / 180.0;
float initialYawRateLim = 20*std::numbers::pi / 180.0;

//sbus calibration values
std::array<uint16_t, 18> center = {
    1500, 1500, 1500, 1500,
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

Quaternion<float> imuFrame(1/std::sqrt(2),0,0,1/std::sqrt(2));
