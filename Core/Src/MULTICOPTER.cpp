/*
 * MULTICOPTER.cpp
 *
 *  Created on: Jul 6, 2024
 *      Author: ohya
 */

#include "MULTICOPTER.h"

namespace multicopter{
std::string to_string(OUTPUT arg){
	return std::to_string((int8_t)(arg[0]*100)) + ", "
			+ std::to_string((int8_t)(arg[1]*100)) + ", "
			+ std::to_string((int8_t)(arg[2]*100)) + ", "
			+ std::to_string((int8_t)(arg[3]*100));
}


MULTICOPTER::MULTICOPTER(PARAMETER &param, ElapsedTimer *elapsedTimer)
:_param(param),elapsedTimer(elapsedTimer)
{
	mainMode = MAIN_MODE::DISARM;
	
	rollController = new TWO_DOF_PID(*param.roll, elapsedTimer);
	pitchController = new TWO_DOF_PID(*param.pitch, elapsedTimer);
	yawRateController = new TWO_DOF_PID(*param.yawRate, elapsedTimer);
	altitudeController = new TWO_DOF_PID(*param.altitude, elapsedTimer);
	altitudeControlMode = param.altitudeControlMode;
}

OUTPUT MULTICOPTER::controller(const INPUT &input){
	OUTPUT res={};

	controllerPreProcess(input);
	if(mainMode != MAIN_MODE::ARM){
		yawRateController->reset();
		rollController->reset();
		pitchController->reset();
		return res;
	}


	float refRoll = input.sbusRollNorm*_param.bankAngleLimit;
	float refPitch = input.sbusPitchNorm*_param.bankAngleLimit;
	float refYawRate = input.sbusYawRateNorm*_param.yawRateLimit;

	std::array<float, 4> u;
	u.at(3) = yawRateController->controller(refYawRate,input.yawRate);
	u.at(1) = rollController->controller(refRoll,input.roll);
	u.at(2) = pitchController->controller(refPitch,input.pitch);

	switch(altitudeControlMode){
		case ALTITUDE_CONTROL_MODE::ALTITUDE_FEEDBACK:
			u.at(0) = altitudeController->controller(input.sbusAltitudeNorm,input.altitude);
			break;
		case ALTITUDE_CONTROL_MODE::THROTTLE:
			u.at(0) = (input.sbusAltitudeNorm + 1.0) * 0.5;
			break;
	}

//	for(auto &it:u){
//		it = std::copysign(std::sqrt(std::abs(it)), it);
//	}

	controlValue = u;

	res[0] = u[0] + u[1] + u[2] - u[3];
	res[1] = u[0] + u[1] - u[2] + u[3];
	res[2] = u[0] - u[1] + u[2] + u[3];
	res[3] = u[0] - u[1] - u[2] - u[3];

	for(auto &it:res){
		if(it<0){
			it = 0;
		}else if(it>1.0){
			it = 1.0;
		}else{
			it = std::sqrt(it);
		}
	}

	return res;

}

void MULTICOPTER::controllerPreProcess(const INPUT &input){
	std::function<bool(const INPUT)> isArming =[](const INPUT &input){
		if(input.sbusAltitudeNorm < -0.99){
			if(input.sbusYawRateNorm > 0.99){
				return true;
			}
		}
		return false;
	};
	std::function<bool(const INPUT)>  isDisArming = [](const INPUT &input){
		if(input.sbusAltitudeNorm < -0.99){
			if(input.sbusYawRateNorm < -0.99){
				return true;
			}
		}
		return false;
	};

	if(mainMode == MAIN_MODE::DISARM && isArming(input)){
		mainMode = MAIN_MODE::ARMING;
		armingMotionStart = elapsedTimer->getTimeMS();
	}else if(mainMode == MAIN_MODE::ARMING){
		if(isArming(input)){
			if(armingMotionStart != 0 && elapsedTimer->getTimeMS() - armingMotionStart >= armingDurationTH){
				mainMode = MAIN_MODE::ARM;
			}
		}else{
			mainMode = MAIN_MODE::DISARM;
			armingMotionStart = 0;
		}
	}else if(mainMode == MAIN_MODE::ARM && isDisArming(input)){
		mainMode = MAIN_MODE::DISARMING;
		armingMotionStart = elapsedTimer->getTimeMS();
	}else if(mainMode == MAIN_MODE::DISARMING){
		if(isDisArming(input)){
			if(armingMotionStart != 0 &&  elapsedTimer->getTimeMS() - armingMotionStart >= armingDurationTH){
				mainMode = MAIN_MODE::DISARM;
			}
		}else{
			mainMode = MAIN_MODE::ARM;
			armingMotionStart = 0;
		}
	}
	
}

std::string MULTICOPTER::getCotrolValue(){
	return std::to_string((int16_t)(controlValue[0]*100)) + ", "
			+ std::to_string((int16_t)(controlValue[1]*100)) + ", "
			+ std::to_string((int16_t)(controlValue[2]*100)) + ", "
			+ std::to_string((int16_t)(controlValue[3]*100));
}


}
