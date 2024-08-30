#include "SBUS_Handller.h"

SBUS_HANDLE::SBUS_HANDLE(nokolat::SBUS_DATA lower, nokolat::SBUS_DATA center, nokolat::SBUS_DATA upper)
:center(center),upper(upper),lower(lower){

}

void SBUS_HANDLE::onReceive(multicopter::INPUT &input){
	data = this->decode(receiveBuffer);
    input.sbusRollNorm = -getRollNorm();
    input.sbusPitchNorm = getPitchNorm();
    input.sbusYawRateNorm = getYawNorm();
    input.sbusAltitudeNorm = -getAltitudeNorm();
    input.updateFlag = true;
}

float SBUS_HANDLE::getNorm(const uint8_t channel){
    float tmp = data.at(channel) - lower.at(channel);
    float res = 0;
	res = (tmp / (upper.at(channel) - lower.at(channel))) * 2.0 - 1.0;

    return res;
}
