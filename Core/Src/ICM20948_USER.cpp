/*
 * ICM20948_USER.cpp
 *
 *  Created on: Feb 10, 2023
 *      Author: OHYA Satoshi
 */

#include "ICM20948_USER.h"

void ICM20948_USER::confirmConnection(){
    uint8_t whoami = icm20948->whoami();
	for(uint8_t n=0; n<10 && whoami!=0xea; n++){
		// message("Error : Icm20948 is not detected \n retrying...",2);
		HAL_I2C_DeInit(icm20948->getI2CHandller());
		HAL_I2C_Init(icm20948->getI2CHandller());
		icm20948->changeUserBank(ICM20948::REGISTER::BANK::BANK0);
		icm20948->reset();
		HAL_Delay(100);
		whoami = icm20948->whoami();
	}
	if(whoami!=0xea){
		throw std::runtime_error("ICM20948 is not detected");
	}
}

void ICM20948_USER::init(){
    icm20948->reset();
    icm20948->accelConfig(ICM20948::AccelSensitivity::SENS_2G,true,7);
    icm20948->gyroConfig(ICM20948::GyroSensitivity::SENS_500, true, 7);

    uint8_t tmp=3;
    icm20948->memWrite(ICM20948::REGISTER::BANK2::GYRO_SMPLRT_DIV, tmp);
    tmp=4;
    icm20948->memWrite(ICM20948::REGISTER::BANK2::GYRO_CONFIG_2, tmp);
    tmp=0;
    icm20948->memWrite(ICM20948::REGISTER::BANK2::ACCEL_CONFIG_2, tmp);
    tmp=3;
    icm20948->memWrite(ICM20948::REGISTER::BANK2::ACCEL_SMPLRT_DIV_2, tmp);
    icm20948->changeUserBank(ICM20948::REGISTER::BANK::BANK0);

    icm20948->pwrmgmt1(0x01);
    icm20948->intPinConfig(0b00010000);
    icm20948->intenable1();
    icm20948->pwrmgmt2(0b0);
}

uint16_t ICM20948_USER::calibration(Vector3D<float> &gyro){
	if(__isCalibrated){
		return 0xffff;
	}

	if(!(averageCounter == 0 && gyro.norm() < 0.1)){
		if((gyroAverage - gyro).norm() > 0.5)
		{
			for(uint8_t n=0; n<3; n++){
				gyroAverage[n] = 0;
			}
			averageCounter = 0;
			return 0;
		}
	}

	gyroAverage = (gyroAverage*averageCounter+gyro)/(double)(averageCounter+1);
	averageCounter += 1;

	if(averageCounter > 1000){
		__isCalibrated = true;
	}

	return averageCounter;
}

void ICM20948_USER::getIMU(Vector3D<float> &accel, Vector3D<float> &gyro){
	icm20948->readIMU();
	icm20948->getIMU(accel, gyro);
	accel = accel - accelAverage;
	gyro = gyro - gyroAverage;
}
