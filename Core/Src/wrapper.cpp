/*
 * wrapper.cpp
 *
 *  Created on: Oct 23, 2023
 *      Author: ohya
 */

#include "wrapper.hpp"

#include "common.h"

#include "tim.h"
#include "usart.h"
#include "dma.h"
#include "gpio.h"
#include <string>
#include <array>
#include <bitset>
#include <exception>
#include <cmath>
#include <functional>

/*
 * param:
 * param1(str):set message in std::string
 * param2(level):set message level. Planning to use message control.(For error = 0 and system message = 1, runtime message = 2, debug = 3)
 */
const uint8_t messageLevel = 3;
static void message(std::string str, uint8_t level = 3);

std::array<float, 3> gyroValue;
std::array<float, 3> AccelValue;

void init(){
	bool isInitializing = true;

	SET_MASK_ICM20948_INTERRUPT();

	HAL_TIM_PWM_Start(RED_LED);
	HAL_TIM_PWM_Start(YELLOW_LED);
	HAL_TIM_PWM_Start(BLUE_LED);

	__HAL_TIM_SET_COMPARE(ledTim, RED_LED_CHANNEL, 100);
	__HAL_TIM_SET_COMPARE(ledTim, YELLOW_LED_CHANNEL, 100);
	__HAL_TIM_SET_COMPARE(ledTim, BLUE_LED_CHANNEL, 100);
	HAL_Delay(1000);
	__HAL_TIM_SET_COMPARE(ledTim, RED_LED_CHANNEL, 0);
	__HAL_TIM_SET_COMPARE(ledTim, BLUE_LED_CHANNEL, 0);

	//start to receive sbus.
	HAL_UART_Receive_DMA(huartSbus,hsbus.getReceiveBufferPtr(),hsbus.getDataLen());
	HAL_TIM_PWM_Start_IT(&htim14, TIM_CHANNEL_1);
	__HAL_TIM_ENABLE_IT(&htim14, TIM_IT_UPDATE);

	if(elapsedTimer->selfTest() == false){
		message("ERROR : elapsed timer freaquency is not correct",0);
	}else{
		message("elapsed timer is working",2);
		__HAL_TIM_SET_COMPARE(ledTim, YELLOW_LED_CHANNEL, 200);
	}
	elapsedTimer->start();

	/*
	 * communication check with icm20948
	 * initialize icm20948
	 */
	try{
		icm20948User.confirmConnection();
		message("ICM20948 is detected",2);
		__HAL_TIM_SET_COMPARE(ledTim, YELLOW_LED_CHANNEL, 300);
	}catch(std::runtime_error &e){
		message("Error : Icm20948 is not detected",0);
	}
	icm20948User.init();
	_icm20948Callback = icm20948CallbackCalibration;
	CLEAR_MASK_ICM20948_INTERRUPT();
	message("ICM20948 is initialized");

	while(isInitializing){
		isInitializing = !attitudeEstimate.isInitialized();
		bool tmp = icm20948User.isCalibrated();
		if(tmp == true){
			message("icm20948 calibration is finished",3);
			_icm20948Callback = icm20948Callback;
		}
		isInitializing = !tmp;
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1));
//		esc.setSpeed(0);
		HAL_Delay(50);
	}
	__HAL_TIM_SET_COMPARE(ledTim, YELLOW_LED_CHANNEL, 400);
	HAL_Delay(100);


	esc.enable();
	esc.arm();

	__HAL_TIM_SET_COMPARE(ledTim, YELLOW_LED_CHANNEL, 500);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	message("Initialization is complete",1);
}

void loop(){
	HAL_Delay(100);
	if(hmulticopter->getMainMode() == multicopter::MAIN_MODE::ARM){
		__HAL_TIM_SET_COMPARE(ledTim, BLUE_LED_CHANNEL, 500);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	}else if(hmulticopter->getMainMode() == multicopter::MAIN_MODE::DISARM){
		__HAL_TIM_SET_COMPARE(ledTim, BLUE_LED_CHANNEL, 0);
	}else{
		__HAL_TIM_SET_COMPARE(ledTim, BLUE_LED_CHANNEL, 250);
	}
}

void icm20948CallbackCalibration(){
	Vector3D<float> accel;
	Vector3D<float> gyro;
	icm20948->readIMU();
	icm20948->getIMU(accel, gyro);
	icm20948User.calibration(gyro);
}

void icm20948Callback(){
	Vector3D<float> accel;
	Vector3D<float> gyro;

	icm20948User.getIMU(accel, gyro);

	attitudeEstimate.setAccelValue(accel);
	attitudeEstimate.setGyroValue(gyro);
	attitudeEstimate.update();

	auto attitude = attitudeEstimate.getAttitude();
	auto z_machienFrame = attitude.rotateVector({0,0,1.0});
	float roll = std::asin(z_machienFrame[0]);
	float pitch = std::asin(z_machienFrame[1]);
	float yawRate = gyro[2];

	multicopterInput.roll = roll;
	multicopterInput.pitch = pitch;
	multicopterInput.yawRate = yawRate;
	auto res = hmulticopter->controller(multicopterInput);
	esc.setSpeed(res);
	message(multicopter::to_string(res), 3);
//	message(hmulticopter->getCotrolValue(), 3);
//	message(std::to_string(int16_t(roll*180/std::numbers::pi))+", "+std::to_string(int16_t(pitch*180/std::numbers::pi)),3);


}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == ICM20948_IT_Pin){
		if(HAL_GPIO_ReadPin(ICM20948_IT_GPIO_Port, ICM20948_IT_Pin) == GPIO_PIN_SET){
			_icm20948Callback();
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == huartSbus){
		htim14.Instance->CNT = 0;
		hsbus.onReceive(multicopterInput);
		if(hsbus.getData().failsafe){
			hmulticopter->rcFailSafe();
		}else if(hsbus.getData().framelost){
			hmulticopter->setRcFrameLost();
			esc.setSpeed(0);
		}else{
			hmulticopter->setRcFrameLost(false);
		}
		HAL_UART_Receive_DMA(huartSbus,hsbus.getReceiveBufferPtr(),hsbus.getDataLen());

		if(hsbus.getData().failsafe || hsbus.getData().framelost){
			return;
		}

		esc.setSpeed(hmulticopter->controller(multicopterInput));
//		std::string str= std::to_string(int8_t(multicopterInput.sbusRollNorm*100))+", ";
//		str += std::to_string(int8_t(multicopterInput.sbusPitchNorm*100))+", ";
//		str += std::to_string(int8_t(multicopterInput.sbusYawRateNorm*100))+", ";
//		str += std::to_string(int8_t(multicopterInput.sbusAltitudeNorm*100))+", ";
//		message(str, 3);
		__HAL_TIM_SET_COMPARE(ledTim, RED_LED_CHANNEL, 500);
	}else if(huart == huartXbee){

	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart == huartXbee){
		if(huart->RxEventType == HAL_UART_RXEVENT_TC){

		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == huartXbee){

	}
}

static void message(std::string str, uint8_t level){
	if(level <= messageLevel){
		str += "\n";
		if(huart4.gState == HAL_UART_STATE_READY){
			static std::string messageBuffer;
			messageBuffer = std::string(str);
			HAL_UART_Transmit_DMA(&huart4, (uint8_t *)messageBuffer.c_str(), messageBuffer.length());
		}
	}
	return;
}

void tim14Callback(){
	auto htim = &htim14;
	uint16_t sr = htim->Instance->SR;
	if((sr & (TIM_IT_CC1)) == (TIM_IT_CC1)){
		__HAL_TIM_SET_COMPARE(ledTim, RED_LED_CHANNEL, 100);
		__HAL_TIM_CLEAR_FLAG(htim,TIM_IT_CC1);
		HAL_UART_AbortReceive(huartSbus);
		HAL_UART_Receive_DMA(huartSbus,hsbus.getReceiveBufferPtr(),hsbus.getDataLen());
		hmulticopter->setRcFrameLost();
	}else if((sr & TIM_IT_UPDATE) == (TIM_IT_UPDATE)){
		__HAL_TIM_SET_COMPARE(ledTim, RED_LED_CHANNEL, 300);
		HAL_UART_Receive_DMA(huartSbus,hsbus.getReceiveBufferPtr(),hsbus.getDataLen());
		__HAL_TIM_CLEAR_FLAG(htim,TIM_IT_UPDATE);
		hmulticopter->rcFailSafe();
//		esc.setSpeed(0);
	}
}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
//	if(huart == &huart3){
//		huart->gState = HAL_UART_STATE_READY;
//	}
//}
