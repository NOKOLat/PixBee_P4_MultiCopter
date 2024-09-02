/*
 * common.h
 *
 *  Created on: Jul 5, 2024
 *      Author: ohya
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#include "ICM20948_USER.h"
#include "elapsedTimer/elapsedTimer.h"
#include "ESC_UTILITY/ESC_UTILITY.h"

#include "AttitudeEstimation.h"
#include "MULTICOPTER.h"
#include "TWO_DOF_PID.h"
#include "SBUS_Handller.h"
#include "params.h"
#include "Madgwick/Madgwick.h"

#include "tim.h"
#include "usart.h"

#include <array>

#define RED_LED ledTim, TIM_CHANNEL_1
#define YELLOW_LED ledTim, TIM_CHANNEL_2
#define BLUE_LED ledTim, TIM_CHANNEL_3
uint16_t RED_LED_CHANNEL=TIM_CHANNEL_1;
uint16_t YELLOW_LED_CHANNEL=TIM_CHANNEL_2;
uint16_t BLUE_LED_CHANNEL=TIM_CHANNEL_3;
TIM_HandleTypeDef *ledTim = &htim1;


UART_HandleTypeDef *huartSbus = &huart5;
UART_HandleTypeDef *huartXbee = &huart4;
SBUS_HANDLE hsbus(lower,center, upper);

static std::function<void(void)> _icm20948Callback = [](){};

ICM20948_HAL *icm20948 = new ICM20948_HAL(&hi2c2, ICM20948::Address::LOW);
//ICM20948_HAL *icm20948 = new ICM20948_HAL(&hi2c1, ICM20948::Address::LOW);
ICM20948_USER icm20948User(icm20948);
void icm20948CallbackCalibration();
void icm20948Callback();


ElapsedTimer *elapsedTimer = new ElapsedTimer(&htim5, 1000000);
Madgwick attitudeEstimate(elapsedTimer,imuFrame);

multicopter::PARAMETER defaultParam(rollParam, rollParam, yawRateParam,altitudeParam, initialAltitudeControl, initialBankAngleLim, initialYawRateLim);
multicopter::MULTICOPTER *hmulticopter = new multicopter::MULTICOPTER(defaultParam,elapsedTimer);
multicopter::INPUT multicopterInput;

std::array<ESC_UTILITY_SINGLE*, 4> escSingle = {
    new ESC_UTILITY_SINGLE(&htim8,TIM_CHANNEL_1,1500,3000),
    new ESC_UTILITY_SINGLE(&htim8,TIM_CHANNEL_2,1500,3000),
    new ESC_UTILITY_SINGLE(&htim8,TIM_CHANNEL_3,1500,3000),
    new ESC_UTILITY_SINGLE(&htim8,TIM_CHANNEL_4,1500,3000),
};
ESC_UTILITY<4> esc(escSingle);

#endif /* INC_COMMON_H_ */
