/*
 * lcd_app.h
 *
 *  Created on: Aug 17, 2023
 *      Author: nraym
 */

#ifndef INC_LCD_APP_H_
#define INC_LCD_APP_H_

#include "stm32f1xx_hal.h"
#include "lcd_driver.h"
#include <stdio.h>


void updateTempsMin(I2C_HandleTypeDef*, float);//
void updateTempsMax(I2C_HandleTypeDef*, float);//
void updateHighVoltage(I2C_HandleTypeDef*, float);//
void updateLowVoltage(I2C_HandleTypeDef*, float);//
void updateSpeed(I2C_HandleTypeDef*, float);//
void updateSOC(I2C_HandleTypeDef*, uint8_t);//
void updateAvgPower(I2C_HandleTypeDef*, int);//
void updateInstPower(I2C_HandleTypeDef*, int);
void updateMessage(char* message);
void SetUpDisplay(I2C_HandleTypeDef*);//


#endif /* INC_LCD_APP_H_ */
