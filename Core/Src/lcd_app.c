/*
 * lcd_app.c
 *
 *  Created on: Aug 17, 2023
 *      Author: nraym
 */

#include "lcd_app.h"

void updateTempsMin(I2C_HandleTypeDef *handle, float tempmin) {
	HD44780_SetCursor(0, 0);
	char str[6];
	snprintf(str, sizeof(str), "%.1f", (float) tempmin);
	HD44780_PrintStr(str);
	HD44780_Display();
}

void updateTempsMax(I2C_HandleTypeDef *handle, float tempmax) {
	HD44780_SetCursor(0, 1);
	char str[6];
	snprintf(str, sizeof(str), "%.1f", (float) tempmax);
	HD44780_PrintStr(str);
	HD44780_Display();
}
void updateHighVoltage(I2C_HandleTypeDef *handle, float highvol) {
	HD44780_SetCursor(16, 0);
	char str[6];
	snprintf(str, sizeof(str), "%.2f", (float) highvol);
	HD44780_PrintStr(str);
	HD44780_Display();

}

void updateLowVoltage(I2C_HandleTypeDef *handle, float lowvol) {
	HD44780_SetCursor(16, 1);
	char str[6];
	snprintf(str, sizeof(str), "%.2f", (float) lowvol);
	HD44780_PrintStr(str);
	HD44780_Display();

}

void updateSpeed(I2C_HandleTypeDef *handle, float spd) {
	HD44780_SetCursor(10, 0);
	char str[6];
	snprintf(str, sizeof(str), "%3.0f", (float) spd * 3.6);
	HD44780_PrintStr(str);
	HD44780_Display();
}

void updateSOC(I2C_HandleTypeDef *handle, uint8_t soc) {
	HD44780_SetCursor(10, 1);
	char str[4];
	snprintf(str, sizeof(str), "%d", (uint8_t) soc);
	HD44780_PrintStr(str);
	HD44780_Display();
}

void updateAvgPower(I2C_HandleTypeDef *handle, int avgpow) {
	HD44780_SetCursor(0, 2);
	char str[5];
	snprintf(str, sizeof(str), "%d", (int) avgpow);
	HD44780_PrintStr(str);
	HD44780_Display();
}

void updateInstPower(I2C_HandleTypeDef *handle, float instpow) {
	HD44780_SetCursor(16, 2);
	char str[5];
	snprintf(str, sizeof(str), "%4.0f", (float) instpow);
	HD44780_PrintStr(str);
	HD44780_Display();
}

void updateMessage(char *message) {
	HD44780_SetCursor(0, 3);
	HD44780_PrintStr(message);
	HD44780_Display();
}

void SetUpDisplay(I2C_HandleTypeDef *handle) {
	HAL_I2C_IsDeviceReady(handle, DEVICE_ADDR, 2, 10);

	HD44780_SetCursor(5, 0);
	HD44780_PrintStr("|V:      |");

	HD44780_SetCursor(5, 1);
	HD44780_PrintStr("|%:      |");

	HD44780_SetCursor(5, 2);
	HD44780_PrintStr("|        |");

	HD44780_Display();
}

