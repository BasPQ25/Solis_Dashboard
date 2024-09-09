#include"main.h"

void Feedback_Erori()
{

}
void Calul_Putere_Lap(float* p_bus_power)
{
	static uint32_t power_samples = 0;
	static int64_t Power_Sum = 0;

	Power_Sum += *p_bus_power;
	power_samples++;



}

/*
void Uart_Transmitter(UART_HandleTypeDef huart2, float Print_Var)
{
	float dummy = 0;
	static char Tx_Buffer[20];
	static uint8_t counter = 0;

	dummy = Print_Var;

	snprintf(Tx_Buffer, sizeof(Tx_Buffer),"%.2f \r \n", dummy);
	if(++counter == 10)
	{
		HAL_UART_Transmit(&huart2, (uint8_t*)Tx_Buffer, strlen(Tx_Buffer), HAL_MAX_DELAY);
		counter = 0;
		memset(Tx_Buffer, 0, sizeof(Tx_Buffer));
	}
}
*/


