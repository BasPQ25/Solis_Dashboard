#include "main.h"

void Calcul_Putere_Lap(float* bat_pow, uint8_t* p_Power_Button_Pressed, float* p_Power_Sum_Print)
{
	static float Power_Sum = 0;

	if( *p_Power_Button_Pressed == 1 )
	{
		Power_Sum += *bat_pow * 0.1;
	}
	else
	{
		*p_Power_Sum_Print = Power_Sum / 3600;
		Power_Sum = 0;
		*p_Power_Button_Pressed = 1;
	}
}





