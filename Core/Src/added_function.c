#include"main.h"

void Calul_Putere_Lap(float bus_pow, uint8_t Power_Button_Pressed, float* Power_Sum_Print)
{
	static double Power_Sum = 0;

	if(Power_Button_Pressed == 1)
	{
		if( Power_Sum < 0xFFFFFFFFFFFFEC77 )
		{
			Power_Sum += bus_pow * 0.1;
		}
	}
	else
	{
		if(Power_Sum < 0xFFFFFFFFFFFFEC77)
		{
				*Power_Sum_Print =  Power_Sum / 3600;
				Power_Sum = 0;
		}
		else if(Power_Sum >= 0xFFFFFFFFFFFFEC77) //OVERFLOW
		{
			*Power_Sum_Print = 0;
			Power_Sum = 0;
		}
		Power_Button_Pressed == 1;
	}
}




