#include "derivative.h" /* include peripheral declarations */
#include "TFC\TFC.h"
#include "cup_car.h"

uint32_t t, i=0;

void toggleLED(uint8 mode, unsigned int msHold)
{
	switch(mode)
	{
	case 0:
		TFC_BAT_LED0_OFF;
		break;
	case 1:
		TFC_BAT_LED0_ON;
		break;
	}
	TFC_Delay_mS(msHold);
}

int pValues[2];
int image[128];
int driving = 0;
int main(void)
{
	TFC_Init();
	t = 0;
	i = 0;
	memset(image, 0, sizeof(image));
	Stop();
	while (!(TFC_PUSH_BUTTON_0_PRESSED))
	{ 	
		toggleLED(1, 500);
		toggleLED(0, 500);
	}
	TFC_Delay_mS(1000);
	for(;;)
	{
		TFC_Task(); //keep this inside for loop, needed for terminal
		if(TFC_PUSH_BUTTON_1_PRESSED) //Button B 
		{
			if(driving)
			{
				driving = 0;
			}
			else
			{
				driving = 1;
			}
			TFC_Delay_mS(250);
		}
		if(driving)
		{
			switch((TFC_GetDIP_Switch()))
			{
				case 1: //Switch labeled 1
					procImage(pValues, image); 
					break;
				default: //No Switches
					Stop();
					printLineScanData(i); 
					break;
			}
		}
		else Stop();
	}
	
	return 0;
}
