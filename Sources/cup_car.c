/*
 * cup_car.c
 *
 *  Created on: Apr 15, 2019
 *      Author: Josh
 */

#include "cup_car.h"


void DriveT(int delayMS, float strength)
{
	TFC_HBRIDGE_ENABLE;
	TFC_SetMotorPWM(strength,strength);
	TFC_Delay_mS(delayMS);
	TFC_HBRIDGE_DISABLE;
}

void Drive(float strength)
{
	TFC_HBRIDGE_ENABLE;
	TFC_SetMotorPWM(strength, strength);
}

void Stop()
{
	TFC_SetMotorPWM(0,0);
	TFC_HBRIDGE_DISABLE;
}

void Steer(DIR d, float strength)
{
	switch(d)
	{
		case LEFT: 
			TFC_SetServo(0, -1.0 * strength);
			break;
		case RIGHT:
			TFC_SetServo(0, strength);
			break;
		case CENTER:
			TFC_SetServo(0,0);
			break;
		case CENTER_L:
			TFC_SetServo(0, +0.05);
			break;
		case CENTER_R:
			TFC_SetServo(0, -0.05);
			break;
	}
	TFC_Delay_mS(20);
}

//void figure8()
//{
//
//	int RESET = 0;
//	int LEFT = -1;
//	int RIGHT = 1;
//	
//	while(!TFC_PUSH_BUTTON_0_PRESSED) continue;
//	TFC_Delay_mS(3000);
//	
//	TFC_Task();
//	
//	
//	SetServoDirection(LEFT);
//	DriveT(3000, 0.55);
//	TFC_SetServo(0,0.05);
//	TFC_Delay_mS(3000);
//	DriveT(4000, 0.35);
//
//	while(!TFC_PUSH_BUTTON_0_PRESSED) continue;
//	TFC_Delay_mS(3000); 
//	
//	SetServoDirection(RIGHT);
//	DriveT(3000, 0.55 );
//	TFC_SetServo(0,-0.05);
//	TFC_Delay_mS(3000);
//	DriveT(4000, 0.35);
//}

void printLineScanData(int t, int i)
{
	if(TFC_Ticker[0]>100 && LineScanImageReady==1)
	{
		TFC_Ticker[0] = 0;
		LineScanImageReady=0;
		if(t==0)
			t=3;
		else
			t--;
		TFC_SetBatteryLED_Level(t);
		
		for(i=0;i<128;i++)
		{
			 TERMINAL_PRINTF("%X,",LineScanImage0[i]);
		}
		TERMINAL_PRINTF("\r");
	}
}

void procImage(int t, int i)
{
	int output[128];
	int sobel[5] = {1,2,0,-2,-1};
	int j;

	memset(output, 0, sizeof(output));
	
	if(TFC_Ticker[0]>100 && LineScanImageReady==1)
	{
		TFC_Ticker[0] = 0;
		LineScanImageReady=0;
		if(t==0)
			t=3;
		else
			t--;
		 TFC_SetBatteryLED_Level(t);
		 for(i=2;i<125;i++)
		 {
			 for(j = 0; j < 5; j++)
			 {
				 output[i] += sobel[j] * (int)LineScanImage0[i + j - 2];
			 }
			 TERMINAL_PRINTF("%X,", output[i]);
		 }
		TERMINAL_PRINTF("\r");
	}
}
