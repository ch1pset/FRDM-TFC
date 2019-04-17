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
	TFC_Delay_mS(20);
	TFC_SetServo(0,0);
	TFC_Delay_mS(20);
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
//	TFC_Delay_mS(20);
}

void printLineScanData(int i)
{
	if(TFC_Ticker[0]>50 && LineScanImageReady==1)
	{
		TFC_Ticker[0] = 0;
		LineScanImageReady=0;
		
		for(i=0;i<128;i++)
		{
			 TERMINAL_PRINTF("%d,",LineScanImage0[i]);
		}
		TERMINAL_PRINTF("\n");
	}
}

int procImage(int pOffset)
{
	int output[128];
	int sobel[5] = {-1,-2,0,2,1};
	int j;
	int i;
	int edge[2] = {0,0};
	int e_pos[2] = {5,123};
	int center = 63;
	memset(output, 0, sizeof(output));
	
	if(TFC_Ticker[0]>50 && LineScanImageReady==1)
	{
		TFC_Ticker[0] = 0;
		LineScanImageReady=0;
		
		for(i=0;i<128;i++)
		{
			output[i] = 0;
			if(i >= 5 || i <= 123)
			{
				for(j = 0; j < 5; j++)
					output[i] += sobel[j] * LineScanImage0[i + j - 2];

//				output[i] = LineScanImage0[i+1] - LineScanImage0[i];	//calc differnce(1st derivative)
				
				if(output[i] > 5000) output[i] = 0;		//set max value
				if(output[i] < -5000) output[i] = 0;	//set min value
				if(output[i] <= 512 && output[i] >= -512) output[i] = 0;	//reduce low dB noise and false edges
				
				if(output[i] > edge[0] && output[i] > 1000) //arbitrary threshold of 1000
				{
					edge[0] = output[i];
					e_pos[0] = i;
				}
				if(output[i] < edge[1] && output[i] < 500) 
				{
					edge[1] = output[i];
					e_pos[1] = i;
				}
			}
//			TERMINAL_PRINTF("%d,", output[i]);
		}
//		TERMINAL_PRINTF("\n");
		return pidSteerControl(center, e_pos, pOffset);
	}
	return pOffset;
}

int pidSteerControl(int center, int e_pos[2], int pOffset)
{
	int offset = center - ((int)((e_pos[0] + e_pos[1])/ 2));
	int delta = offset - pOffset;
	TFC_SetServo(0, 1.0 * (offset + delta) / 64);
//	TERMINAL_PRINTF("CENTER = %4d OFF = %4d DEL = %4d LEDGE = %4d  REDGE = %4d\r", center, offset, delta, e_pos[0], e_pos[1]);
	return offset;
}

int images[SAMPLES][128];

int avgImage(int i, int s)
{
	if(s < SAMPLES)
	{
		if(TFC_Ticker[0]>50 && LineScanImageReady==1)
		{
			TFC_Ticker[0] = 0;
			LineScanImageReady=0;
			for(i = 0; i < 128; i++)
			{
				images[s][i] = LineScanImage0[i];
			}
			s++;
		}
	}
	else
	{
		int output[128];
		memset(output, 0, sizeof(output));
		for(i = 0; i < 128; i++)
		{
			int n;
			for(n = 0; n < 4; n++)
			{
				output[i] += images[n][i];
			}
			output[i] = (output[i] >> (SAMPLES / 2));
			TERMINAL_PRINTF("%d,",output[i]);
		}
		s = 0;
		TERMINAL_PRINTF("\n");
	}
	return s;
}
