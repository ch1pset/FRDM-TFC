/*
 * cup_car.c
 *
 *  Created on: Apr 15, 2019
 *      Author: Josh
 */

#include "cup_car.h"

int output[128];
int tmpImg[128];
int i;
int center = 64;
int offset, deltaOffset, ampDiff, deltaAmpDiff;

float error;
float l_att, r_att;	//attenuation for left/right motor
float max_strength;
float strength[2];

/**
 * Sets both motors to specified strength value
 * for specified time in milliseconds
 */
void DriveT(int delayMS, float strength)
{
	TFC_HBRIDGE_ENABLE;
	TFC_SetMotorPWM(strength,strength);
	TFC_Delay_mS(delayMS);
	TFC_HBRIDGE_DISABLE;
}

/**
 * Enables HBridge and sets each motor to specified strength values individually.
 * Adds a 10ms delay after sending signal.
 */
void Drive(float l_strength, float r_strength)
{
	TFC_HBRIDGE_ENABLE;
	TFC_SetMotorPWM(l_strength, r_strength);
	TFC_Delay_mS(10);
}

/**
 * Sets motors to 0 strength, centers servo position, and disables HBridge
 */
void Stop()
{
	TFC_SetMotorPWM(0,0);
	TFC_Delay_mS(10);
	TFC_SetServo(0,0);
	TFC_Delay_mS(10);
	TFC_HBRIDGE_DISABLE;
}

/**
 * Sets servo to specified position and adds 10ms delay after sending signal
 */
void Steer(float strength)
{
	TFC_SetServo(0, strength);
	TFC_Delay_mS(10);
}

/**
 * Sends raw line scan camera data to terminal
 */
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

/**
 * Processes line scan camera data by calculating the central difference of each pixel
 * from i = 5 to 123. Also finds edges by looking for the max and min values of the
 * curve.
 */
void procImage(int pValues[2], int pImage[128])
{
	int edge[2] = {0,0};
	int e_pos[2] = {0,127};
	memset(output, 0, sizeof(output));
	if(TFC_Ticker[0]>50 && LineScanImageReady==1)
	{
		TFC_Ticker[0] = 0;
		LineScanImageReady=0;
		
		for(i=2;i<=125;i++)
		{	
			output[i] =  (LineScanImage0[i+1] - LineScanImage0[i-1]) / 2;	//central difference for smoother output
//			output[i] = (LineScanImage0[i+1] - LineScanImage0[i]);
			
			if(output[i] > 2048 || output[i] < -2048) output[i] = 0;
			if(output[i] < 30 && output[i] > -30) output[i] = 0;
			
//			tmpImg[i] = output[i];
//			output[i] += pImage[i];
//			output[i] /= 2;
			
			if(output[i] >= edge[0])
			{
				edge[0] = output[i];
				e_pos[0] = i + 1;
			}
			if(output[i] < edge[1]) 
			{
				edge[1] = output[i];
				e_pos[1] = i + 1;
			}
			
//			pImage[i] = tmpImg[i];

//			TERMINAL_PRINTF("%d,", output[i]);  //print processed data to terminal, separated by commas
		}
		
//		TERMINAL_PRINTF("\n"); //print termination character(newline character)
		pidSteerControl(center, e_pos, edge, pValues);
	}
}

/**
 * Finds offset from center based on edge positions as well as the difference in amplitude
 * of the max and min values to calculate an error from -1.0 to 1.0
 * Error is used to steer the car.
 */
void pidSteerControl(int center, int e_pos[2], int edge[2], int pValues[2])
{
	//Calculate current center offset from true center
	offset = center - ((e_pos[0] + e_pos[1])/ 2);
//	if(offset < 1 && offset > -1) offset = 0;	//reduce oscillations, may need fine-tuning
	deltaOffset = offset - pValues[0]; //calculate offset delta from last sample
	
	//calculate current amplitude difference between edges
	ampDiff = edge[0] - (-1 * edge[1]);
//	if(ampDiff < 4 && ampDiff > -4) ampDiff = 0; //reduce oscillations, may need fine-tuning
	deltaAmpDiff = ampDiff- pValues[ 1]; //get amp-diff delta from last sample
	
	//Error = AmpErr + OffsetErr
	error = 1.0 * (ampDiff + deltaAmpDiff) / 2048; //2048 is maximum amp_diff/amp_delta
	error += 1.0 * (offset + deltaOffset) / 64; //64 is maximum offset/delta(128 pixels / 2 = 64 pixels)
	if(error > 1.0) error = 1.0;
	if(error < -1.0) error = -1.0;
	
	Steer(error * TFC_ReadPot(1)); //Servo does not have full 180 degrees of motion. Lower reduces turning radius.
	pidDriveControl(error); //Use error for drive control
	
	//store current offset and amp_diff in pValues array
	pValues[0] = offset;
	pValues[1] = ampDiff;
	
	//Print all data to terminal.
//	TERMINAL_PRINTF("CENTER=%4d OFF=%4d DELOFF=%4d AMPD=%4d  DELAMPD=%4d  ERROR=%4d\r", center, offset, deltaOffset, e_pos[0], e_pos[1], error);
}

/**
 * Uses error calculated from steer control function to determine when to slow down for corners
 * and to speed up individual wheels for faster turns.
 */
void pidDriveControl(float error)
{
	if(error < 0) //negative means turning right(oddly enough)
	{
		l_att = -0.5 * error;	//if turning right, left motor gains a maximum of +15% speed
		r_att = 0;
		strength[0] = max_strength - (-0.4 * error) + l_att; //left motor
		strength[1] = max_strength - (-0.4 * error) + r_att; //right motor
	}
	else //positive is turning left
	{
		r_att = 0.5 * error; //if turning left, right motor gains a maximum of +15% speed
		l_att = 0;
		strength[0] = max_strength - (0.4 * error) + l_att; //left motor
		strength[1] = max_strength - (0.4 * error) + r_att; //right motor
	}
	max_strength = TFC_ReadPot(0); //set max speed with pot 0
	
	//When turning, top speed is reduced by a maximum of -15%
	//with an additional maximum of +5% speed for the
	//specific motor used to overdrive one side of the car for
	//better steer control
	Drive(strength[0], strength[1]);
}

