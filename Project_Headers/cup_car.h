/*
 * cup_car.h
 *
 *  Created on: Apr 15, 2019
 *      Author: Josh
 */

#ifndef CUP_CAR_H_
#define CUP_CAR_H_

#include "TFC\TFC.h"
#define SAMPLES 4

typedef enum DIRECTION
{
	CENTER = 0x1,
	LEFT = 0x02,
	CENTER_L = 0x03,
	RIGHT = 0x04,
	CENTER_R = 0x05
} DIR;

typedef int img[128];


void Drive(float strength);
void DriveT(int delayMS, float strength);
void Steer(DIR d, float strength);
int pidSteerControl(int center, int e_pos[2], int pOffset);
void Stop();

void printLineScanData(int i);
int procImage(int pOffset);
int avgImage(int i, int s);

#endif /* CUP_CAR_H_ */
