/*
 * cup_car.h
 *
 *  Created on: Apr 15, 2019
 *      Author: Josh
 */

#ifndef CUP_CAR_H_
#define CUP_CAR_H_

#include "TFC\TFC.h"

typedef enum DIRECTION
{
	CENTER = 0x1,
	LEFT = 0x02,
	CENTER_L = 0x03,
	RIGHT = 0x04,
	CENTER_R = 0x05
} DIR;

void Drive(float strength);
void DriveT(int delayMS, float strength);
void Steer(DIR d, float strength);
void Stop();

//void figure8();

void printLineScanData(int t, int i);

void procImage(int t,int i);


#endif /* CUP_CAR_H_ */
