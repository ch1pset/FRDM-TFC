/*
 * cup_car.h
 *
 *  Created on: Apr 15, 2019
 *      Author: Josh
 */

#ifndef CUP_CAR_H_
#define CUP_CAR_H_

#include "TFC\TFC.h"

void Drive(float l_strength, float r_strength);
void DriveT(int delayMS, float strength);
void Steer(float strength);
void pidSteerControl(int center, int e_pos[2], int edge[2], int pValues[2]);
void pidDriveControl(float error);
void Stop();

void printLineScanData(int i);
void procImage(int pValues[2], int pImage[128]);

#endif /* CUP_CAR_H_ */
