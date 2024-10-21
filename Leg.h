/*
 * Servo.h
 *
 *  Created on: 1 Jun 2023
 *      Author: jondurrant
 */

#ifndef SERVO_SRC_SERVO_H_
#define SERVO_SRC_SERVO_H_

#include "pico/stdlib.h"

class Leg
{
public:
	Leg();
	virtual ~Leg();

	void loop();
	bool inverse_kinematics(int leg);

	void setter_coor(float x_1, float y_1, float z_1, float x_2, float y_2, float z_2);
};

#endif /* SERVO_SRC_SERVO_H_ */