/*
 * Servo.h
 *
 *  Created on: 1 Jun 2023
 *      Author: jondurrant
 */

#ifndef SERVO_SRC_SERVO_H_
#define SERVO_SRC_SERVO_H_

#include "pico/stdlib.h"

struct vector
{
	float x;
	float y;
	float z;
};

class Leg
{
public:
	Leg();
	virtual ~Leg();

	void caliberate_middle();
	bool inverse_kinematics(int leg, float phase_shift,float speed_multiple, float turning);
	void movement_loop();
	// int *read();
	int normalize(int value, int type);

};

#endif /* SERVO_SRC_SERVO_H_ */