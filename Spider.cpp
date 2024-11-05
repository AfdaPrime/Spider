/*
 * main.cpp
 *
 *  Created on: 9 Jun 2022
 *      Author: jondurrant
 */

#include <stdio.h>
#include "Leg.h"
#include <iostream>

#define DELAY 1000 // in microseconds
using std::cout;
using std::endl;

void test()
{
    cout << "interupt" << endl;
}

int main()
{

    stdio_init_all();
    Leg leg;

    leg.movement_loop();
    // leg.read();

    // leg.caliberate_middle();

    // while (1)
    // {

    //     current_time = to_us_since_boot(get_absolute_time());
    //     if (current_time - prev_time >= 5000)
    //     { // start 5ms timed loop
    //         prev_time = current_time;
    //         // cout << flag << " outside top" << endl;
    //         if (flag == 0 && pass)
    //         {
    //             // cout << flag << " inside top" << endl;
    //             leg.setter_coor(40.0, 0.0, 0.0);
    //             flag = 1;
    //             prev_step_time = to_us_since_boot(get_absolute_time());
    //             pass = false;
    //         }
    //         else if (flag == 1 && pass)
    //         {

    //             // cout << flag << " inside mid" << endl;
    //             leg.setter_coor(0.0, 0.0, 0.0);
    //             flag = 2;
    //             prev_step_time = to_us_since_boot(get_absolute_time());
    //             pass = false;
    //         }
    //         else if (flag == 2 && pass)
    //         {

    //             // cout << flag << " inside bottom" << endl;
    //             leg.setter_coor(-40.0, 0.0, 0.0);
    //             flag = 0;
    //             prev_step_time = to_us_since_boot(get_absolute_time());
    //             pass = false;
    //         }

    //         pass = leg.inverse_kinematics(0);
    //     }
    // }
}