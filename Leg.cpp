#include "Leg.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <cmath>
#include <stdio.h>
#include <iostream>

using std::cout;
using std::endl;

const int rotate_0 = 500;
const int rotate_180 = 2500;
// define servos' ports
const int servo_pin[1][3] = {{2, 3, 4}};
// const int servo_pin[4][3] = { {2, 3, 4}, {5, 6, 7}, {8, 9, 10}, {11, 12, 13} };
/* Size of the robot ---------------------------------------------------------*/
const float length_femur = 55;
const float length_tibia = 77.5;
const float length_c = 27.5;
const float length_side = 71;
const float z_absolute = -28;
/* Constants for movement ----------------------------------------------------*/
const float z_default = -50, z_up = -30, z_boot = z_absolute;
const float x_default = 62, x_offset = 0;
const float y_start = 0, y_step = 40;
const float y_default = x_default;
/* variables for movement ----------------------------------------------------*/
volatile float site_now[4][3];	  // real-time coordinates of the end of each leg
volatile float site_expect[4][3]; // expected coordinates of the end of each leg
float temp_speed[4][3];			  // each axis' speed, needs to be recalculated before each movement
float move_speed;				  // movement speed
float speed_multiple = 1;		  // movement speed multiple
const float spot_turn_speed = 4;
const float leg_move_speed = 8;
const float body_move_speed = 3;
const float stand_seat_speed = 1;
volatile int rest_counter; //+1/0.02s, for automatic rest
// functions' parameter
const float KEEP = 255;
// define PI for calculation
const float pi = 3.1415926;
/* Constants for turn --------------------------------------------------------*/
// temp length
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
// site for turn
const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;
/* ---------------------------------------------------------------------------*/

double coxa_smooth_prev = 0.0;
double femur_smooth_prev = 0.0;
double tibia_smooth_prev = 0.0;

float x1_ik;
float y1_ik;
float z1_ik;
float x2_ik;
float y2_ik;
float z2_ik;
float current_t = 0;

Leg::Leg()
{

	int rows = sizeof(servo_pin) / sizeof(servo_pin[0]);
	int columns = sizeof(servo_pin[0]) / sizeof(servo_pin[0][0]);

	for (int i = 0; i < 1; i++)
	{
		for (int j = 0; j < 3; j++)
		{

			uint pin = servo_pin[i][j];

			gpio_init(pin);
			gpio_set_function(pin, GPIO_FUNC_PWM);
			pwm_set_gpio_level(pin, 0);

			uint slice_num = pwm_gpio_to_slice_num(pin);

			uint32_t clk = clock_get_hz(clk_sys);
			uint32_t div = clk / (20000 * 50);

			// Check div is in range
			if (div < 1)
			{
				div = 1;
			}
			if (div > 255)
			{
				div = 255;
			}

			pwm_config config = pwm_get_default_config();
			pwm_config_set_clkdiv(&config, (float)div);

			// Set wrap so the period is 20 ms
			pwm_config_set_wrap(&config, 20000);

			pwm_init(slice_num, &config, true);
		}
	}
}

Leg::~Leg()
{
	// TODO Auto-generated destructor stub
}

void Leg::loop()
{

	while (1)
	{
		pwm_set_gpio_level(3, 1500);
		sleep_ms(1000);
		pwm_set_gpio_level(3, 2000);
		sleep_ms(1000);
	}
}


void Leg::setter_coor(float x_1, float y_1, float z_1, float x_2, float y_2, float z_2)
{

	x1_ik = x_1;
	y1_ik = y_1;
	z1_ik = z_1;

	x2_ik = x_2;
	y2_ik = y_2;
	z2_ik = z_2;
}

bool Leg::inverse_kinematics(int leg)
{

	float x = ((0.5 - current_t) * x1_ik) + (current_t * x2_ik);
	float y = ((0.5 - current_t) * (y1_ik + +y_default)) + (current_t * (y2_ik + +y_default));
	float z = ((0.5 - current_t) * (z1_ik + z_default)) + (current_t * (z2_ik + z_default));

	double j_1 = atan(x / y) * (180 / pi);

	double h = sqrt((y * y) + (x * x));

	double l = sqrt((h * h) + (z * z));

	double j_3 = acos(((length_femur * length_femur) + (length_tibia * length_tibia) - (l * l)) / (2 * length_femur * length_tibia)) * (180 / pi);

	double b = acos(((length_femur * length_femur) + (l * l) - (length_tibia * length_tibia)) / (2 * l * length_femur)) * (180 / pi);

	double a = atan(z / h) * (180 / pi);

	double j_2 = b + a;

	double coxa_duty = (((rotate_180 - rotate_0) / 180) * (90 - j_1)) + rotate_0;
	double femur_duty = (((rotate_180 - rotate_0) / 180) * (90 - j_2)) + rotate_0;
	double tibia_duty = (((rotate_180 - rotate_0) / 180) * (j_3)) + rotate_0;

	double coxa_smooth = (coxa_duty * 0.05) + (coxa_smooth_prev * 0.95);
	double femur_smooth = (femur_duty * 0.05) + (femur_smooth_prev * 0.95);
	double tibia_smooth = (tibia_duty * 0.05) + (tibia_smooth_prev * 0.95);

	coxa_smooth_prev = coxa_smooth;
	femur_smooth_prev = femur_smooth;
	tibia_smooth_prev = tibia_smooth;

	pwm_set_gpio_level(servo_pin[leg][0], coxa_smooth);
	pwm_set_gpio_level(servo_pin[leg][1], femur_smooth);
	pwm_set_gpio_level(servo_pin[leg][2], tibia_smooth);

	if (fabs(current_t - 0.5) < 0.1)
	{
		current_t = 0;
	}
	else
	{
		current_t += 0.005 * speed_multiple;
	}

	return (fabs(current_t - 0.5) < 0.1);
}