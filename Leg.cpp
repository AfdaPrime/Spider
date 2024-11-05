#include "Leg.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include <cmath>
#include <stdio.h>
#include <iostream>
#include "pico/multicore.h"
using std::cout;
using std::endl;

const int rotate_0 = 500;
const int rotate_180 = 2500;
// define servos' ports
const int servo_pin[4][3] = {{6, 7, 8}, {10, 11, 12}, {21, 20, 19}, {18, 17, 16}};
volatile int remote_input[7];
mutex_t buffer_mutex;

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

// define PI for calculation
const float pi = 3.1415926;

float current_t = 0;
uint64_t current_time = to_us_since_boot(get_absolute_time());
uint64_t prev_time = to_us_since_boot(get_absolute_time());

Leg::Leg()
{
	gpio_init(25);
	gpio_set_dir(25, GPIO_OUT);
	// setup receiver - ibus(uart)
	gpio_set_function(5, UART_FUNCSEL_NUM(uart1, 5));
	uart_init(uart1, 115200);

	// setup pwm
	int rows = sizeof(servo_pin) / sizeof(servo_pin[0]);
	int columns = sizeof(servo_pin[0]) / sizeof(servo_pin[0][0]);

	for (int i = 0; i < 4; i++)
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
void read()
{
	// gpio_set_function(4, UART_FUNCSEL_NUM(uart1, 4));

	while (1)
	// for (int z = 0; z < 10; z++)
	{
		mutex_enter_blocking(&buffer_mutex);
		uint8_t buffer[31];
		char c = uart_getc(uart1);
		if (c == 0x20)
		{
			uart_read_blocking(uart1, buffer, 31);
			uint32_t checksum = 0Xffdf;

			for (int i = 0; i < 29; i++)
			{
				checksum -= buffer[i];
				if (checksum == (buffer[30] << 8) | buffer[29])
				{
					remote_input[0] = 1; // status 1 = success
					for (int i = 1; i <= 6; i++)
					{
						remote_input[i] = (int16_t)(buffer[(i * 2) - 1] + (buffer[i * 2] << 8));
					}
				}
				else
				{
					remote_input[0] = -2; // Checksum error
				}
			}
		}
		else
		{
			remote_input[0] = -1;
		}
		remote_input[0] = -1;
		mutex_exit(&buffer_mutex);
		// cout << remote_input[2] << endl;
	}

	// return remote_input;
}
void Leg::caliberate_middle()
{

	while (1)
	{
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				pwm_set_gpio_level(servo_pin[i][j], 1500);
				sleep_ms(1000);
			}
		}
	}
}

void Leg::movement_loop()
{
	int *buffer;
	mutex_init(&buffer_mutex);
	multicore_launch_core1(read);
	gpio_put(25, 1);
	while (1)
	{

		// buffer = read();
		mutex_enter_blocking(&buffer_mutex);
		current_time = to_us_since_boot(get_absolute_time());

		if (current_time - prev_time >= 5000)
		{ // start 5ms timed loop
			prev_time = current_time;

			inverse_kinematics(0, pi / 2, normalize(remote_input[2], 1), normalize(remote_input[1], 1));
			inverse_kinematics(1, 0, normalize(remote_input[2], 1), normalize(remote_input[1], 1));
			inverse_kinematics(2, 3 * pi / 2, normalize(remote_input[2], 1), normalize(remote_input[1], 1));
			inverse_kinematics(3, pi, normalize(remote_input[2], 1), normalize(remote_input[1], 1));
			if (1 + current_t < 0.001)
			{
				current_t = 1;
			}
			else if (1 - current_t < 0.001)
			{
				current_t = 0;
			}
			else
			{
				float speed_multiple = (float)normalize(remote_input[2], 1) / 100 * 1;
				current_t += 0.005 * speed_multiple;
				// cout << current_t << " " << speed_multiple << endl;
			}
		}
		mutex_exit(&buffer_mutex);
	}
}

float prev = 0;

bool Leg::inverse_kinematics(int leg, float phase_shift, float speed_multiple, float turning)
{
	// float x;
	// float x = (0.5 - current_t) * (40 * 0.01) + current_t * (prev * 0.99);
	// if (fabs(current_t) <= 0.5)
	// {
	// 	x = (((0.5 - fabs(current_t)) / 0.5) * 40) + (fabs(current_t) / 0.5 * (-40));
	// }
	// else
	// {

	// 	x = ((1 - ((fabs(current_t) - 0.5) / 0.5)) * (-40)) + ((fabs(current_t) - 0.5) / 0.5 * (40));
	// }
	float x = 40 * sin(2 * pi * fabs(current_t) + phase_shift);

	if (x < 0)
	{
		x = 0;
	}
	float y = ((1 - fabs(current_t)) * (0 + y_default)) + (fabs(current_t) * ((turning / 20) + y_default));
	float z;
	z = 40 * sin(2 * pi * fabs(current_t) + phase_shift + pi / 2);
	if (leg == 0 || leg == 2)
	{
		if (z < 0)
		{
			z = z_default;
		}
		else
		{
			z += z_default;
		}
	}
	else if (leg == 1 || leg == 3)
	{
		if (z < 0)
		{
			z = fabs(z) + z_default;
		}
		else
		{
			z = z_default;
		}
	}

	// cout << "X " << x << " y " << y << " z " << z << " current time " << current_t << endl;
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

	servo_write(leg, coxa_duty, femur_duty, tibia_duty);

	return (fabs(current_t - 0.5) < 0.1);
}

void Leg::servo_write(int leg, double coxa, double femur, double tibia)
{
	cout << femur << " " << tibia << endl;
	if (leg == 0)
	{
		pwm_set_gpio_level(servo_pin[leg][0], coxa);
		pwm_set_gpio_level(servo_pin[leg][1],  2500 - (femur - 500));
		pwm_set_gpio_level(servo_pin[leg][2], 2500 - (tibia - 500));
	}
	if (leg == 1)
	{
		pwm_set_gpio_level(servo_pin[leg][0], 2500 - (coxa - 500));
		pwm_set_gpio_level(servo_pin[leg][1], femur);
		pwm_set_gpio_level(servo_pin[leg][2], tibia);
	}
	else if (leg == 2)
	{

		pwm_set_gpio_level(servo_pin[leg][0], 2500 - (coxa - 500));
		pwm_set_gpio_level(servo_pin[leg][1], femur);
		pwm_set_gpio_level(servo_pin[leg][2], tibia);
	}
	else
	{
		pwm_set_gpio_level(servo_pin[leg][0], coxa);
		pwm_set_gpio_level(servo_pin[leg][1],  2500 - (femur - 500));
		pwm_set_gpio_level(servo_pin[leg][2], 2500 - (tibia - 500));
	}
}
int Leg::normalize(int value, int type)
{
	if (type == 2)
	{
		return (value - 1000) / 10;
	}
	else
	{
		return (value - 1500) / 5;
	}
}