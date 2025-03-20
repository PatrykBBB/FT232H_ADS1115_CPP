#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <iostream>
#include <chrono>
#include <thread>

#include "ft232h_ads1115.hpp"

using namespace std;

int main(void)
{
	ADS1115::FT232H_ADS1115* ads = new ADS1115::FT232H_ADS1115();

	ads->adc.i2c_get_device_info();
	ads->adc.i2c_get_channel_info();
	ads->adc.i2c_open_connection();

	ads->read_mes();

	delete ads;

	return 0;
}
