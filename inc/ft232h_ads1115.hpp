#pragma once

#include "ftd2xx.h"
#include "libmpsse_i2c.h"
#include "ADS1115.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <iostream>
#include <chrono>
#include <thread>

#define ADS1115_DEVICE_ADDRESS 0x48

namespace ADS1115
{
	class FT232H_ADS1115
	{
	private:
		const UCHAR address;

	public:
		ADC adc;

		FT232H_ADS1115();
		~FT232H_ADS1115();

		void ads_set_conf();
		void ads_get_conf();
		void ads_read_mes();
	};
}

