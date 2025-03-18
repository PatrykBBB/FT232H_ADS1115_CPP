#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <iostream>
#include <chrono>
#include <thread>

#ifndef _WIN32
#include <unistd.h>
#include <dlfcn.h>	/*for dlopen() & dlsym()*/
#endif

#include "ft232h_ads1115.hpp"

using namespace std;

int main(void)
{
	ADS1115::FT232H_ADS1115* ads = new ADS1115::FT232H_ADS1115();

	ads->ads_read_mes();

	delete ads;

	return 0;
}
