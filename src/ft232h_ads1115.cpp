#include "ft232h_ads1115.hpp"

using namespace ADS1115;
using namespace std;

FT232H_ADS1115::FT232H_ADS1115() : address(ADS1115_DEVICE_ADDRESS), adc(address)
{
	if (ADS1115::is_valid_ads_address(address) != Error::NONE)
	{
		cout << "BAD_ADDRESS" << endl;
	}

	set_conf();
	adc.i2c_init(I2C_CLOCK_STANDARD_MODE, 10, 0, 0);
}

FT232H_ADS1115::~FT232H_ADS1115()
{
	adc.i2c_close_connection();
	adc.i2c_clear();
}

void FT232H_ADS1115::open_connection()
{
	adc.i2c_open_connection();
	adc.i2c_write_adc_config();
}

void FT232H_ADS1115::set_conf()
{
	auto config_fsr = FullScaleRange::FSR_6_144V;
	auto config_dr = DataRate::SPS_860;
	auto config_mux = Multiplex::AIN0;
	auto config_conv = ConversionMode::Continuous;

	cout << "Setting FSR to +-" << config_fsr << endl;
	cout << "Setting DR to " << config_dr << endl;
	cout << "Setting MUX to " << config_mux << endl;
	cout << "Setting CONV to " << config_conv << endl;

	adc.set_fsr(config_fsr);
	adc.set_data_rate(config_dr);
	adc.set_multiplexing(config_mux);
	adc.set_conversion_mode(config_conv);
}

void FT232H_ADS1115::get_conf()
{
	cout << "ADC Configuration" << endl;
	cout << "\tfsr             : " << adc.get_fsr() << endl;
	cout << "\tmultiplexing    : " << adc.get_multiplexing() << endl;
	cout << "\tdata rate       : " << adc.get_data_rate() << endl;
	cout << "\tconversion mode : " << adc.get_conversion_mode() << endl;
}

void FT232H_ADS1115::read_mes()
{
	for (int i = 0; i < 10; i++)
	{
		//read conv register in continous mode
		adc.i2c_read_measurement_continous();
		this_thread::sleep_for(chrono::milliseconds(2));
	}
}