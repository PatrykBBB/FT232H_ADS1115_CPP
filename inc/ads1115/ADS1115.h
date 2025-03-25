#pragma once

#include "error.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <chrono>
#include <thread>

namespace ADS1115
{
    #define APP_CHECK_STATUS(exp) {if (exp!=FT_OK){printf(" status(0x%x) != FT_OK\n", exp);}else{;}};

    enum RegisterAddress : uint8_t
    {
        Conversion = 0b00,
        Config     = 0b01,
        Lo_thresh  = 0b10,
        Hi_thresh  = 0b11,
    };

    enum AddressPin : uint8_t
    {
        GND = 0b1001000,
        VDD = 0b1001001,
        SDA = 0b1001010,
        SCL = 0b1001011,
    };

    enum FullScaleRange : uint16_t
    {
        FSR_6_144V  = (0b000) << 9, // : FSR = ±6.144 V(1)
        FSR_4_096V  = (0b001) << 9, // : FSR = ±4.096 V(1)
        FSR_2_048V  = (0b010) << 9, // : FSR = ±2.048 V (default)
        FSR_1_024V  = (0b011) << 9, // : FSR = ±1.024 V
        FSR_0_512V  = (0b100) << 9, // : FSR = ±0.512 V
        FSR_0_256V  = (0b101) << 9, // : FSR = ±0.256 V
        FSR_0_256V1 = (0b110) << 9, // : FSR = ±0.256 V
        FSR_0_256V2 = (0b111) << 9, // : FSR = ±0.256 V
    };

    enum Multiplex : uint16_t
    {
        AIN0_AIN1 = (0b000) << 12, // : AINP = AIN0 and AINN = AIN1 (default)
        AIN0_AIN3 = (0b001) << 12, // : AINP = AIN0 and AINN = AIN3
        AIN1_AIN3 = (0b010) << 12, // : AINP = AIN1 and AINN = AIN3
        AIN2_AIN3 = (0b011) << 12, // : AINP = AIN2 and AINN = AIN3
        AIN0      = (0b100) << 12, // : AINP = AIN0 and AINN = GND
        AIN1      = (0b101) << 12, // : AINP = AIN1 and AINN = GND
        AIN2      = (0b110) << 12, // : AINP = AIN2 and AINN = GND
        AIN3      = (0b111) << 12, // : AINP = AIN3 and AINN = GND
    };

    enum DataRate : uint16_t
    {
        SPS_8   = (0b000) << 5, // : 8 SPS
        SPS_16  = (0b001) << 5, // : 16 SPS
        SPS_32  = (0b010) << 5, // : 32 SPS
        SPS_64  = (0b011) << 5, // : 64 SPS
        SPS_128 = (0b100) << 5, // : 128 SPS (default)
        SPS_250 = (0b101) << 5, // : 250 SPS
        SPS_475 = (0b110) << 5, // : 475 SPS
        SPS_860 = (0b111) << 5, // : 860 SPS
    };

    /**
    @author     lokraszewski
    @date       05-Sep-2018
    @brief      Comparator queue

    @details    These bits perform two functions. When set to 11, the comparator is
                disabled and the ALERT/RDY pin is set to a high-impedance state.
                When set to any other value, the ALERT/RDY pin and the comparator
                function are enabled, and the set value determines the number of
                successive conversions exceeding the upper or lower threshold
                required before asserting the ALERT/RDY pin. These bits serve no
                function on the ADS1113.
    */
    enum ComparatorQueue : uint16_t
    {
        ONE_CONVERSION   = 0b00, //! Assert after one conversion
        TWO_CONVERSIONS  = 0b01, //! Assert after two conversions
        FOUR_CONVERSIONS = 0b10, //! Assert after four conversions
        DISABLE          = 0b11, //! Disable comparator and set ALERT/RDY pin to high-impedance (default)
    };

    enum ConversionMode : uint16_t
    {
        Continuous = 0,
        SingleShot = (1 << 8),
    };

    enum Config : uint16_t
    {
        OS = (1 << 15),                 /* When writing: 0 : No effect 1 : Start a single conversion (when in
                                        power-down state).  When reading: 0 : Device is currently
                                        performing a conversion 1 : Device is not currently performing a
                                        conversion */
        COMPARATOR_MODE = (1 << 4),     /* Comparator mode (ADS1114 and ADS1115 only) This bit configures the
                                        comparator operating mode. This bit serves no function on the
                                        ADS1113. 0 : Traditional comparator (default) 1 : Window comparator */
        COMPARATOR_POL = (1 << 3),      /* Comparator polarity(ADS1114 and ADS1115 only) This bit controls the
                                        polarity of the ALERT pin.This bit serves no function on the
                                        ADS1113. 0 : Active low(default) */
        COMPARATOR_LATCHING = (1 << 2), /* Latching comparator (ADS1114 and ADS1115 only) This bit controls
                                        * whether the ALERT/RDY pin latches after being asserted or clears
                                        * after conversions are within the margin of the upper and lower
                                        * threshold values. This bit serves no function on the ADS1113. 0 :
                                        * Nonlatching comparator . The ALERT/RDY pin does not latch when
                                        * asserted (default). 1 : Latching comparator. The asserted
                                        * ALERT/RDY pin remains latched until conversion data are read by
                                        * the master or an appropriate SMBus alert response is sent by the
                                        * master. The device responds with its address, and it is the
                                        * lowest address currently asserting the ALERT/RDY bus line.
                                        */
    };

    inline bool is_valid_address(const uint8_t address)
    {
        // The 2 lowest pins are selectable so we need to check the upper 6 bits.
        return (address & ~0b11) == 0b1001000;
    }

    inline Error is_valid_ads_address(UCHAR address)
    {
        if (!is_valid_address(address))
        {
            printf("Address invalid, possible addresses include:\n");
            printf("\t ADR pin to GND: 0x%X\n", AddressPin::GND);
            printf("\t ADR pin to VDD: 0x%X\n", AddressPin::VDD);
            printf("\t ADR pin to SDA: 0x%X\n", AddressPin::SDA);
            printf("\t ADR pin to SCL: 0x%X\n", AddressPin::SCL);

            return Error::BAD_ADDRESS;
        }
        return Error::NONE;
    }

    inline double get_fsr_voltage(const FullScaleRange fsr)
    {
        switch (fsr)
        {
            case FSR_6_144V: return 6.144L;
            case FSR_4_096V: return 4.096L;
            case FSR_2_048V: return 2.048L;
            case FSR_1_024V: return 1.024L;
            case FSR_0_512V: return 0.512L;
            default: return 0.256L;
        }
    }

    /**
     * \brief      Class for ADS1115.
     *
     */
    class ADC
    {
    private:
        static constexpr auto DEFAULT_CFG = 0x0583; // Default register configuration.
        const UCHAR           m_address; // I2C address, fixed at construction.
        uint16_t              m_config;  // Packed structure of the configuration. Essentially this is the

        ChannelConfig channelConf;
        DWORD channel;
        DWORD channels;
        FT_HANDLE ftHandle;

        UCHAR readbuf[2];
        UCHAR writebuf[2];

    public:
        FT_STATUS status;
    
        ADC(const UCHAR address) : m_address(address), m_config(DEFAULT_CFG) {}
        ~ADC() {}

        void set_fsr(const FullScaleRange fsr)
        {
            m_config &= ~FSR_0_256V2;
            m_config |= fsr;
        }
        void set_multiplexing(const Multiplex mult)
        {
            m_config &= ~AIN3;
            m_config |= mult;
        }
        void set_data_rate(const DataRate dr)
        {
            m_config &= ~SPS_860;
            m_config |= dr;
        }
        void set_conversion_mode(const ConversionMode mode)
        {
            if (mode)
                m_config |= ConversionMode::SingleShot;
            else
                m_config &= ~ConversionMode::SingleShot;
        }

        const FullScaleRange get_fsr(void) const { return static_cast<FullScaleRange>((m_config & FullScaleRange::FSR_0_256V2)); }
        const Multiplex      get_multiplexing(void) const { return static_cast<Multiplex>((m_config & Multiplex::AIN3)); }
        const DataRate       get_data_rate(void) const { return static_cast<DataRate>((m_config & DataRate::SPS_860)); }
        const ConversionMode get_conversion_mode(void) const { return static_cast<ConversionMode>(m_config & ConversionMode::SingleShot); }

        double raw_to_voltage(const int16_t raw_value)
        {
            const auto fsr_v = get_fsr_voltage(get_fsr());
            return raw_value * (fsr_v / static_cast<double>(0x7FFF));
        }

        FT_STATUS i2c_read(FT_HANDLE ftHandle, UCHAR address, UCHAR reg, PUCHAR value, UCHAR length)
        {
            FT_STATUS status;
            DWORD xfer = 0;

            /* I2C Multiple Byte Read. */
            status = I2C_DeviceWrite(ftHandle, address, 1, &reg, &xfer,
                I2C_TRANSFER_OPTIONS_START_BIT |
                I2C_TRANSFER_OPTIONS_BREAK_ON_NACK);

            if (status == FT_OK)
            {
                /* Repeated Start condition generated. */
                status = I2C_DeviceRead(ftHandle, address, length, value, &xfer,
                    I2C_TRANSFER_OPTIONS_START_BIT |
                    I2C_TRANSFER_OPTIONS_STOP_BIT |
                    I2C_TRANSFER_OPTIONS_NACK_LAST_BYTE);
            }
            APP_CHECK_STATUS(status);

            return status;
        }

        FT_STATUS i2c_write(FT_HANDLE ftHandle, UCHAR address, UCHAR reg, PUCHAR value, UCHAR length)
        {
            FT_STATUS status;
            DWORD xfer = 0;

            /* I2C Multiple Byte Write (not autoincremented) */
            status = I2C_DeviceWrite(ftHandle, address, 1, &reg, &xfer,
                I2C_TRANSFER_OPTIONS_START_BIT |
                I2C_TRANSFER_OPTIONS_BREAK_ON_NACK);
            APP_CHECK_STATUS(status);

            if (status == FT_OK)
            {
                /* Register address not sent on register write. */
                status = I2C_DeviceWrite(ftHandle, address, length, value, &xfer,
                    I2C_TRANSFER_OPTIONS_NO_ADDRESS |
                    I2C_TRANSFER_OPTIONS_STOP_BIT |
                    I2C_TRANSFER_OPTIONS_BREAK_ON_NACK);
                APP_CHECK_STATUS(status);
            }

            return status;
        }

        void i2c_set_conf(I2C_ClockRate_t mode, UCHAR latencyTimer, DWORD options, DWORD channel)
        {
            channelConf.ClockRate = mode;
            channelConf.LatencyTimer = latencyTimer;
            channelConf.Options = options;

            this->channel = channel;
        }

#ifndef _WIN32
        void i2c_get_device_info()
        {
            DWORD numDevs;

            status = FT_ListDevices(&numDevs,NULL,FT_LIST_NUMBER_ONLY);
            if (status == FT_OK) {
                printf("FT_ListDevices OK, number of devices connected: %d\n", numDevs);
            }
            else 
            {
                printf("FT_ListDevices failed\n");
                return;
            }

            // printf("FT_ListDevices - serial numbers\n");

            // char Buffer[64]; // more than enough room!

            // for (DWORD devIndex = 0; devIndex < numDevs; devIndex++)
            // {
            //     status = FT_ListDevices((PVOID)devIndex,Buffer,FT_LIST_BY_INDEX|FT_OPEN_BY_SERIAL_NUMBER);
            //     if (status == FT_OK) 
            //     {
            //         printf("%.*s\n", 64, Buffer);
            //     }
            //     else 
            //     {
            //         printf("FT_ListDevices failed\n");
            //     }
            // }

            FT_PROGRAM_DATA Data;
            FT_DEVICE ftDevice;
            int iport = 0;

            printf("Opening port %d\n", iport);
	
            status = FT_Open(iport, &ftHandle);
            if(status != FT_OK) {
                /* 
                    This can fail if the ftdi_sio driver is loaded
                    use lsmod to check this and rmmod ftdi_sio to remove
                    also rmmod usbserial
                */
                printf("FT_Open(%d) failed\n", iport);
                return;
            }
            
            printf("FT_Open succeeded.  Handle is %p\n", ftHandle);

            status = FT_GetDeviceInfo(ftHandle,
                                        &ftDevice,
                                        NULL,
                                        NULL,
                                        NULL,
                                        NULL); 
            if (status != FT_OK) 
            { 
                printf("FT_GetDeviceType FAILED!\n");
                goto exit;
            }  

            printf("FT_GetDeviceInfo succeeded.  Device is type %d.\n", 
                (int)ftDevice);

            /* MUST set Signature1 and 2 before calling FT_EE_Read */
            Data.Signature1 = 0x00000000;
            Data.Signature2 = 0xffffffff;
            Data.Manufacturer = (char *)malloc(256); /* E.g "FTDI" */
            Data.ManufacturerId = (char *)malloc(256); /* E.g. "FT" */
            Data.Description = (char *)malloc(256); /* E.g. "USB HS Serial Converter" */
            Data.SerialNumber = (char *)malloc(256); /* E.g. "FT000001" if fixed, or NULL */
            if (Data.Manufacturer == NULL ||
                Data.ManufacturerId == NULL ||
                Data.Description == NULL ||
                Data.SerialNumber == NULL)
            {
                printf("Failed to allocate memory.\n");
                goto exit;
            }

            status = FT_EE_Read(ftHandle, &Data);
            if(status != FT_OK) {
                printf("FT_EE_Read failed\n");
                goto exit;
            }

            printf("FT_EE_Read succeeded.\n\n");
                
            printf("Signature1 = %d\n", (int)Data.Signature1);			
            printf("Signature2 = %d\n", (int)Data.Signature2);			
            printf("Version = %d\n", (int)Data.Version);				
                                        
            printf("VendorId = 0x%04X\n", Data.VendorId);				
            printf("ProductId = 0x%04X\n", Data.ProductId);
            printf("Manufacturer = %s\n", Data.Manufacturer);			
            printf("ManufacturerId = %s\n", Data.ManufacturerId);		
            printf("Description = %s\n", Data.Description);			
            printf("SerialNumber = %s\n", Data.SerialNumber);			
            printf("MaxPower = %d\n", Data.MaxPower);				
            printf("PnP = %d\n", Data.PnP) ;					
            printf("SelfPowered = %d\n", Data.SelfPowered);			
            printf("RemoteWakeup = %d\n", Data.RemoteWakeup);

        exit:
            free(Data.Manufacturer);
            free(Data.ManufacturerId);
            free(Data.Description);
            free(Data.SerialNumber);
            FT_Close(ftHandle);
        }
#endif
        void i2c_get_channel_info()
        {
#ifndef _WIN32
            FT_DEVICE_LIST_INFO_NODE* pDevList;

            status = FT_CreateDeviceInfoList(&channels);
            printf("		FT_CreateDeviceInfoList returned %d; channels=%d\n", status, channels);

            if (( FT_OK == status ) && ( channels > 0 ))
            {
                size_t size = sizeof(FT_DEVICE_LIST_INFO_NODE);
                pDevList = (FT_DEVICE_LIST_INFO_NODE *)malloc( size * channels );

                status = FT_GetDeviceInfoList( pDevList, &channels );

                if ( FT_OK == status )
                {
                    for (DWORD channel = 0; channel < channels; channel++)
                    {
                        printf("Information on channel number %d:\n", channel);
                        /*print the dev info*/
                        printf("        Flags=0x%x\n",pDevList[channel].Flags);
                        printf("        Type=0x%x\n",pDevList[channel].Type);
                        printf("        ID=0x%x\n",pDevList[channel].ID);
                        printf("        LocId=0x%x\n",pDevList[channel].LocId);
                        printf("        SerialNumber=%s\n",pDevList[channel].SerialNumber);
                        printf("        Description=%s\n\n",pDevList[channel].Description);
                    }
                }
            }

            free( pDevList );

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
#endif
            FT_DEVICE_LIST_INFO_NODE devList;

            printf("LIBMPSSE test ---------------\n\n");
            printf("\nTest case 1 - I2C_GetNumChannels\n");
            status = I2C_GetNumChannels(&channels);
            printf("		I2C_GetNumChannels returned %d; channels=%d\n", status, channels);

            printf("\nTest case 2 - I2C_GetChannelInfo\n");
            for (DWORD channel = 0; channel < channels; channel++)
            {
                status = I2C_GetChannelInfo(channel, &devList);
                printf("		I2C_GetNumChannels returned %d for channel =%d\n", status, channel);
                /*print the dev info*/
                printf("		Flags=0x%x\n", devList.Flags);
                printf("		Type=0x%x\n", devList.Type);
                printf("		ID=0x%x\n", devList.ID);
                printf("		LocId=0x%x\n", devList.LocId);
                printf("		SerialNumber=%s\n", devList.SerialNumber);
                printf("		Description=%s\n", devList.Description);
                printf("		ftHandle=%p (should be zero)\n", devList.ftHandle);
            }
        }

        void i2c_init(I2C_ClockRate_t mode, UCHAR latencyTimer, DWORD options, DWORD channel)
        {
            i2c_set_conf(mode, latencyTimer, options, channel);
            printf("Configuration has been set\n");

            Init_libMPSSE();
            printf("Initialized libMPSSE\n");
        }

        void i2c_clear()
        {
            Cleanup_libMPSSE();
        }

        void i2c_open_connection()
        {
            status = I2C_OpenChannel(channel, &ftHandle);
            APP_CHECK_STATUS(status);
            printf("Channel %d open status=%d\n", channel, status);

            status = I2C_InitChannel(ftHandle, &channelConf);
            APP_CHECK_STATUS(status);
            printf("Channel %d init status=%d\n", channel, status);
        }

        void i2c_close_connection()
        {
            status = I2C_CloseChannel(ftHandle);
            APP_CHECK_STATUS(status);
            printf("Channel %d close status=%d\n", channel, status);
        }



        // write config without converion request (continous mode)
        void i2c_write_adc_config()
        {
            UCHAR cfgbuf[2];
      
            printf("Config: %u\n", m_config);

            cfgbuf[0] = static_cast<UCHAR>(m_config >> 8);
            cfgbuf[1] = static_cast<UCHAR>(m_config & 0xFF);

            status = i2c_write(ftHandle, m_address, ADS1115::RegisterAddress::Config, cfgbuf, 2);
            APP_CHECK_STATUS(status);
        }

        // write config with converion request (singleshot mode)
        void i2c_write_adc_config_with_conv_request()
        {
            UCHAR cfgbuf[2];

            uint16_t config = m_config | ADS1115::Config::OS;

            printf("Config: %u\n", config);

            cfgbuf[0] = static_cast<UCHAR>(config >> 8);
            cfgbuf[1] = static_cast<UCHAR>(config & 0xFF);

            status = i2c_write(ftHandle, m_address, ADS1115::RegisterAddress::Config, cfgbuf, 2);
            APP_CHECK_STATUS(status);
        }

        uint16_t i2c_read_adc_config()
        {
            UCHAR cfgbuf[2];
            uint16_t cfg;

            status = i2c_read(ftHandle, m_address, ADS1115::RegisterAddress::Config, cfgbuf, 2);
            APP_CHECK_STATUS(status);

            if (status == FT_OK)
            {
                cfg = (cfgbuf[0] << 8) | cfgbuf[1];
                printf("Config: %u\n", cfg);
                return cfg;
            }
            return 0;
        }

        // before execution write config with Continous mode
        double i2c_read_measurement_continous()
        {
            status = i2c_read(ftHandle, m_address, ADS1115::RegisterAddress::Conversion, readbuf, 2);
            APP_CHECK_STATUS(status);

            if (status == FT_OK)
            {
                uint16_t readRawU = (readbuf[0] << 8) | readbuf[1];
                int16_t readRaw = static_cast<int16_t>(readRawU);
                printf("RAW value: %d\n", readRaw);
                double readVoltage = raw_to_voltage(readRaw);
                printf("Voltage: %.5f\n", readVoltage);
                return readVoltage;
            }
            return -1;
        }

        // before execution write config with SingleShot mode
        double i2c_read_measurement_single()
        {
            // Conviently we can send the configuration and the conversion request in one command.
            i2c_write_adc_config_with_conv_request();

            if (status != FT_OK)
                return -1;

            // Once the conversion has been requested we must wait for it do be finished.
            bool is_done = false;
            do
            {
                is_done = i2c_is_adc_conversion_done();
                if (status != FT_OK)
                    return -1;

            } while (!is_done);

            // The conversion register stores the latest result.
            return i2c_read_measurement_continous();
        }

        bool i2c_is_adc_conversion_done()
        {
            uint16_t cfg = i2c_read_adc_config();
            bool is_done = (cfg & Config::OS) ? true : false;
            return is_done;
        }
    };
} // namespace ADS1115
