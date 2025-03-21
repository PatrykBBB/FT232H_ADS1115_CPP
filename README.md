# FT232H_ADS1115_CPP
C++ implementation of I2C communication using FT232H with adc ADS1115

## Linux notes
Install `libftd2xx` and `LibMPSSE` according to instructions (libs should be located at: `/usr/local/lib` and headers at: `/usr/local/include`).

if `FT_CreateDeviceInfoList returns SerialNumber and Description  = NULL`, the EEPROM have to be programed.

Correct example output:

```
FT_CreateDeviceInfoList returned 0; channels=1
Information on channel number 0:
        Flags=0x2
        Type=0x8
        ID=0x4036014
        LocId=0x103
        SerialNumber=FTAIJM7J
        Description=USB <-> Serial Converter
```

To program EEPROM you can use `FT_Prog` from FTDI in Windows

When you use the `FT_Prog` be sure to set pin `C8/C9` to `PWRERR` and `TX&RXEN` to maintain the supported LEDs.
- Download/run `FT_Prog`:
- Go down to `Hardware Specific/IO Controls/`
- Set `C8 = TX&RXLED#`
- Set `C9 = PWREN#`
- Click `Program Devices` Icon (or `Ctrl-P`)

Remember to disable modules:

```
sudo rmmod ftdi_sio
sudo rmmod usbserial
```
and

execute program as `sudo`.