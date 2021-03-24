#include "Arduino.h"
#include "Wire.h"
#include "ldc1614_lib.h"

int _address;

//I2C register addresses
//Default values are shown. 
//See Section [8.2.5 Recommended Initial Register Configuration Values] in the LDC1614 datasheet for recommended initial values.

#define LDC_DEFAULT_ADDRESS   0x2A //LDC1614 address if ADDR pin is pulled low
#define LDC_SECONDARY_ADDRESS 0x2B //LDC1614 address if ADDR pin is pulled high

#define LDC_DATA0             0x00 //ERR_UR0 | ERR_OR0 | ERR_WD0 | ERR_AE0 | DATA0[11:0]
#define LDC_DATA1             0x02 //ERR_UR1 | ERR_OR1 | ERR_WD1 | ERR_AE1 | DATA1[11:0]
#define LDC_DATA2             0x04 //ERR_UR2 | ERR_OR2 | ERR_WD2 | ERR_AE2 | DATA2[11:0]
#define LDC_DATA3             0x06 //ERR_UR3 | ERR_OR3 | ERR_WD3 | ERR_AE3 | DATA3[11:0]

#define LDC_RCOUNT0           0x08 //default 0x08
#define LDC_RCOUNT1           0x09 //default 0x08
#define LDC_RCOUNT2           0x0A //default 0x08
#define LDC_RCOUNT3           0x0B //default 0x08

#define LDC_OFFSET0           0x0C //default 0x00
#define LDC_OFFSET1           0x0D //default 0x00
#define LDC_OFFSET2           0x0E //default 0x00
#define LDC_OFFSET3           0x0F //default 0x00

#define LDC_SETTLECOUNT0      0x10 //default 0x00
#define LDC_SETTLECOUNT1      0x11 //default 0x00
#define LDC_SETTLECOUNT2      0x12 //default 0x00
#define LDC_SETTLECOUNT3      0x13 //default 0x00

#define LDC_CLOCK_DIVIDERS0   0x14 //default 0x00
#define	LDC_CLOCK_DIVIDERS1   0x15 //default 0x00
#define LDC_CLOCK_DIVIDERS2   0x16 //default 0x00
#define	LDC_CLOCK_DIVIDERS3   0x17 //default 0x00

#define LDC_STATUS            0x18
#define LDC_ERROR_CONFIG      0x19 //default 0x0000
#define LDC_CONFIG            0x1A //default 0x2801
#define LDC_MUX_CONFIG        0x1B //default 0x020F
#define LDC_RESET_DEV         0x1C //default 0x0000

#define LDC_DRIVE_CURRENT0    0x1E //default 0x00
#define LDC_DRIVE_CURRENT1    0x1F //default 0x00
#define LDC_DRIVE_CURRENT2    0x20 //default 0x00
#define LDC_DRIVE_CURRENT3    0x21 //default 0x00

#define LDC_MANUFACTURER_ID   0x7E //default 0x5449
#define LDC_DEVICE_ID         0x7F //default 0x3054

LDC161X::LDC161X(bool addr) {
    if (!addr) {
        _address = LDC_DEFAULT_ADDRESS;
    }
    else {
        _address = LDC_SECONDARY_ADDRESS;
    }
}

void LDC161X::LDC_setRecommendedSettings(int channel)
{
    uint8_t rcount_address = 0;
    uint8_t settlecount_address = 0;
    uint8_t clockdividers_address = 0;
    uint8_t drivecurrent_address = 0;
    uint16_t config_code = 0;

    /* CONFIG settings for active channel */
    switch(channel)
    {
        case 0:
            config_code = 0x1601; //0001 0110  0000 0001
            break;
        case 1:
            config_code = 0x5601; //0101 0110  0000 0001
            break;
        case 2:
            config_code = 0x9601; //1001 0110  0000 0001
            break;
        case 3:
            config_code = 0xD601; //1101 0110  0000 0001
            break;
        default:
            config_code = 0x1601; //defaults to channel 0 on invalid argument
    };

    //configure settings
    LDC_setConversionTime(channel, 0x04D6);
    LDC_setSettleTime(channel, 0x000A);
    LDC_setClockDividers(channel, 0x1002);
    LDC_setErrorConfig(0x0000);
    LDC_setDriveCurrent(channel, 0x9000);
    LDC_setConfig(config_code);
}

uint16_t LDC161X::LDC_readRegister(uint8_t register_address)
{
    uint16_t data = 0;

    Wire.beginTransmission(_address); //I2C request code by Matthew Post
    Wire.write(register_address); //Data register for specified channel
    Wire.endTransmission();
    Wire.requestFrom(_address, 2); //request 2 bytes
    while(Wire.available()) {     //read bytes
      data = data << 8;             //shift MSB to left 8 bits
      data = data | Wire.read();    //stop at LSB
    }

    return data;
}

void LDC161X::LDC_setRegister(uint8_t register_address, uint16_t contents)
{
    uint8_t byte1 = (contents >> 8) & 0xFF; //MSB
    uint8_t byte0 = contents & 0xFF; //LSB
    Wire.beginTransmission(_address);
    Wire.write(register_address);
    Wire.write(byte1);
    Wire.write(byte0);
    Wire.endTransmission();
}

uint32_t LDC161X::LDC_readData(int channel)
{
    uint8_t register_address = 0;
    
    switch(channel)
    {
        case 0:
            register_address = LDC_DATA0;
            break;
        case 1:
            register_address = LDC_DATA1;
            break;
        case 2:
            register_address = LDC_DATA2;
            break;
        case 3:
            register_address = LDC_DATA3;
            break;
        default:
            register_address = LDC_DATA0; //defaults to DATA0 on invalid argument
    };

    uint16_t data1 = 0;
    uint16_t data2 = 0;

    Wire.beginTransmission(_address); //I2C request code by Matthew Post
    Wire.write(register_address); //Data register MSB for specified channel
    Wire.endTransmission();
    Wire.requestFrom(_address, 2); //request 2 bytes
    while(Wire.available()) {     //read bytes
      data1 = data1 << 8;             //shift MSB to left 8 bits
      data1 = data1 | Wire.read();    //stop at LSB
    }

    Wire.beginTransmission(_address); 
    Wire.write(register_address + 1); //Data register LSB for specified channel
    Wire.endTransmission();
    Wire.requestFrom(_address, 2); //request 2 bytes
    while (Wire.available()) {     //read bytes
        data2 = data2 << 8;             //shift MSB to left 8 bits
        data2 = data2 | Wire.read();    //stop at LSB
    }

    uint32_t data = ((uint32_t)data1 << 16) + data2;


    return data;
}

uint16_t LDC161X::LDC_readManufacturerID()
{
    uint16_t data = 0;
    
    Wire.beginTransmission(_address);
    Wire.write(LDC_MANUFACTURER_ID); //Manufacturer ID register
    Wire.endTransmission();
    Wire.requestFrom(_address, 2); //request 2 bytes
    while(Wire.available()) {     //read bytes
      data = data << 8;             //shift MSB to left 8 bits
      data = data | Wire.read();    //stop at LSB
    }

    return data;
}

uint16_t LDC161X::LDC_readDeviceID()
{
    uint16_t data = 0;
    
    Wire.beginTransmission(_address);
    Wire.write(LDC_DEVICE_ID); //Device ID register
    Wire.endTransmission();
    Wire.requestFrom(_address, 2); //request 2 bytes
    while(Wire.available()) {     //read bytes
      data = data << 8;             //shift MSB to left 8 bits
      data = data | Wire.read();    //stop at LSB
    }

    return data;
}

uint16_t LDC161X::LDC_readStatus()
{
    uint16_t data = 0;

    Wire.beginTransmission(_address);
    Wire.write(LDC_STATUS); //Data register for Status bytes
    Wire.endTransmission();
    Wire.requestFrom(_address, 2); //request 2 bytes
    while(Wire.available()) {     //read bytes
      data = data << 8;             //shift MSB to left 8 bits
      data = data | Wire.read();    //stop at LSB
    }

    return data;
}

bool LDC161X::LDC_hasNewData(int channel, uint16_t status_result)
{
    switch(channel)
    {
        case 0:
            return ((status_result & 0x0008) == 0x0008); //0000 0000  0000 1000 - UNREADCONV0 bit
        case 1:
            return ((status_result & 0x0004) == 0x0004); //0000 0000  0000 0100 - UNREADCONV1 bit
        case 2:
            return ((status_result & 0x0002) == 0x0002); //0000 0000  0000 0010 - UNREADCONV2 bit
        case 3:
            return ((status_result & 0x0001) == 0x0001); //0000 0000  0000 0001 - UNREADCONV3 bit
        default:
            return ((status_result & 0x0008) == 0x0008); //0000 0000  0000 1000 - UNREADCONV0 bit
                                                         //defaults to channel 0 on invalid argument
    };
}

bool LDC161X::LDC_hasNewData(int channel)
{
    return LDC_hasNewData(channel, LDC_readStatus());
}

bool LDC161X::LDC_hasConversionErrors(uint16_t status_result)
{ 
    return (status_result & 0x3F00) != 0x0000; //error bits: 0011 1111 0000 0000
}

uint16_t LDC161X::LDC_getChannelWithErrors(uint16_t status_result)
{
    return (status_result >> 14) & 0x0003; //shift right 14 bits and & w/ 0000 0000 0000 0011
}

void LDC161X::LDC_resetLDC()
{ //Writes b1 to MSB of RESET_DEV
    Wire.beginTransmission(_address);
    Wire.write(LDC_RESET_DEV);
    Wire.write(0x80); //1000 0000
    Wire.write(0x00);
    Wire.endTransmission();
}

uint16_t LDC161X::LDC_readResetDev()
{
    uint16_t data = 0;

    Wire.beginTransmission(_address);
    Wire.write(LDC_RESET_DEV); //register for specified channel
    Wire.endTransmission();
    Wire.requestFrom(_address, 2); //request 2 bytes
    while(Wire.available()) {     //read bytes
      data = data << 8;             //shift MSB to left 8 bits
      data = data | Wire.read();    //stop at LSB
    }

    return data;
}

void LDC161X::LDC_setResetDev(uint16_t config_code)
{
    uint8_t byte1 = (config_code >> 8) & 0xFF; //MSB
    uint8_t byte0 = config_code & 0xFF; //LSB
    Wire.beginTransmission(_address);
    Wire.write(LDC_RESET_DEV);
    Wire.write(byte1);
    Wire.write(byte0);
    Wire.endTransmission();
}

void LDC161X::LDC_setOffset(int channel, uint16_t offset)
{
    uint8_t register_address = 0;

    switch(channel)
    {
        case 0:
            register_address = LDC_OFFSET0;
            break;
        case 1:
            register_address = LDC_OFFSET1;
            break;
        case 2:
            register_address = LDC_OFFSET2;
            break;
        case 3:
            register_address = LDC_OFFSET3;
            break;
        default:
            register_address = LDC_OFFSET0; //defaults to channel 0 on invalid argument
    };

    uint8_t byte1 = (offset >> 8) & 0xFF; //MSB
    uint8_t byte0 = offset & 0xFF; //LSB
    Wire.beginTransmission(_address);
    Wire.write(register_address);
    Wire.write(byte1);
    Wire.write(byte0);
    Wire.endTransmission();
}

uint16_t LDC161X::LDC_readOffset(int channel)
{
    uint8_t register_address = 0;

    switch(channel)
    {
        case 0:
            register_address = LDC_OFFSET0;
            break;
        case 1:
            register_address = LDC_OFFSET1;
            break;
        case 2:
            register_address = LDC_OFFSET2;
            break;
        case 3:
            register_address = LDC_OFFSET3;
            break;
        default:
            register_address = LDC_OFFSET0; //defaults to channel 0 on invalid argument
    };

    uint16_t data = 0;

    Wire.beginTransmission(_address);
    Wire.write(register_address); //register for specified channel
    Wire.endTransmission();
    Wire.requestFrom(_address, 2); //request 2 bytes
    while(Wire.available()) {     //read bytes
      data = data << 8;             //shift MSB to left 8 bits
      data = data | Wire.read();    //stop at LSB
    }

    return data;
}

void LDC161X::LDC_setConversionTime(int channel, uint16_t conversion_count)
{
    uint8_t register_address = 0;

    switch(channel)
    {
        case 0:
            register_address = LDC_RCOUNT0;
            break;
        case 1:
            register_address = LDC_RCOUNT1;
            break;
        case 2:
            register_address = LDC_RCOUNT2;
            break;
        case 3:
            register_address = LDC_RCOUNT3;
            break;
        default:
            register_address = LDC_RCOUNT0; //defaults to channel 0 on invalid argument
    };

    uint8_t byte1 = (conversion_count >> 8) & 0xFF; //MSB
    uint8_t byte0 = conversion_count & 0xFF; //LSB
    Wire.beginTransmission(_address);
    Wire.write(register_address);
    Wire.write(byte1);
    Wire.write(byte0);
    Wire.endTransmission();
}

uint16_t LDC161X::LDC_readConversionTime(int channel)
{
    uint8_t register_address = 0;

    switch(channel)
    {
        case 0:
            register_address = LDC_RCOUNT0;
            break;
        case 1:
            register_address = LDC_RCOUNT1;
            break;
        case 2:
            register_address = LDC_RCOUNT2;
            break;
        case 3:
            register_address = LDC_RCOUNT3;
            break;
        default:
            register_address = LDC_RCOUNT0; //defaults to channel 0 on invalid argument
    };

    uint16_t data = 0;

    Wire.beginTransmission(_address);
    Wire.write(register_address); //register for specified channel
    Wire.endTransmission();
    Wire.requestFrom(_address, 2); //request 2 bytes
    while(Wire.available()) {     //read bytes
      data = data << 8;             //shift MSB to left 8 bits
      data = data | Wire.read();    //stop at LSB
    }

    return data;
}

void LDC161X::LDC_setRCount(int channel, uint16_t conversion_count)
{
    uint8_t register_address = 0;

    switch(channel)
    {
        case 0:
            register_address = LDC_RCOUNT0;
            break;
        case 1:
            register_address = LDC_RCOUNT1;
            break;
        case 2:
            register_address = LDC_RCOUNT2;
            break;
        case 3:
            register_address = LDC_RCOUNT3;
            break;
        default:
            register_address = LDC_RCOUNT0; //defaults to channel 0 on invalid argument
    };

    uint8_t byte1 = (conversion_count >> 8) & 0xFF; //MSB
    uint8_t byte0 = conversion_count & 0xFF; //LSB
    Wire.beginTransmission(_address);
    Wire.write(register_address);
    Wire.write(byte1);
    Wire.write(byte0);
    Wire.endTransmission();
}

uint16_t LDC161X::LDC_readRCount(int channel)
{
    uint8_t register_address = 0;

    switch(channel)
    {
        case 0:
            register_address = LDC_RCOUNT0;
            break;
        case 1:
            register_address = LDC_RCOUNT1;
            break;
        case 2:
            register_address = LDC_RCOUNT2;
            break;
        case 3:
            register_address = LDC_RCOUNT3;
            break;
        default:
            register_address = LDC_RCOUNT0; //defaults to channel 0 on invalid argument
    };

    uint16_t data = 0;

    Wire.beginTransmission(_address);
    Wire.write(register_address); //register for specified channel
    Wire.endTransmission();
    Wire.requestFrom(_address, 2); //request 2 bytes
    while(Wire.available()) {     //read bytes
      data = data << 8;             //shift MSB to left 8 bits
      data = data | Wire.read();    //stop at LSB
    }

    return data;
}

void LDC161X::LDC_setClockDividers(int channel, uint16_t config_code)
{
    uint8_t register_address = 0;

    switch(channel)
    {
        case 0:
            register_address = LDC_CLOCK_DIVIDERS0;
            break;
        case 1:
            register_address = LDC_CLOCK_DIVIDERS1;
            break;
        case 2:
            register_address = LDC_CLOCK_DIVIDERS2;
            break;
        case 3:
            register_address = LDC_CLOCK_DIVIDERS3;
            break;
        default:
            register_address = LDC_CLOCK_DIVIDERS0; //defaults to channel 0 on invalid argument
    };

    uint8_t byte1 = (config_code >> 8) & 0xFF; //MSB
    uint8_t byte0 = config_code & 0xFF; //LSB
    Wire.beginTransmission(_address);
    Wire.write(register_address);
    Wire.write(byte1);
    Wire.write(byte0);
    Wire.endTransmission();
}

void LDC161X::LDC_setClockDividers(int channel, uint16_t input_divider, uint16_t reference_divider)
{
    input_divider = (input_divider << 11) & 0xF000; //shift to the left and & w/ 1111 0000  0000 0000
    reference_divider = reference_divider & 0x03FF; // & w/ 0000 0011  1111 1111

    uint16_t config_code = input_divider | reference_divider; //combine arguments

    LDC_setClockDividers(channel, config_code); //send to LDC1312
}

uint16_t LDC161X::LDC_readClockDividers(int channel)
{
    uint8_t register_address = 0;

    switch(channel)
    {
        case 0:
            register_address = LDC_CLOCK_DIVIDERS0;
            break;
        case 1:
            register_address = LDC_CLOCK_DIVIDERS1;
            break;
        case 2:
            register_address = LDC_CLOCK_DIVIDERS2;
            break;
        case 3:
            register_address = LDC_CLOCK_DIVIDERS3;
            break;
        default:
            register_address = LDC_CLOCK_DIVIDERS0; //defaults to channel 0 on invalid argument
    };

    uint16_t data = 0;

    Wire.beginTransmission(_address);
    Wire.write(register_address); //CLOCK_DIVIDERSx register for specified channel
    Wire.endTransmission();
    Wire.requestFrom(_address, 2); //request 2 bytes
    while(Wire.available()) {     //read bytes
      data = data << 8;             //shift MSB to left 8 bits
      data = data | Wire.read();    //stop at LSB
    }

    return data;
}

uint16_t LDC161X::LDC_getInputDivider(uint16_t config_code)
{
    return (config_code >> 12) & 0x000F; //shift right 12 bits and & w/ 0000 0000  0000 1111
}

uint16_t LDC161X::LDC_getReferenceDivider(uint16_t config_code)
{
    return config_code & 0x03FF; //return bits 9:0 0000 0011 1111 1111
}

void LDC161X::LDC_setSettleTime(int channel, uint16_t settle_count)
{
    uint8_t register_address = 0;

    switch(channel)
    {
        case 0:
            register_address = LDC_SETTLECOUNT0;
            break;
        case 1:
            register_address = LDC_SETTLECOUNT1;
            break;
        case 2:
            register_address = LDC_SETTLECOUNT2;
            break;
        case 3:
            register_address = LDC_SETTLECOUNT3;
            break;
        default:
            register_address = LDC_SETTLECOUNT0; //defaults to channel 0 on invalid argument
    };

    uint8_t byte1 = (settle_count >> 8) & 0xFF; //MSB
    uint8_t byte0 = settle_count & 0xFF; //LSB
    Wire.beginTransmission(_address);
    Wire.write(register_address);
    Wire.write(byte1);
    Wire.write(byte0);
    Wire.endTransmission();
}

uint16_t LDC161X::LDC_readSettleTime(int channel)
{
    uint8_t register_address = 0;

    switch(channel)
    {
        case 0:
            register_address = LDC_SETTLECOUNT0;
            break;
        case 1:
            register_address = LDC_SETTLECOUNT1;
            break;
        case 2:
            register_address = LDC_SETTLECOUNT2;
            break;
        case 3:
            register_address = LDC_SETTLECOUNT3;
            break;
        default:
            register_address = LDC_SETTLECOUNT0; //defaults to channel 0 on invalid argument
    };

    uint16_t data = 0;

    Wire.beginTransmission(_address);
    Wire.write(register_address); //register for specified channel
    Wire.endTransmission();
    Wire.requestFrom(_address, 2); //request 2 bytes
    while(Wire.available()) {     //read bytes
      data = data << 8;             //shift MSB to left 8 bits
      data = data | Wire.read();    //stop at LSB
    }

    return data;
}

void LDC161X::LDC_setConfig(uint16_t config_code)
{
    uint8_t byte1 = (config_code >> 8) & 0xFF; //MSB
    uint8_t byte0 = config_code & 0xFF; //LSB
    Wire.beginTransmission(_address);
    Wire.write(LDC_CONFIG);
    Wire.write(byte1);
    Wire.write(byte0);
    Wire.endTransmission();
}

uint16_t LDC161X::LDC_readConfig()
{
    uint16_t data = 0;

    Wire.beginTransmission(_address);
    Wire.write(LDC_CONFIG); //Data register for Status bytes
    Wire.endTransmission();
    Wire.requestFrom(_address, 2); //request 2 bytes
    while(Wire.available()) {     //read bytes
      data = data << 8;             //shift MSB to left 8 bits
      data = data | Wire.read();    //stop at LSB
    }

    return data;
}

int LDC161X::LDC_getActiveChannel(uint16_t config_code)
{
    return (config_code >> 14) & 0x0003; //shift right 14 bits and & w/ 0000 0000  0000 0011
                                         //Active channel bits @ 15:14
}

bool LDC161X::LDC_isSleeping(uint16_t config_code)
{
    return (config_code & 0x2000) == 0x2000; //0010 0000  0000 0000
}

bool LDC161X::LDC_isCurrentOverrideEnabled(uint16_t config_code)
{
    return (config_code & 0x1000) == 0x1000; //0001 0000  0000 0000
}

bool LDC161X::LDC_isLowPowerModeEnabled(uint16_t config_code)
{
    return (config_code & 0x0800) == 0x0800; //0000 1000  0000 0000
}

bool LDC161X::LDC_isAutoAmplitudeDisabled(uint16_t config_code)
{
    return (config_code & 0x0400) == 0x0400; //0000 0100  0000 0000
}

bool LDC161X::LDC_hasExternalOscillator(uint16_t config_code)
{
    return (config_code & 0x0200) == 0x0200; //0000 0010  0000 0000
}

bool LDC161X::LDC_isInterruptDisabled(uint16_t config_code)
{
    return (config_code & 0x0080) == 0x0080; //0000 0000  1000 0000
}

bool LDC161X::LDC_isHighCurrentDriveEnabled(uint16_t config_code)
{
    return (config_code & 0x0020) == 0x0020; //0000 0000  0010 0000
}

void LDC161X::LDC_setErrorConfig(uint16_t config_code)
{
    uint8_t byte1 = (config_code >> 8) & 0xFF; //MSB
    uint8_t byte0 = config_code & 0xFF; //LSB
    Wire.beginTransmission(_address);
    Wire.write(LDC_ERROR_CONFIG);
    Wire.write(byte1);
    Wire.write(byte0);
    Wire.endTransmission();
}

uint16_t LDC161X::LDC_readErrorConfig()
{
    uint16_t data = 0;

    Wire.beginTransmission(_address);
    Wire.write(LDC_ERROR_CONFIG); //Data register for Status bytes
    Wire.endTransmission();
    Wire.requestFrom(_address, 2); //request 2 bytes
    while(Wire.available()) {     //read bytes
      data = data << 8;             //shift MSB to left 8 bits
      data = data | Wire.read();    //stop at LSB
    }

    return data;
}

void LDC161X::LDC_setMUXConfig(uint16_t config_code)
{
    uint8_t byte1 = (config_code >> 8) & 0xFF; //MSB
    uint8_t byte0 = config_code & 0xFF; //LSB
    Wire.beginTransmission(_address);
    Wire.write(LDC_MUX_CONFIG);
    Wire.write(byte1);
    Wire.write(byte0);
    Wire.endTransmission();
}

uint16_t LDC161X::LDC_readMUXConfig()
{
    uint16_t data = 0;

    Wire.beginTransmission(_address);
    Wire.write(LDC_MUX_CONFIG);
    Wire.endTransmission();
    Wire.requestFrom(_address, 2); //request 2 bytes
    while(Wire.available()) {     //read bytes
      data = data << 8;             //shift MSB to left 8 bits
      data = data | Wire.read();    //stop at LSB
    }

    return data;
}

bool LDC161X::LDC_isAutoscanEnabled(uint16_t config_code)
{
    return (config_code & 0x8000) == 0x8000; //1000 0000  0000 0000
}

uint16_t LDC161X::LDC_getAutoscanSequence(uint16_t config_code)
{
    return (config_code >> 13) & 0x0003; //shift right 13 bits and & w/ 0000 0000  0000 0011
}

uint16_t LDC161X::LDC_getDeglitchBandwidth(uint16_t config_code)
{
    return config_code & 0x0007; //return last 3 bits 0000 0000  0000 0111
}

void LDC161X::LDC_setDriveCurrent(int channel, uint16_t drive_current)
{
    uint8_t register_address = 0;

    switch(channel)
    {
        case 0:
            register_address = LDC_DRIVE_CURRENT0;
            break;
        case 1:
            register_address = LDC_DRIVE_CURRENT1;
            break;
        case 2:
            register_address = LDC_DRIVE_CURRENT2;
            break;
        case 3:
            register_address = LDC_DRIVE_CURRENT3;
            break;
        default:
            register_address = LDC_DRIVE_CURRENT0; //defaults to channel 0 on invalid argument
    };

    uint8_t byte1 = (drive_current >> 8) & 0xFF; //MSB
    uint8_t byte0 = drive_current & 0xFF; //LSB

    Wire.beginTransmission(_address);
    Wire.write(register_address);
    Wire.write(byte1);
    Wire.write(byte0);
    Wire.endTransmission();
}

uint16_t LDC161X::LDC_readDriveCurrent(int channel)
{
    uint8_t register_address = 0;

    switch(channel)
    {
        case 0:
            register_address = LDC_DRIVE_CURRENT0;
            break;
        case 1:
            register_address = LDC_DRIVE_CURRENT1;
            break;
        case 2:
            register_address = LDC_DRIVE_CURRENT2;
            break;
        case 3:
            register_address = LDC_DRIVE_CURRENT3;
            break;
        default:
            register_address = LDC_DRIVE_CURRENT0; //defaults to channel 0 on invalid argument
    };

    uint16_t data = 0;

    Wire.beginTransmission(_address);
    Wire.write(register_address);
    Wire.endTransmission();
    Wire.requestFrom(_address, 2); //request 2 bytes
    while(Wire.available()) {     //read bytes
      data = data << 8;             //shift MSB to left 8 bits
      data = data | Wire.read();    //stop at LSB
    }

    return data;
}

uint16_t LDC161X::LDC_getDriveCurrent(int channel)
{
    return (LDC_readDriveCurrent(channel) >> 11) & 0x001F; //shift right 11 bits and & w/ 0000 0000 0001 1111
}

uint16_t LDC161X::LDC_getInitialDriveCurrent(int channel)
{
    return (LDC_readDriveCurrent(channel) >> 6) & 0x001F; //shift right 6 bits and & w/ 0000 0000 0001 1111
}