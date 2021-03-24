// LDC1614 datasheet: https://www.ti.com/lit/ds/symlink/ldc1614.pdf
//
// This library works for both the LDC1612 and LDC1614 sensors - if you're
// using an LDC1612, just avoid using channels 2 and 3 because the LDC1612
// doesn't have those.
// 
// Most of the channel-specific functions default to Channel 0 if they're
// given an invalid channel argument.
//
// Function naming:
// setXXX = sets all or part of a register
// readXXX = returns an entire register
// getXXX = returns part of a register
// is/hasXXX = boolean condition is met?
//
// Written/modified by Jan Sonntag based of the LDC1312-Arduino library: https://github.com/SirSundays/LDC1312-Arduino
// The main differences are the lack of the gain settings and that you have to read more data

#ifndef _LDC1614_LIB_H //inclusion guard
#define _LDC1614_LIB_H

#include <Arduino.h>

class LDC161X {
public:
	LDC161X(bool addr);
	//------------------------------------------------------------------------------------------
	// Sets the LDC to the settings recommended by the LDC1614 datasheet, section 8.2.5
	// This sets the LDC to the single-channel recommended settings for the specified channel.
	// [Table 44. Recommended Initial Register Configuration Values (Single-Channel Operation)]
	// Recommended to use resetLDC() and a small delay before calling this function.
	//------------------------------------------------------------------------------------------
	void LDC_setRecommendedSettings(int channel);

	//------------------------------------------------------------------------------------------
	// Reads and returns the contents of a register.
	// Not all registers are readable, and some registers are changed when they're read.
	// Read the datasheet before using this function.
	//------------------------------------------------------------------------------------------
	uint16_t LDC_readRegister(uint8_t register_address);

	//------------------------------------------------------------------------------------------
	// Sets the contents of a register. There is no error checking or hand-holding.
	// The likelihood of screwing something up is high if you don't know what you're doing.
	// Read the datasheet before using this function.
	//------------------------------------------------------------------------------------------
	void LDC_setRegister(uint8_t register_address, uint16_t contents);

	//------------------------------------------------------------------------------------------
	// Reads and returns the manufacturer ID via I2C.
	//------------------------------------------------------------------------------------------
	uint16_t LDC_readManufacturerID();

	//------------------------------------------------------------------------------------------
	// Reads and returns the device ID via I2C.
	//------------------------------------------------------------------------------------------
	uint16_t LDC_readDeviceID();

	//------------------------------------------------------------------------------------------
	// Reads and returns the raw sensor output for channel x via I2C. Doesn't remove error bits.
	// Output bits: ERR_URx | ERR_ORx | ERR_WDx | ERR_AEx | DATAx[11:0]
	//------------------------------------------------------------------------------------------
	uint32_t LDC_readData(int channel);

	//------------------------------------------------------------------------------------------
	// Reads and returns the status bytes for the sensor via I2C.
	// Output bits: ERR_CHAN[1:0] | ERR_UR | ERR_OR | ERR_WD | ERR_AHE | ERR_ALE | ERR_ZC
	//                0  |  DRDY  |   0    |    0   | UNREADCONV[0:4] //channels 0-4
	// See [Table 21. Address 0x18, STATUS Field Descriptions] in the LDC1614 datasheet.
	// The only supporting functions this has are hasNewData() and checking if errors exist
	// - you will need to parse the bits yourself to read the error reporting bits in this 
	//   register for specific details.
	//------------------------------------------------------------------------------------------
	uint16_t LDC_readStatus();

	//------------------------------------------------------------------------------------------
	// Reads UNREADCONVx bit from an already existing getStatus() result.
	// Returns 1 (true) if there is a new conversion result available for channel x.
	// Returns 0 (false) if there is NOT a new conversion result available for channel x.
	//------------------------------------------------------------------------------------------
	bool LDC_hasNewData(int channel, uint16_t status_result);

	//------------------------------------------------------------------------------------------
	// Reads and returns the UNREADCONVx bit from the STATUS register via I2C.
	// Reading from the STATUS register wipes errors, so only use this function if you're
	// ignoring errors.
	// Returns 1 (true) if there is a new conversion result available for channel x.
	// Returns 0 (false) if there is NOT a new conversion result available for channel x.
	//------------------------------------------------------------------------------------------
	bool LDC_hasNewData(int channel);

	//------------------------------------------------------------------------------------------
	// Reads from an already existing getStatus() result.
	// Returns 1 (true) if the LDC is reporting a conversion error.
	// Returns 0 (false) if the LDC is NOT reporting a conversion error.
	//------------------------------------------------------------------------------------------
	bool LDC_hasConversionErrors(uint16_t status_result);

	//------------------------------------------------------------------------------------------
	// Reads from an already existing getStatus() result. Returns the channel in which an error
	// was detected. Only use this after checking if there's an error in the first place
	// (use hasConversionErrors()).
	// b00: Channel 0 is source of flag or error.
	// b01: Channel 1 is source of flag or error.
	// b10: Channel 2 is source of flag or error (LDC1314 only).
	// b11: Channel 3 is source of flag or error (LDC1314 only).
	//------------------------------------------------------------------------------------------
	uint16_t LDC_getChannelWithErrors(uint16_t status_result);

	//------------------------------------------------------------------------------------------
	// Resets the LDC1614 to its default values.
	// It's probably a good idea to add a delay after calling this function to give the
	// sensor some time to settle.
	//------------------------------------------------------------------------------------------
	void LDC_resetLDC();

	//------------------------------------------------------------------------------------------
	// Reads and returns the entire RESET_DEV register.
	// Usage of this function is not recommended; it's better to use the read/setGain and
	// resetLDC functions instead.
	//------------------------------------------------------------------------------------------
	uint16_t LDC_readResetDev();

	//------------------------------------------------------------------------------------------
	// Writes to the RESET_DEV register.
	// Usage of this function is not recommended; it's better to use the read/setGain and
	// resetLDC functions instead.
	//------------------------------------------------------------------------------------------
	void LDC_setResetDev(uint16_t config_code);

	//------------------------------------------------------------------------------------------
	// Sets the offset to apply to the output of the specified channel.
	// See Section [8.1.3.1 Data Offset] in the LDC1614 datasheet.
	// The offset values should be < ƒSENSORx_MIN / ƒREFx.
	//------------------------------------------------------------------------------------------
	void LDC_setOffset(int channel, uint16_t offset);

	//------------------------------------------------------------------------------------------
	// Reads and returns the OFFSETx register for the specified channel.
	//------------------------------------------------------------------------------------------
	uint16_t LDC_readOffset(int channel);

	//------------------------------------------------------------------------------------------
	// Sets the conversion count for channel x. Conversion time is calculated with:
	// t = (RCOUNTx × 16 + 4)/fREFx - meaning that the conversion time depends on both the
	//                                value set here and the reference frequency
	//                                (and the frequency divider).
	// A higher conversion time results in higher resolution but obviously takes more time.
	// See Section [8.1.4 Sensor Conversion Time] in the LDC1614 datasheet.
	//------------------------------------------------------------------------------------------
	void LDC_setConversionTime(int channel, uint16_t conversion_count);

	//------------------------------------------------------------------------------------------
	// Reads and returns the RCOUNTx register for the specified channel.
	// conversion time = (RCOUNTx × 16 + 4) /fREFx
	//------------------------------------------------------------------------------------------
	uint16_t LDC_readConversionTime(int channel);

	//------------------------------------------------------------------------------------------
	// Does the same thing as readConversionTime().
	// RCOUNTx, and someone familiar with the register names might be confused by a missing
	// RCOUNT function.
	// Sets the conversion count for channel x. Conversion time is calculated with:
	// t = (RCOUNTx × 16 + 4)/fREFx - meaning that the conversion time depends on both the
	//                                value set here and the reference frequency
	//                                (and the frequency divider).
	// A higher conversion time results in higher resolution but obviously takes more time.
	// See Section [8.1.4 Sensor Conversion Time] in the LDC1614 datasheet.
	//------------------------------------------------------------------------------------------
	void LDC_setRCount(int channel, uint16_t conversion_count);

	//------------------------------------------------------------------------------------------
	// Does the same thing as readConversionTime(). Included because the register itself is
	// RCOUNTx, and someone familiar with the register names might be confused by a missing
	// RCOUNT function.
	// Reads and returns the RCOUNTx register for the specified channel.
	// conversion time = (RCOUNTx × 16 + 4) /fREFx
	//------------------------------------------------------------------------------------------
	uint16_t LDC_readRCount(int channel);

	//------------------------------------------------------------------------------------------
	// Read Section [8.1.4.1 Settling Time] in the LDC1614 datasheet or things will break.
	// t = (SETTLECOUNTx × 16 + 4)/fREFx  - meaning that the conversion time depends on both the
	//                                      value set here and the reference frequency
	//                                      (and the frequency divider).
	// Settling time is how long it takes for the current oscillation amplitude to stabilize.
	// There is a minimum value that the settling time must be, 
	// and the settle count must satisfy the following equation:
	// SETTLECOUNTx ≥ QSENSORx × ƒREFx / (16 × ƒSENSORx)
	// where:
	// • ƒSENSORx = Sensor Frequency of Channel 0
	// • ƒREFx = Reference frequency for Channel 0
	// • QSENSORx = Quality factor of the sensor coil on Channel 0 (Q = R*sqrt(C/L))
	//------------------------------------------------------------------------------------------
	void LDC_setSettleTime(int channel, uint16_t settle_count);

	//------------------------------------------------------------------------------------------
	// Reads and returns the SETTLECOUNTx register for the specified channel.
	// settle time = (SETTLECOUNTx × 16 + 4)/fREFx
	//------------------------------------------------------------------------------------------
	uint16_t LDC_readSettleTime(int channel);

	//------------------------------------------------------------------------------------------
	// Refer to Section [7.6.18 Address 0x14, CLOCK_DIVIDERS0] in the LDC1614 datasheet or
	// things will almost certainly break. The bytes must be formatted carefully.
	// Sets the input and reference dividers for the specified channel.
	//------------------------------------------------------------------------------------------
	void LDC_setClockDividers(int channel, uint16_t config_code);

	//------------------------------------------------------------------------------------------
	// Like the previous setClockDividers() function, except it'll format the bytes for you.
	// input_divider only uses the last 4 bits (0-15). The first 12 bits are ignored.
	// reference_divider only uses the last 10 bits (0-1023). The first 6 bits are ignored.
	// The remaining bits in between the two are reserved and are always set to 0.
	//------------------------------------------------------------------------------------------
	void LDC_setClockDividers(int channel, uint16_t input_divider, uint16_t reference_divider);

	//------------------------------------------------------------------------------------------
	// Reads and returns the entire CLOCK_DIVIDERSx register for the specified channel.
	//------------------------------------------------------------------------------------------
	uint16_t LDC_readClockDividers(int channel);

	//------------------------------------------------------------------------------------------
	// Takes the output of readClockDividers() and returns the input divider settings.
	// Always returns a value between 0 and 15.
	//------------------------------------------------------------------------------------------
	uint16_t LDC_getInputDivider(uint16_t config_code);

	//------------------------------------------------------------------------------------------
	// Takes the output of readClockDividers() and returns the reference divider settings.
	// Always returns a value between 0 and 1023.
	//------------------------------------------------------------------------------------------
	uint16_t LDC_getReferenceDivider(uint16_t config_code);

	//------------------------------------------------------------------------------------------
	// Sets config register bytes. Read Section [7.6.24 Address 0x1A, CONFIG] in
	// the LDC1312 datasheet for which bit does what. Make sure to respect reserved bits.
	//------------------------------------------------------------------------------------------
	void LDC_setConfig(uint16_t config_code);

	//------------------------------------------------------------------------------------------
	// Reads and returns the entire CONFIG register via I2C.
	//------------------------------------------------------------------------------------------
	uint16_t LDC_readConfig();

	//------------------------------------------------------------------------------------------
	// Reads the CONFIG register value and returns the active channel (0-3).
	// This is for continuous conversions without channel switching and only does anything if
	//  MUX_CONFIG.AUTOSCAN_EN = 0.
	// The readConfig() function must be called to get the input for this function.
	//------------------------------------------------------------------------------------------
	int LDC_getActiveChannel(uint16_t config_code);

	//------------------------------------------------------------------------------------------
	// Reads the CONFIG register value and returns the SLEEP_MODE_EN bit.
	// See Section [7.4.2 Sleep Mode (Configuration Mode)] for more information.
	// The readConfig() function must be called to get the input for this function.
	//------------------------------------------------------------------------------------------
	bool LDC_isSleeping(uint16_t config_code);

	//------------------------------------------------------------------------------------------
	// Reads the CONFIG register value and returns the RP_OVERRIDE_EN bit.
	// This is set to 1 when the current drive is being manually controlled.
	// The readConfig() function must be called to get the input for this function.
	//------------------------------------------------------------------------------------------
	bool LDC_isCurrentOverrideEnabled(uint16_t config_code);

	//------------------------------------------------------------------------------------------
	// Reads the CONFIG register value and returns the SENSOR_ACTIVATE_SEL bit.
	// See Section [8.1.4.2 Sensor Activation] for more information.
	// The readConfig() function must be called to get the input for this function.
	//------------------------------------------------------------------------------------------
	bool LDC_isLowPowerModeEnabled(uint16_t config_code);

	//------------------------------------------------------------------------------------------
	// Reads the CONFIG register value and returns the AUTO_AMP_DIS bit.
	// NOTE THAT THIS FUNCTION CHECKS IF SOMETHING IS *DISABLED*, not ENABLED
	// See Section [8.1.5.4 Sensor Auto-Calibration Mode] for more information.
	// The readConfig() function must be called to get the input for this function.
	//------------------------------------------------------------------------------------------
	bool LDC_isAutoAmplitudeDisabled(uint16_t config_code);

	//------------------------------------------------------------------------------------------
	// Reads the CONFIG register value and returns the REF_CLK_SRC bit.
	// Returns 1 if the LDC161x is using an external oscillator, 0 if not
	// (just because it returns 0 doesn't mean there isn't an oscillator that physically exists)
	// The readConfig() function must be called to get the input for this function.
	//------------------------------------------------------------------------------------------
	bool LDC_hasExternalOscillator(uint16_t config_code);

	//------------------------------------------------------------------------------------------
	// Reads the CONFIG register value and returns the INTB_DIS bit.
	// NOTE THAT THIS FUNCTION CHECKS IF SOMETHING IS *DISABLED*, not ENABLED
	// Returns 1 if the INTB pin is disabled, 0 if it is enabled.
	// The readConfig() function must be called to get the input for this function.
	//------------------------------------------------------------------------------------------
	bool LDC_isInterruptDisabled(uint16_t config_code);

	//------------------------------------------------------------------------------------------
	// Reads the CONFIG register value and returns the HIGH_CURRENT_DRV bit.
	// 0: The LDC will drive all channels with normal sensor current (1.5 mA max).
	// 1: The LDC will drive channel 0 with current >1.5 mA.
	// The readConfig() function must be called to get the input for this function.
	//------------------------------------------------------------------------------------------
	bool LDC_isHighCurrentDriveEnabled(uint16_t config_code);

	//------------------------------------------------------------------------------------------
	// Refer to Section [7.6.27 Address 0x1E, DRIVE_CURRENT0] in the LDC1312 datasheet.
	// Sets the DRIVE_CURRENTx register via I2C.
	//------------------------------------------------------------------------------------------
	void LDC_setDriveCurrent(int channel, uint16_t drive_current);

	//------------------------------------------------------------------------------------------
	// Sets error config register bytes. Read Section [7.6.23 Address 0x19, ERROR_CONFIG] in
	// the LDC1614 datasheet for which bit does what. Make sure to respect reserved bits.
	//------------------------------------------------------------------------------------------
	void LDC_setErrorConfig(uint16_t config_code);

	//------------------------------------------------------------------------------------------
	// Reads and returns the entire CONFIG register via I2C.
	// This function doesn't come with the supporting functions that the other readConfig()
	// have - you'll have to go through the bits yourself if you want to see specific error
	// settings.
	//------------------------------------------------------------------------------------------
	uint16_t LDC_readErrorConfig();

	//------------------------------------------------------------------------------------------
	// Sets MUX config register bytes. Read Section [7.6.25 Address 0x1B, MUX_CONFIG] in
	// the LDC1614 datasheet for which bit does what. Make sure to respect reserved bits.
	//------------------------------------------------------------------------------------------
	void LDC_setMUXConfig(uint16_t config_code);

	//------------------------------------------------------------------------------------------
	// Reads the entire MUX_CONFIG register via I2C.
	//------------------------------------------------------------------------------------------
	uint16_t LDC_readMUXConfig();

	//------------------------------------------------------------------------------------------
	// Reads the MUX_CONFIG register value and returns the AUTOSCAN_EN bit.
	// 0: Continuous conversion on channel specified in CONFIG.ACTIVE_CHAN register bit.
	// 1: Auto-scans in sequence set by MUX_CONFIG.RR_SEQUENCE register bits.
	// The readMUXConfig() function must be called to get the input for this function.
	//------------------------------------------------------------------------------------------
	bool LDC_isAutoscanEnabled(uint16_t config_code);

	//------------------------------------------------------------------------------------------
	// Reads the MUX_CONFIG register value and returns the RR_SEQUENCE bits.
	// From datasheet: Configure multiplexing channel sequence. The LDC will perform a single
	//                 conversion on each channel in the sequence selected, and then restart 
	//                 the sequence continuously.
	// b00: Ch0, Ch1
	// b01: Ch0, Ch1, Ch2 (LDC1614 only)
	// b10: Ch0, Ch1, Ch2, Ch3 (LDC1614 only)
	// b11: Ch0, Ch1
	// The value is stored in the 2 least significant bits of the int this function returns.
	// The readMUXConfig() function must be called to get the input for this function.
	//------------------------------------------------------------------------------------------
	uint16_t LDC_getAutoscanSequence(uint16_t config_code);

	//------------------------------------------------------------------------------------------
	// Reads the MUX_CONFIG register value and returns the RR_SEQUENCE bits.
	// From datasheet: Select the lowest setting that exceeds the maximum sensor
	//                 oscillation frequency.
	// b001: 1.0 MHz
	// b100: 3.3 MHz
	// b101: 10 MHz
	// b111: 33 MHz
	// The value is stored in the 3 least significant bits of the int this function returns.
	// The readMUXConfig() function must be called to get the input for this function.
	//------------------------------------------------------------------------------------------
	uint16_t LDC_getDeglitchBandwidth(uint16_t config_code);

	//------------------------------------------------------------------------------------------
	// Reads the entire DRIVE_CURRENTx register via I2C.
	//------------------------------------------------------------------------------------------
	uint16_t LDC_readDriveCurrent(int channel);

	//------------------------------------------------------------------------------------------
	// Reads the DRIVE_CURRENTx register via I2C and returns just the manual drive current.
	//------------------------------------------------------------------------------------------
	uint16_t LDC_getDriveCurrent(int channel);

	//------------------------------------------------------------------------------------------
	// Reads the DRIVE_CURRENTx register via I2C and returns just the initial drive current
	// that is stored during auto-calibration.
	//------------------------------------------------------------------------------------------
	uint16_t LDC_getInitialDriveCurrent(int channel);
private:
};

#endif //end inclusion guard