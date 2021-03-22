// 8.1.5.2 Automatic IDRIVE Setting with RP_OVERRIDE_EN
// The automatic sensor amplitude setting is useful for initial system prototyping if the sensor amplitude is unknown.

#include <ldc1312_lib.h>
#include <Wire.h>

// Init the LDC - false in this case: no alternative address
LDC131X ldc1314(false);
uint16_t data;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  // This example is for channel 0
  // Reset just in case
  ldc1314.LDC_resetLDC();
  delay(500);
  // 1. Set target at the maximum planned operating distance from the sensor.
  // 2. Place the device into SLEEP mode by setting CONFIG.SLEEP_MODE_EN to b0.
  // On page 26 of the documentation is clearly stated: b1: Device is in Sleep Mode.
  // So this sketch will use b1 instead of b0.
  // This config will be used (Channel 0 and Sleep Mode enabled, everything else default):
  // 00 1 0 1 0 0 0 0 0 000001 -> 0x2801
  ldc1314.LDC_setConfig(0x2801);
  // 3. Program the desired values of SETTLECOUNT and RCOUNT values for the channel.
  // Here are the values used from 8.2.5 Recommended Initial Register Configuration Values (page 49).
  ldc1314.LDC_setConversionTime(0, 0x04D6); // RCOUNT
  ldc1314.LDC_setSettleTime(0, 0x000A); // SETTLECOUNT
  // 4. Enable auto-calibration by setting RP_OVERRIDE_EN to b0.
  // 5. Take the device out of SLEEP mode by setting CONFIG.SLEEP_MODE_EN to b1.
  // These steps are done in one step because both have to be done by setting CONFIG.
  // Again there seems to be a mistake with CONFIG.SLEEP_MODE_EN so it will now be set b0.
  // This config will be used (Channel 0, Sleep Mode disabled, RP Override enabled, everything else default):
  // 00 0 1 1 0 0 0 0 0 000001 -> 0x1801
  ldc1314.LDC_setConfig(0x1801);
  // 6. Allow the device to perform at least one measurement, with the target stable (fixed) at the maximum operating range.
  // Just in case a delay:
  delay(500);
  // 7. Read the channel current drive value from the appropriate DRIVE_CURRENTx register (addresses 0x1e, 0x1f, 0x20,or 0x21), in the INIT_DRIVEx field (bits 10:6). Save this value.
  // Or right it down somewhere from the Serial Monitor.
  data = ldc1314.LDC_getInitialDriveCurrent(0);
  // 8. During startup for normal operating mode, write the value saved from the INIT_DRIVEx bit field into the IDRIVEx bit field (bits 15:11).
  // 9. During normal operating mode, the RP_OVERRIDE_EN should be set to b1 for a fixed current drive.
}

void loop() {
  Serial.print("DRIVE_CURRENT0: ");
  Serial.println(data);
  delay(10000);
}

// This is an implementation like described in the documentation (8.1.5.2)
// Also it is a good example on how to use the library
// I (personally) didnt used the value I got here! I rather took what was stated in Table 39. Optimum Sensor Rp Ranges for Sensor IDRIVEx Setting in chapter 8.1.5
