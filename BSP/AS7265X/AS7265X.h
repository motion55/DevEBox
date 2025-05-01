/*
 * AS7265X.h
 *
 *  Created on: May 1, 2025
 *      Author: Butch
 */

#ifndef AS7265X_AS7265X_H_
#define AS7265X_AS7265X_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

void AS7265X_begin(I2C_HandleTypeDef *hi2c);

uint8_t AS7265X_getTemperature(uint8_t deviceNumber); //Get temp in C of the master IC
float AS7265X_getTemperatureAverage();                    //Get average of all three ICs

void AS7265X_takeMeasurements();
void AS7265X_takeMeasurementsWithBulb();

void AS7265X_enableIndicator(); //Blue status LED
void AS7265X_disableIndicator();

void AS7265X_enableBulb(uint8_t device);
void AS7265X_disableBulb(uint8_t device);

void AS7265X_setGain(uint8_t gain);            //1 to 64x
void AS7265X_setMeasurementMode(uint8_t mode); //4 channel, other 4 channel, 6 chan, or 6 chan one shot
void AS7265X_setIntegrationCycles(uint8_t cycleValue);

void AS7265X_setBulbCurrent(uint8_t current, uint8_t device); //
void AS7265X_setIndicatorCurrent(uint8_t current);            //0 to 8mA

void AS7265X_enableInterrupt();
void AS7265X_disableInterrupt();

void AS7265X_softReset();

bool AS7265X_dataAvailable(); //Returns true when data is available

#ifdef __cplusplus
}
#endif

#endif /* AS7265X_AS7265X_H_ */
