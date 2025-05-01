/*
 * AS7265X.c
 *
 *  Created on: May 1, 2025
 *      Author: Butch
 */
#include "main.h"
#include "SparkFun_AS7265X.h"
#include "AS7265X.h"

class AS7265X AS7265X_sensor;

void AS7265X_begin(I2C_HandleTypeDef *hi2c)
{
	AS7265X_sensor.begin(hi2c);
}

uint8_t AS7265X_getTemperature(uint8_t deviceNumber)
{
	return AS7265X_sensor.getTemperature(deviceNumber);
}

float AS7265X_getTemperatureAverage()
{
	return AS7265X_sensor.getTemperatureAverage();
}

void AS7265X_takeMeasurements()
{
	AS7265X_sensor.takeMeasurements();
}

void AS7265X_takeMeasurementsWithBulb()
{
	AS7265X_sensor.takeMeasurementsWithBulb();
}

void AS7265X_enableIndicator()
{
	AS7265X_sensor.enableIndicator();
}

void AS7265X_disableIndicator()
{
	AS7265X_sensor.disableIndicator();
}

void AS7265X_enableBulb(uint8_t device)
{
	AS7265X_sensor.enableBulb(device);
}

void AS7265X_disableBulb(uint8_t device)
{
	AS7265X_sensor.disableBulb(device);
}

void AS7265X_setGain(uint8_t gain)
{
	AS7265X_sensor.setGain(gain);
}

void AS7265X_setMeasurementMode(uint8_t mode)
{
	AS7265X_sensor.setMeasurementMode(mode);
}

void AS7265X_setIntegrationCycles(uint8_t cycleValue)
{
	AS7265X_sensor.setIntegrationCycles(cycleValue);
}

void AS7265X_setBulbCurrent(uint8_t current, uint8_t device)
{
	AS7265X_sensor.setBulbCurrent(current, device);
}

void AS7265X_setIndicatorCurrent(uint8_t current)
{
	AS7265X_sensor.setIndicatorCurrent(current);
}

void AS7265X_enableInterrupt()
{
	AS7265X_sensor.enableInterrupt();
}

void AS7265X_disableInterrupt()
{
	AS7265X_sensor.disableInterrupt();
}

void AS7265X_softReset()
{
	AS7265X_sensor.softReset();
}

bool AS7265X_dataAvailable()
{
	return AS7265X_sensor.dataAvailable();
}
