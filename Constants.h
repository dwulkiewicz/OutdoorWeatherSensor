#pragma once

#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

const char* prefixClientID = "OutdoorWeather";

const char* sensorsBME280CommandTopic = "sensors/bme280/command";
const char* sensorsBME280CommandTemp = "temperature";
const char* sensorsBME280CommandHum = "humidity";
const char* sensorsBME280CommandPress = "pressure";
const char* sensorsBME280TemperatureTopic = "sensors/bme280/temperature";
const char* sensorsBME280HumidityTopic = "sensors/bme280/humidity";
const char* sensorsBME280PressureTopic = "sensors/bme280/pressure";


#define HOSTNAME_PREFIX "OutdoorWeather_" ///< Hostename. The setup function adds the Chip ID at the end.

#define BUILT_LED 16 //(0x10)
#define LED_ON 0x0
#define LED_OFF 0x1

#define BME280_I2C_ADDRESS 0x76
#define SEALEVELPRESSURE_HPA (1013.25)

#endif /* #ifndef __CONSTANTS_H__ */
