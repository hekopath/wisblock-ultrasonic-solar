/**
 * @file main.h
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief Includes, definitions and global declarations for DeepSleep example
 * @version 0.1
 * @date 2020-08-15
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include <Arduino.h>
#include <SPI.h>
#include <LoRaWan-RAK4630.h>

// Comment the next line if you want DEBUG output. Note, this uses way more battery when enabled.
#define MAX_SAVE

// LoRaWan stuff
int8_t initLoRaWan(void);
bool sendLoRaFrame(float Temperature, float Humidity, float Pressure, float SoilMoisture, float GasLevel, float BatteryLevel);
extern SemaphoreHandle_t loraEvent;

// Main loop stuff
void periodicWakeup(TimerHandle_t unused);
extern SemaphoreHandle_t taskEvent;
extern uint8_t rcvdLoRaData[];
extern uint8_t rcvdDataLen;
extern uint8_t eventType;
extern SoftwareTimer taskWakeupTimer;

// Battery functions
/** Definition of the Analog input that is connected to the battery voltage divider */
#define PIN_VBAT A0
/** Definition of milliVolt per LSB => 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096 */
#define VBAT_MV_PER_LSB (0.73242188F)
/** Voltage divider value => 1.5M + 1M voltage divider on VBAT = (1.5M / (1M + 1.5M)) */
#define VBAT_DIVIDER (0.4F)
/** Compensation factor for the VBAT divider */
#define VBAT_DIVIDER_COMP (1.73)
/** Fixed calculation of milliVolt from compensation value */
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)
void initReadVBAT(void);
float readVBAT(void);
uint8_t readBatt(void);