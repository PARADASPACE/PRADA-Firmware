#pragma once
#ifndef __BLINK_H__
#define __BLINK_H__

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "tickConversion.h"

#define BLINK_LED 33
enum StatusType{
    START, END, ERROR, WARNING,SUCCESS
};
void logLEDSequence(const char* taskName, enum StatusType status);
void blink(const int numOfBlinks);
#endif
