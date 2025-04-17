#pragma once
#ifndef __tickConversion_H__
#define __tickConversion_H__
#include "freertos/FreeRTOS.h"
TickType_t milliseconds(const int ms);
TickType_t seconds(const int s);
#endif
