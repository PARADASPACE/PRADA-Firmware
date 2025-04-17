#include "tickConversion.h"
#include "portmacro.h"
TickType_t milliseconds(const int ms){
    return ms/portTICK_PERIOD_MS;
}
TickType_t seconds(const int s){
    return milliseconds(s*1000);
}
