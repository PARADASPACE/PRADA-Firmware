#include "blink.h"

// Stupid LED blinking, might remove or make useful
void logLEDSequence(const char* taskName,enum StatusType status){
    switch (status) {
        case START:
            blink(3);
            break;
        default:
            ESP_LOGW(taskName,"Not yet implemented.");
            break;
    }
}
void blink(const int numOfBlinks){
            gpio_set_level(BLINK_LED, 1);
            vTaskDelay(milliseconds(1000));
            gpio_set_level(BLINK_LED, 0);
            vTaskDelay(milliseconds(1000));
            for(int i = 0; i < numOfBlinks;i++){
                gpio_set_level(BLINK_LED, 1);
                vTaskDelay(milliseconds(200));
                gpio_set_level(BLINK_LED, 0);
                vTaskDelay(milliseconds(200));
            }
            gpio_set_level(BLINK_LED, 0);
}
