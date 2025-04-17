#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Custom headers */
#include "misc/blink.h"
#include "misc/tickConversion.h"

#include "driver/gpio.h"

#include "i2c_bus.h"
#define I2C_MASTER_SDA_IO   (gpio_num_t)14       /*!< gpio number for I2C master data  */
#define I2C_MASTER_SCL_IO   (gpio_num_t)15       /*!< gpio number for I2C master clock */
#define I2C_MASTER_FREQ_HZ  100000               /*!< I2C master clock frequency */
#define ESP_SLAVE_ADDR      0x28                 /*!< ESP32 slave address, you can set any 7bit value */
#define DATA_LENGTH         64                   /*!< Data buffer length for test buffer*/

#include "bme280.h"
typedef int bme_err_t;
#define BME_OK 0x0
#define BME_ERR_HUMIDITY 0x1
#define BME_ERR_PRESSURE 0x2
#define BME_ERR_TEMPERATURE 0x3


#include "esp_log.h"
#define START_BLINK_COUNT 3
#define END_BLINK_COUNT 5
#define ERROR_BLINK_COUNT 7
#define WARNING_BLINK_COUNT 2
#define SUCCESS_BLINK_COUNT 1



/* @System declarations*/
// TODO: Change data types to esp_err_t and ensure errors are handled properly
struct BME_STRUCTURE{
    bme280_handle_t bmeHandlePtr;
    float temperature;
    float humidity;
    float pressure;
};

int systemInitializaton();

/* @I2C Declarations*/
static i2c_bus_handle_t i2cBusHandle = NULL;
static struct BME_STRUCTURE bme280 = {NULL, 0,0,0};
i2c_bus_handle_t i2cInit();
void updateBME(struct BME_STRUCTURE* bme);

void app_main(void){
    systemInitializaton();
    while(1){
        vTaskDelay(1000);
    }
}

int systemInitializaton(){
    char* taskName = pcTaskGetName(NULL);
    ESP_LOGI(taskName, "Prada Initializaton...");
    enum StatusType status = START;
    gpio_reset_pin(BLINK_LED);
    gpio_set_direction(BLINK_LED, GPIO_MODE_OUTPUT);
    logLEDSequence(taskName, status);
    i2cBusHandle= i2cInit();
    if(i2cBusHandle == NULL){
        ESP_LOGE(taskName, "I2C Initializaton failed...");
        return -1; // Going to handle this more properly later
    }

    // >! BME Initializaton
    bme280.bmeHandlePtr = bme280_create(i2cBusHandle,
                                        BME280_I2C_ADDRESS_DEFAULT);
    bme280_default_init(bme280.bmeHandlePtr);

    while(1){
        updateBME(&bme280);
        ESP_LOGI(taskName, "Humidity:%f", bme280.humidity);
        ESP_LOGI(taskName, "Pressure:%f", bme280.pressure);
        ESP_LOGI(taskName, "Temperature:%f",bme280.temperature);
        vTaskDelay(500);
    }
    return 1;
}




/* @I2C Definitions */
// Add proper error handling
i2c_bus_handle_t i2cInit(){
    i2c_bus_handle_t i2c_bus = NULL;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_bus = i2c_bus_create(I2C_NUM_0, &conf); // Returns NULL if failed
    return i2c_bus;
}

void updateBME(struct BME_STRUCTURE* bme){
    /* Error handling is done outside the function */
    bme280_read_humidity(bme->bmeHandlePtr, &bme->humidity);
    bme280_read_pressure(bme->bmeHandlePtr, &bme->pressure);
    bme280_read_temperature(bme->bmeHandlePtr, &bme->temperature);
}
