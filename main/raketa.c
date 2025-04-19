#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "unity.h"

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
/*
 * mpu6050.h is using the new i2c driver
*/
#include "mpu6050.h"


#include "esp_log.h"
#define START_BLINK_COUNT 3
#define END_BLINK_COUNT 5
#define ERROR_BLINK_COUNT 7
#define WARNING_BLINK_COUNT 2
#define SUCCESS_BLINK_COUNT 1



/* @System declarations*/
// TODO: Change data types to esp_err_t and ensure errors are handled properly
struct BME_STRUCTURE{
    bme280_handle_t bmeHandle;
    float temperature;
    float humidity;
    float pressure;
};
struct MPU_STRUCTURE{
    mpu6050_handle_t mpuHandle;
    mpu6050_acce_value_t acceleration;
    mpu6050_gyro_value_t gyroscope;
    mpu6050_temp_value_t temperature;
};

struct MEASURING_MODULES{
    struct BME_STRUCTURE bme280;
    struct MPU_STRUCTURE mpu6050;
};

int systemInitializaton(struct MEASURING_MODULES* modules);

/* @I2C Declarations*/
static i2c_bus_handle_t i2cBusHandle = NULL;
i2c_bus_handle_t i2cInit();
void updateBME(struct BME_STRUCTURE* bme);
void updateMPU(struct MPU_STRUCTURE* mpu);

static esp_err_t err;

void app_main(void){
    char* taskName = pcTaskGetName(NULL);
    struct MEASURING_MODULES modules = {};
    if(systemInitializaton(&modules) == 1)
        ESP_LOGI(taskName, "System initialization successfull.");
    else{
        ESP_LOGW(taskName, "System initialization unsuccessfull.");
        // blah blah blah...
    }
    while(1){
        updateBME(&modules.bme280);
        updateMPU(&modules.mpu6050);
        ESP_LOGI(taskName, "BME: Temperature: %f", modules.bme280.temperature);
        ESP_LOGI(taskName, "BME: Humidity: %f", modules.bme280.humidity);
        ESP_LOGI(taskName, "BME: Pressure: %f", modules.bme280.pressure);

        ESP_LOGI(taskName, "MPU: Temperature: %f", modules.mpu6050.temperature.temp);
        ESP_LOGI(taskName, "MPU: acc_x: %f\tacc_y: %f\tacc_z: %f",
                modules.mpu6050.acceleration.acce_x,
                modules.mpu6050.acceleration.acce_y,
                modules.mpu6050.acceleration.acce_z);
        ESP_LOGI(taskName, "MPU: gyro_x: %f\tgyro_y: %f\tgyro_z: %f",
                modules.mpu6050.gyroscope.gyro_x,
                modules.mpu6050.gyroscope.gyro_y,
                modules.mpu6050.gyroscope.gyro_z);
        vTaskDelay(seconds(1));
    }
}

int systemInitializaton(struct MEASURING_MODULES* modules){
    char* taskName = pcTaskGetName(NULL);
    ESP_LOGI(taskName, "Starting Prada Initializaton...");
    enum StatusType status = START;
    gpio_reset_pin(BLINK_LED);
    gpio_set_direction(BLINK_LED, GPIO_MODE_OUTPUT);
    logLEDSequence(taskName, status);

    // >! I2C Initialization
    i2cBusHandle = i2cInit();
    TEST_ASSERT_NOT_NULL_MESSAGE(i2cInit(), "I2C Initialization failed.");
    // >>! BME280 Initializaton
    modules->bme280.bmeHandle = bme280_create(i2cBusHandle,
                                        BME280_I2C_ADDRESS_DEFAULT);
    TEST_ASSERT_NOT_NULL_MESSAGE(modules->bme280.bmeHandle, "BME280 Handle is NULL.");
    err = bme280_default_init(modules->bme280.bmeHandle);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err, "BME280 Default initialization failed.");

    // >>! MPU6050 Initialization
    modules->mpu6050.mpuHandle = mpu6050_create(i2cBusHandle,
                                                MPU6050_I2C_ADDRESS);
    TEST_ASSERT_NOT_NULL_MESSAGE(modules->mpu6050.mpuHandle, "MPU6050 Handle is NULL.");
    err = mpu6050_config(modules->mpu6050.mpuHandle, ACCE_FS_4G, GYRO_FS_500DPS);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err, "MPU6050 Config failed.");
    err = mpu6050_wake_up(modules->mpu6050.mpuHandle);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err, "MPU6050 Waking up failed.");

    vTaskDelay(milliseconds(500));

    return 1;
}

/* @I2C Definitions */
i2c_bus_handle_t i2cInit(){
    i2c_bus_handle_t i2c_bus = NULL;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_bus = i2c_bus_create(I2C_NUM_0, &conf);
    TEST_ASSERT_NOT_NULL_MESSAGE(i2c_bus, "i2C bus initialization failed.");
    return i2c_bus;
}

void updateBME(struct BME_STRUCTURE* bme){
    err = bme280_read_humidity(bme->bmeHandle, &bme->humidity);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err, "BME280 Failed to read the humidity.");
    err = bme280_read_pressure(bme->bmeHandle, &bme->pressure);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err, "BME280 Failed to read the pressure.");
    vTaskDelay(milliseconds(100));
    err = bme280_read_temperature(bme->bmeHandle, &bme->temperature);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err, "BME280 Failed to read the temperature.");
    vTaskDelay(milliseconds(100));
}

void updateMPU(struct MPU_STRUCTURE* mpu){
    err = mpu6050_get_acce(mpu->mpuHandle, &mpu->acceleration);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err, "MPU6050 Failed to read the acceleration.");
    vTaskDelay(milliseconds(100));
    err = mpu6050_get_gyro(mpu->mpuHandle, &mpu->gyroscope);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err, "MPU6050 Failed to read the gyroscope.");
    vTaskDelay(milliseconds(100));
    err = mpu6050_get_temp(mpu->mpuHandle, &mpu->temperature);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err, "MPU6050 Failed to read the temperature.");
    vTaskDelay(milliseconds(100));
}

