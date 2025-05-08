#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/uart_types.h"
#include "unity.h"
#include "memory.h"

/* Custom headers */
#include "misc/blink.h"
#include "misc/tickConversion.h"

#include "driver/gpio.h"
#include "i2c_bus.h"
#include "driver/uart.h"
#define I2C_MASTER_SDA_IO   (gpio_num_t)14       /*!< gpio number for I2C master data  */
#define I2C_MASTER_SCL_IO   (gpio_num_t)15       /*!< gpio number for I2C master clock */
#define I2C_MASTER_FREQ_HZ  100000               /*!< I2C master clock frequency */
#define ESP_SLAVE_ADDR      0x28                 /*!< ESP32 slave address, you can set any 7bit value */
#define DATA_LENGTH         64                   /*!< Data buffer length for test buffer*/
#define BUFFER_LENGTH 1024
#define UART_NUM UART_NUM_1


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

/*
 * Lora sx1278-smt definitions and functions
*/
#include "sx127x.h"
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5
#define PIN_NUM_RST  14
#define PIN_NUM_DIO0 26

static struct sx127x_t *lora = NULL;

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
    char gpsBuffer[BUFFER_LENGTH];
};

int systemInitializaton(struct MEASURING_MODULES* modules);

/* @I2C Declarations*/
static i2c_bus_handle_t i2cBusHandle = NULL;
i2c_bus_handle_t i2cInit();
void updateBME(struct BME_STRUCTURE* bme);
void updateMPU(struct MPU_STRUCTURE* mpu);

/* @GPS Declarations */
void gpsInit(char* gpsBuffer);
void gpsUpdate(char* gpsBuffer);


// lora testing
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_intr_alloc.h>
#include <esp_log.h>
#include <freertos/task.h>
#include <sx127x.h>
#include "misc/esp_utils.h"

static const char *TAG = "sx127x";

sx127x device;
int messages_sent = 0;
int supported_power_levels[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 20};
int supported_power_levels_count = sizeof(supported_power_levels) / sizeof(int);
int current_power_level = 0;

void tx_callback(sx127x *device) {
  if (messages_sent > 0) {
    ESP_LOGI(TAG, "transmitted");
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  if (messages_sent == 0) {
    uint8_t data[] = {0xCA, 0xFE};
    ESP_ERROR_CHECK(sx127x_lora_tx_set_for_transmission(data, sizeof(data), device));
  } else if (messages_sent == 1) {
    // 200 bytes
    uint8_t data[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, 0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7};
    ESP_ERROR_CHECK(sx127x_lora_tx_set_for_transmission(data, sizeof(data), device));
  } else if (messages_sent == 2) {
    // 255 bytes
    uint8_t data[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, 0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf, 0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf, 0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef, 0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe};
    ESP_ERROR_CHECK(sx127x_lora_tx_set_for_transmission(data, sizeof(data), device));
  } else if (current_power_level < supported_power_levels_count) {
    uint8_t data[] = {0xCA, 0xFE};
    ESP_ERROR_CHECK(sx127x_lora_tx_set_for_transmission(data, sizeof(data), device));
    ESP_ERROR_CHECK(sx127x_tx_set_pa_config(SX127x_PA_PIN_BOOST, supported_power_levels[current_power_level], device));
    current_power_level++;
  } else {
    return;
  }
  ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_LORA, device));
  ESP_LOGI(TAG, "transmitting");
  messages_sent++;
}


// lora testing


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
    ESP_LOGI(TAG, "starting up");
    sx127x_reset();

    spi_device_handle_t spi_device;
    sx127x_init_spi(&spi_device);

    ESP_ERROR_CHECK(sx127x_create(spi_device, &device));
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_LORA, &device));
    ESP_ERROR_CHECK(sx127x_set_frequency(434770012, &device));
    ESP_ERROR_CHECK(sx127x_lora_reset_fifo(&device));
    ESP_ERROR_CHECK(sx127x_set_opmod(SX127x_MODE_STANDBY, SX127x_MODULATION_LORA, &device));
    ESP_ERROR_CHECK(sx127x_lora_set_bandwidth(SX127x_BW_125000, &device));
    ESP_ERROR_CHECK(sx127x_lora_set_implicit_header(NULL, &device));
    ESP_ERROR_CHECK(sx127x_lora_set_modem_config_2(SX127x_SF_9, &device));
    ESP_ERROR_CHECK(sx127x_lora_set_syncword(18, &device));
    ESP_ERROR_CHECK(sx127x_set_preamble_length(8, &device));
    sx127x_tx_set_callback(tx_callback, &device);

    gpio_install_isr_service(0);
    setup_gpio_interrupts((gpio_num_t)DIO0, &device, GPIO_INTR_POSEDGE);

    ESP_ERROR_CHECK(sx127x_tx_set_pa_config(SX127x_PA_PIN_BOOST, supported_power_levels[current_power_level], &device));
    sx127x_tx_header_t header = {
        .enable_crc = true,
        .coding_rate = SX127x_CR_4_5};
    ESP_ERROR_CHECK(sx127x_lora_tx_set_explicit_header(&header, &device));

    ESP_ERROR_CHECK(setup_tx_task(&device, tx_callback));
    while(1){
        updateBME(&modules.bme280);
        updateMPU(&modules.mpu6050);
        ESP_LOGI(taskName, "BME: Temperature: %f", modules.bme280.temperature);
        ESP_LOGI(taskName, "BME: Humidity: %f", modules.bme280.humidity);
        ESP_LOGI(taskName, "BME: Pressure: %f", modules.bme280.pressure);

        ESP_LOGI(taskName, "MPU: Temperature: %f",
                modules.mpu6050.temperature.temp);
        ESP_LOGI(taskName, "MPU: acc_x: %f\tacc_y: %f\tacc_z: %f",
                modules.mpu6050.acceleration.acce_x,
                modules.mpu6050.acceleration.acce_y,
                modules.mpu6050.acceleration.acce_z);
        ESP_LOGI(taskName, "MPU: gyro_x: %f\tgyro_y: %f\tgyro_z: %f",
                modules.mpu6050.gyroscope.gyro_x,
                modules.mpu6050.gyroscope.gyro_y,
                modules.mpu6050.gyroscope.gyro_z);
        gpsUpdate(modules.gpsBuffer);
        ESP_LOGI(taskName, "GPS: %s", modules.gpsBuffer);
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
    TEST_ASSERT_NOT_NULL_MESSAGE(i2cInit(),
            "I2C Initialization failed.");
    // >>! BME280 Initializaton
    modules->bme280.bmeHandle = bme280_create(i2cBusHandle,
                                        BME280_I2C_ADDRESS_DEFAULT);
    TEST_ASSERT_NOT_NULL_MESSAGE(modules->bme280.bmeHandle,
            "BME280 Handle is NULL.");
    err = bme280_default_init(modules->bme280.bmeHandle);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err,
            "BME280 Default initialization failed.");

    // >>! MPU6050 Initialization
    modules->mpu6050.mpuHandle = mpu6050_create(i2cBusHandle,
                                                MPU6050_I2C_ADDRESS);
    TEST_ASSERT_NOT_NULL_MESSAGE(modules->mpu6050.mpuHandle,
            "MPU6050 Handle is NULL.");
    err = mpu6050_config(modules->mpu6050.mpuHandle,
            ACCE_FS_4G, GYRO_FS_500DPS);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err, "MPU6050 Config failed.");
    err = mpu6050_wake_up(modules->mpu6050.mpuHandle);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err, "MPU6050 Waking up failed.");

    // >! GPS Initialization
    gpsInit(modules->gpsBuffer);
    vTaskDelay(milliseconds(600));

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
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err,
            "BME280 Failed to read the humidity.");
    vTaskDelay(milliseconds(100));
    err = bme280_read_pressure(bme->bmeHandle, &bme->pressure);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err,
            "BME280 Failed to read the pressure.");
    vTaskDelay(milliseconds(100));
    err = bme280_read_temperature(bme->bmeHandle, &bme->temperature);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err,
            "BME280 Failed to read the temperature.");
    vTaskDelay(milliseconds(100));
}

void updateMPU(struct MPU_STRUCTURE* mpu){
    err = mpu6050_get_acce(mpu->mpuHandle, &mpu->acceleration);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err,
            "MPU6050 Failed to read the acceleration.");
    vTaskDelay(milliseconds(100));
    err = mpu6050_get_gyro(mpu->mpuHandle, &mpu->gyroscope);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err,
            "MPU6050 Failed to read the gyroscope.");
    vTaskDelay(milliseconds(100));
    err = mpu6050_get_temp(mpu->mpuHandle, &mpu->temperature);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err,
            "MPU6050 Failed to read the temperature.");
    vTaskDelay(milliseconds(100));
}

void gpsInit(char* gpsBuffer){
    uart_config_t uartConfig = {
        .baud_rate= 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uartConfig));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, 12,
                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUFFER_LENGTH * 2,
                0, 0, NULL, 0));
    vTaskDelay(seconds(1));
}
void gpsUpdate(char* gpsBuffer) {
    /*
       memset(gpsBuffer, 0, BUFFER_LENGTH);
       int idx = 0;
       while (idx < BUFFER_LENGTH - 1) {
       uint8_t byte;
       int len = uart_read_bytes(UART_NUM, &byte, 1, pdMS_TO_TICKS(100));  // Wait 100ms for a byte
       if (len > 0) {
       gpsBuffer[idx++] = byte;
       if (byte == '\n') {
       gpsBuffer[idx] = '\0';
       return;
       }
       } else {
       break;
       }
       }
       strcpy(gpsBuffer, "No full GPS sentence received.");
       */
    memset(gpsBuffer, 0, BUFFER_LENGTH);
    int idx = 0;
    while (idx < BUFFER_LENGTH - 1) {
        uint8_t byte;
        uart_read_bytes(UART_NUM, &byte, 1, pdMS_TO_TICKS(100));  // Wait 100ms for a byte
        gpsBuffer[idx++] = byte;
        if (byte == '\n') {
            gpsBuffer[idx] = '\0';
        }
    }
}

