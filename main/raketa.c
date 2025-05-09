// This is the source code for a rocket build by the PARADASPACE team.
// It's purpose is to measure and provide
// useful data and emulate real life scenarios.
#include "freertos/FreeRTOS.h"

#include "freertos/task.h"
#include "memory.h"

#include "esp_log.h"
#include "esp_err.h"
#include "unity.h"

#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "hal/uart_types.h"
#include "i2c_bus.h"


/* Custom headers */
#include "misc/blink.h"
#include "misc/tickConversion.h"



#define I2C_MASTER_SDA_IO   (gpio_num_t)14
#define I2C_MASTER_SCL_IO   (gpio_num_t)15
#define I2C_MASTER_FREQ_HZ  100000
//#define ESP_SLAVE_ADDR      0x28
#define DATA_LENGTH         64
#define BUFFER_LENGTH 1024
#define UART_NUM UART_NUM_1

/* !< SPI definitions */
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define DIO0 26
#define RST 23



#define START_BLINK_COUNT 3
#define END_BLINK_COUNT 5
#define ERROR_BLINK_COUNT 7
#define WARNING_BLINK_COUNT 2
#define SUCCESS_BLINK_COUNT 1



/* @I2C */
static i2c_bus_handle_t i2cBusHandle = NULL;
i2c_bus_handle_t i2cInit();

/* @SPI */
spi_device_handle_t spiInit();

/* @BME */
#include "bme280.h"
struct BME_STRUCTURE{
    bme280_handle_t bmeHandle;
    float temperature;
    float humidity;
    float pressure;
};
void updateBME(struct BME_STRUCTURE* bme);

/* @MPU */
/*
 * mpu6050.h is using the new i2c driver!
*/
#include "mpu6050.h"
struct MPU_STRUCTURE{
    mpu6050_handle_t mpuHandle;
    mpu6050_acce_value_t acceleration;
    mpu6050_gyro_value_t gyroscope;
    mpu6050_temp_value_t temperature;
};
void updateMPU(struct MPU_STRUCTURE* mpu);

/* @GPS */
void gpsInit(char* gpsBuffer);
void gpsUpdate(char* gpsBuffer);


/* @LORA */
#include <esp_intr_alloc.h>
#include <sx127x.h>

sx127x loraDevice;
uint64_t loraFrequency = 434770012;
const UBaseType_t xArrayIndex = 0;

TaskHandle_t handle_interrupt;
// breakpoint
int current_power_level = 11;
void loraInit();



// >! esp_utils.h
void setup_gpio_interrupts(gpio_num_t gpio, sx127x *device,
        gpio_int_type_t type);
void sx127x_reset();
void sx127x_init_spi(spi_device_handle_t *handle);
esp_err_t setup_task(sx127x *device);
// setup_tx_task moved because of the modules structure declaration
void handle_interrupt_task(void *arg);
void IRAM_ATTR handle_interrupt_fromisr(void *arg);

/* @Structure to tx*/
struct MEASURING_MODULES{
    struct BME_STRUCTURE bme280;
    struct MPU_STRUCTURE mpu6050;
    char gpsBuffer[BUFFER_LENGTH];
};
int systemInitializaton(struct MEASURING_MODULES* modules);

/* @Error handling */
void espHandleError(const char* tag, esp_err_t err);


void loraTx(sx127x *device, struct MEASURING_MODULES* structureToTx);
esp_err_t setup_tx_task(sx127x *device, void (*tx_callback)(sx127x *device, struct MEASURING_MODULES* structureToTx));
void (*global_tx_callback)(sx127x *device, struct MEASURING_MODULES* structureToTx);
void handle_interrupt_tx_task(void *arg, struct MEASURING_MODULES* modules);

static esp_err_t err;
/* @TaskNames */
static const char *utilsTag = "esp_utils";
static const char *sxTag = "sx127x";
static const char* bmeTag = "BME280";
static const char* mpuTag = "MPU6050";
static const char* gpsTag = "GPS";

void app_main(void){
    char* mainTask = "Main";

    struct MEASURING_MODULES modules = {};

    if(systemInitializaton(&modules) == 1)
        ESP_LOGI(mainTask, "System initialization successfull.");
    else{
        ESP_LOGW(mainTask, "System initialization unsuccessfull.");
    }

    setup_tx_task(&loraDevice, loraTx);

    while(1){
        updateBME(&modules.bme280);
        updateMPU(&modules.mpu6050);

        ESP_LOGI(bmeTag, "Temperature: %f", modules.bme280.temperature);
        ESP_LOGI(bmeTag, "Humidity: %f", modules.bme280.humidity);
        ESP_LOGI(bmeTag, "Pressure: %f", modules.bme280.pressure);

        ESP_LOGI(mpuTag, "Temperature: %f",
                modules.mpu6050.temperature.temp);
        ESP_LOGI(mpuTag, "acc_x: %f\tacc_y: %f\tacc_z: %f",
                modules.mpu6050.acceleration.acce_x,
                modules.mpu6050.acceleration.acce_y,
                modules.mpu6050.acceleration.acce_z);
        ESP_LOGI(mpuTag, "gyro_x: %f\tgyro_y: %f\tgyro_z: %f",
                modules.mpu6050.gyroscope.gyro_x,
                modules.mpu6050.gyroscope.gyro_y,
                modules.mpu6050.gyroscope.gyro_z);
        //ESP_LOGI(mainTask, "GPS: %s", modules.gpsBuffer);
        vTaskDelay(seconds(1));
    }
}

int systemInitializaton(struct MEASURING_MODULES* modules){
    char* sysInitTask = "System Initialization";
    ESP_LOGI(sysInitTask , "Starting Prada Initializaton...");
    enum StatusType status = START;
    gpio_reset_pin(BLINK_LED);
    gpio_set_direction(BLINK_LED, GPIO_MODE_OUTPUT);
    logLEDSequence(sysInitTask, status);

    // >! Initialize lora module
    loraInit();
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
    vTaskDelay(milliseconds(600));

    return 1;
}

/* @I2C Definition */
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
    TEST_ASSERT_NOT_NULL_MESSAGE(i2c_bus, "I2C Bus initialization failed.");
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

void loraInit(){
    spi_device_handle_t loraHandle;
    sx127x_reset();
    sx127x_init_spi(&loraHandle);
    sx127x_create(loraHandle, &loraDevice);
    sx127x_set_opmod(SX127x_MODE_SLEEP, SX127x_MODULATION_LORA,
                &loraDevice);
    sx127x_set_frequency(loraFrequency, &loraDevice);
    sx127x_lora_reset_fifo(&loraDevice);
    sx127x_set_opmod(SX127x_MODE_STANDBY,
                SX127x_MODULATION_LORA, &loraDevice);
    sx127x_lora_set_bandwidth(SX127x_BW_125000, &loraDevice);
    sx127x_lora_set_implicit_header(NULL, &loraDevice);
    sx127x_lora_set_modem_config_2(SX127x_SF_9, &loraDevice);
    sx127x_lora_set_syncword(18, &loraDevice);
    sx127x_set_preamble_length(8, &loraDevice);
    sx127x_tx_set_callback(loraTx, &loraDevice);
    gpio_install_isr_service(0);
    setup_gpio_interrupts((gpio_num_t)DIO0, &loraDevice, GPIO_INTR_POSEDGE);

    sx127x_tx_set_pa_config(SX127x_PA_PIN_BOOST,
                current_power_level, &loraDevice);
    sx127x_tx_header_t header = {
        .enable_crc = true,
        .coding_rate = SX127x_CR_4_5};
    sx127x_lora_tx_set_explicit_header(&header, &loraDevice);

}
void loraTx(sx127x *device, struct MEASURING_MODULES* structureToTx) {
    /* 255 bytes just for testing
    uint8_t data[] =
        {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, 0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf, 0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf, 0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef, 0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe};
    */

    // not a final version, just a quick sketch
    sx127x_lora_tx_set_for_transmission(structureToTx, sizeof(structureToTx), device);
    sx127x_set_opmod(SX127x_MODE_TX, SX127x_MODULATION_LORA,
                device);
    ESP_LOGI(utilsTag, "Transmitting...");
}


void setup_gpio_interrupts(gpio_num_t gpio, sx127x *device,
        gpio_int_type_t type) {
  gpio_set_direction(gpio, GPIO_MODE_INPUT);
  gpio_pulldown_en(gpio);
  gpio_pullup_dis(gpio);
  gpio_set_intr_type(gpio, type);
  gpio_isr_handler_add(gpio, handle_interrupt_fromisr, (void *) device);
}

esp_err_t setup_task(sx127x *device) {
  BaseType_t task_code = xTaskCreatePinnedToCore(handle_interrupt_task,
          "handle interrupt", 8196, device, 2, &handle_interrupt,
          xPortGetCoreID());
  if (task_code != pdPASS) {
    ESP_LOGE(utilsTag, "can't create task %d", task_code);
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t setup_tx_task(sx127x *device, void (*tx_callback)(sx127x *device, struct MEASURING_MODULES* structureToTx)) {
  global_tx_callback = tx_callback;
  BaseType_t task_code = xTaskCreatePinnedToCore(handle_interrupt_tx_task,
          "handle interrupt", 8196, device,
          2, &handle_interrupt, xPortGetCoreID());
  if (task_code != pdPASS) {
    ESP_LOGE(utilsTag, "can't create task %d", task_code);
    return ESP_FAIL;
  }
  return ESP_OK;
}

void sx127x_reset() {
  ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t) RST, GPIO_MODE_OUTPUT));
  ESP_ERROR_CHECK(gpio_set_level((gpio_num_t) RST, 0));
  vTaskDelay(pdMS_TO_TICKS(5));
  ESP_ERROR_CHECK(gpio_set_level((gpio_num_t) RST, 1));
  vTaskDelay(pdMS_TO_TICKS(10));
  ESP_LOGI(utilsTag, "sx127x was reset");
}

void sx127x_init_spi(spi_device_handle_t *handle) {
  spi_bus_config_t config = {
      .mosi_io_num = MOSI,
      .miso_io_num = MISO,
      .sclk_io_num = SCK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0,
  };
  espHandleError(sxTag,spi_bus_initialize(SPI2_HOST, &config, 1));
  spi_device_interface_config_t dev_cfg = {
      .clock_speed_hz = 4E6,
      .spics_io_num = SS,
      .queue_size = 16,
      .command_bits = 0,
      .address_bits = 8,
      .dummy_bits = 0,
      .mode = 0};
  espHandleError(sxTag,spi_bus_add_device(SPI2_HOST, &dev_cfg, handle));
}
void handle_interrupt_task(void *arg) {
  while (1) {
    if (ulTaskNotifyTakeIndexed(xArrayIndex, pdTRUE, portMAX_DELAY) > 0) {
      sx127x_handle_interrupt((sx127x *) arg);
    }
  }
}
void IRAM_ATTR handle_interrupt_fromisr(void *arg) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveIndexedFromISR(handle_interrupt, xArrayIndex,
          &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
void handle_interrupt_tx_task(void *arg, struct MEASURING_MODULES* modules) {
  global_tx_callback((sx127x *) arg, modules);
  while (1) {
    if (ulTaskNotifyTakeIndexed(xArrayIndex, pdTRUE, portMAX_DELAY) > 0) {
      sx127x_handle_interrupt((sx127x *) arg);
    }
  }
}

void espHandleError(const char* tag, esp_err_t err){
    if(err != ESP_OK){
        ESP_LOGW(tag, " failed: %s", esp_err_to_name(err));
    }
}

