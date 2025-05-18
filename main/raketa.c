/*
                                                                       ███
                                                                      ███
                                                                     ███

   ████████████████████████████████████    ███████████████    █████████████████ ████████████       █████████████████       █████████
    █                  █             ███    █           ███    █            ████         █████      █            ████    █████████████
  ███           ███  ███             ███  ███           ███  ███            ████            ███   ███             ███   ███████████████
  ███           ███  ███             ███  ███           ███  ███            ████             ███  ███             ███   █  █   █   █  █
  █████████████████  ███ ███████████████  █████████████████  ███████████████████             ███  ███████████████████    █  █  █  █  █
  ████████████████   ███  ██████████████  ████████████████   ███  ██████████████             ███  ███  ██████████████     █ █  █ █  █
  ███                ███             ███  ███    █████       ███            ████            ███   ███             ███      █ █ █ ███
  ███                ███             ███  ███      █████     ███            ████         █████    ███             ███       ██████
  ███                ███             ███  ███        ████    ███            ████████████████      ███             ███         ███
                                                                                                                               █
*/
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
#include "misc/tickConversion.h"



#define I2C_MASTER_SDA_IO   (gpio_num_t)22
#define I2C_MASTER_SCL_IO   (gpio_num_t)23
#define I2C_MASTER_FREQ_HZ  100000
//#define ESP_SLAVE_ADDR      0x28
#define DATA_LENGTH         64
#define BUFFER_LENGTH 1024
#define UART_NUM UART_NUM_1

/* !< SPI definitions */
#define SCK 0
#define MISO 15
#define MOSI 2
#define SS 16
#define DIO0 17
#define RST 5





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
#define GPS_BUFFER 1024
void gpsInit(void);
void gpsUpdate(void);


/* @LORA */
#include "lora.h"
void taskTx(void *pvParameters);

/* @SYSTEM OUTPUT */
#define START 0x1
#define END 0x2
#define ERROR 0x3
#define WARNING 0x4
#define SUCCESS 0x5

#define LED_BUILTIN 33
#define LED_GREEN 34
#define LED_RED 35
typedef struct{
    uint8_t led;
    uint8_t count;
    uint16_t duration;
} led_cmd_t;
#define LED_QUEUE_LEN 10
QueueHandle_t ledQueue;
void logLEDSequence(const char* taskName, const uint8_t status);
void blink(uint8_t led);
void ledTask(void *pvParameters);

/* @Structure to tx*/
struct MEASURING_MODULES{
    struct BME_STRUCTURE bme280;
    struct MPU_STRUCTURE mpu6050;
    //char gpsBuffer[BUFFER_LENGTH];
};
int systemInitializaton(struct MEASURING_MODULES* modules);

/* @Error handling */
void espHandleError(const char* tag, esp_err_t err);


static esp_err_t err;
/* @TaskNames */
static const char *sxTag = "sx127x";
static const char* bmeTag = "BME280";
static const char* mpuTag = "MPU6050";
static const char* gpsTag = "GPS";
static const char* loraTag = "LORA";

void app_main(void){
    char* mainTask = "Main";
    struct MEASURING_MODULES modules = {};

    if(systemInitializaton(&modules) == 1)
        ESP_LOGI(mainTask, "System initialization successfull.");
    else{
        ESP_LOGW(mainTask, "System initialization unsuccessfull.");
    }
    // setup the lora module
	if (lora_init() == 0) {
		ESP_LOGE(loraTag, "Does not recognize the module");
		while(1) {
			vTaskDelay(1);
		}
	}
    ESP_LOGI(loraTag, "Settings the frequency to 433MHz");
    int cr = 1;
	int bw = 7;
	int sf = 7;
	lora_set_frequency(433e6); // 433MHz
	lora_set_coding_rate(cr);
	//lora_set_coding_rate(CONFIG_CODING_RATE);
	//cr = lora_get_coding_rate();
	ESP_LOGI(pcTaskGetName(NULL), "coding_rate=%d", cr);

	lora_set_bandwidth(bw);
	//lora_set_bandwidth(CONFIG_BANDWIDTH);
	//int bw = lora_get_bandwidth();
	ESP_LOGI(pcTaskGetName(NULL), "bandwidth=%d", bw);

	lora_set_spreading_factor(sf);
	//lora_set_spreading_factor(CONFIG_SF_RATE);
	//int sf = lora_get_spreading_factor();
	ESP_LOGI(pcTaskGetName(NULL), "spreading_factor=%d", sf);
    xTaskCreate(&taskTx, "TX", 1024*3, &modules, 5, NULL);
}

int systemInitializaton(struct MEASURING_MODULES* modules){
    char* sysInitTask = "System Initialization";
    ESP_LOGI(sysInitTask , "Starting Prada Initializaton...");
    gpio_reset_pin(LED_BUILTIN);
    gpio_reset_pin(LED_RED);
    gpio_reset_pin(LED_GREEN);
    gpio_set_direction(LED_BUILTIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_GREEN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_RED, GPIO_MODE_OUTPUT);
    ledQueue= xQueueCreate(LED_QUEUE_LEN, sizeof(led_cmd_t));
    xTaskCreate(ledTask, "LED_CONTROLLER", 2048, NULL, 10, NULL);
    logLEDSequence(sysInitTask, SUCCESS);


    // >! I2C Initialization
    i2cBusHandle = i2cInit();
    TEST_ASSERT_NOT_NULL_MESSAGE(i2cInit(),
            "I2C Initialization failed.");
    // >! GPS Initialization
    gpsInit();
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
void espHandleError(const char* tag, esp_err_t err){
    if(err != ESP_OK){
        ESP_LOGW(tag, " failed: %s", esp_err_to_name(err));
    }
}

void taskTx(void *pvParameters){
    ESP_LOGI(loraTag, "Transmit start");
    struct MEASURING_MODULES* modules = (struct MEASURING_MODULES*)pvParameters;
    while(1) {
        updateBME(&modules->bme280);
        updateMPU(&modules->mpu6050);
        ESP_LOGI(loraTag, "Temperature: %f", modules->bme280.temperature);
        ESP_LOGI(loraTag, "Humidity: %f", modules->bme280.humidity);
        ESP_LOGI(loraTag, "Pressure: %f", modules->bme280.pressure);
        ESP_LOGI(loraTag, "Temperature: %f",modules->mpu6050.temperature.temp);
        ESP_LOGI(loraTag, "acc_x: %f\tacc_y: %f\tacc_z: %f",
                modules->mpu6050.acceleration.acce_x,
                modules->mpu6050.acceleration.acce_y,
                modules->mpu6050.acceleration.acce_z);
        ESP_LOGI(loraTag, "gyro_x: %f\tgyro_y: %f\tgyro_z: %f",
                modules->mpu6050.gyroscope.gyro_x,
                modules->mpu6050.gyroscope.gyro_y,
                modules->mpu6050.gyroscope.gyro_z);
        lora_send_packet((uint8_t*)pvParameters,
                sizeof(struct MEASURING_MODULES));
        ESP_LOGI(loraTag, "%d byte packet sent...",
                sizeof(struct MEASURING_MODULES));
        int lost = lora_packet_lost();
        if (lost != 0) {
            ESP_LOGW(loraTag, "%d packets lost", lost);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    ESP_LOGI(loraTag, "Transmit end");
}

void gpsInit(void){
    const uart_port_t uart_num = UART_NUM_2;
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_PIN_NO_CHANGE, 12,
                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(uart_num, GPS_BUFFER, 0, 0, NULL, 0));
}


void gpsUpdate(void){
    char tempBuf[GPS_BUFFER];
    memset(tempBuf, 0, GPS_BUFFER);
    uart_read_bytes(UART_NUM_2, tempBuf, GPS_BUFFER, portMAX_DELAY);
    ESP_LOGI(gpsTag, "%s", tempBuf);
}

void logLEDSequence(const char* taskName, const uint8_t status) {
    led_cmd_t cmd = {0};

    switch (status) {
        case START:
            cmd.led = LED_GREEN;
            cmd.count = 1;
            cmd.duration = 100;
            xQueueSend(ledQueue, &cmd, 0);

            cmd.led= LED_RED;
            xQueueSend(ledQueue, &cmd, 0);

            cmd.led = LED_GREEN | LED_RED;
            xQueueSend(ledQueue, &cmd, 0);

            cmd.led = LED_BUILTIN;
            xQueueSend(ledQueue, &cmd, 0);
            break;

        case END:
            cmd.led = LED_RED;
            cmd.count = 3;
            cmd.duration = 100;
            xQueueSend(ledQueue, &cmd, 0);
            break;
        case WARNING:
            cmd.led = LED_GREEN | LED_RED;
            cmd.count = 5;
            cmd.duration = 100;
            xQueueSend(ledQueue, &cmd, 0);
            break;

        case ERROR:
        case SUCCESS:
            cmd.led = LED_GREEN;
            cmd.count = 3;
            cmd.duration = 100;
            xQueueSend(ledQueue, &cmd, 0);
            break;

        default:
            ESP_LOGW(taskName, "Not yet implemented.");
            break;
    }
}
void ledTask(void *pvParameters){
    led_cmd_t cmd;

    while (1) {
        if (xQueueReceive(ledQueue, &cmd, portMAX_DELAY)) {
            for (int i = 0; i < cmd.count; ++i) {
                if (cmd.led & LED_GREEN) gpio_set_level(LED_GREEN, 1);
                if (cmd.led & LED_RED)   gpio_set_level(LED_RED, 1);
                if (cmd.led & LED_BUILTIN)  gpio_set_level(LED_BUILTIN, 1);

                vTaskDelay(pdMS_TO_TICKS(cmd.duration));

                if (cmd.led & LED_GREEN) gpio_set_level(LED_GREEN, 0);
                if (cmd.led & LED_RED)   gpio_set_level(LED_RED, 0);
                if (cmd.led & LED_BUILTIN)  gpio_set_level(LED_BUILTIN, 0);

                vTaskDelay(pdMS_TO_TICKS(cmd.duration));
            }
        }
    }
}
