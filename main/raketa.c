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


#define LOG_MODULES 1

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


#define I2C_MASTER_SDA_IO   (gpio_num_t)21
#define I2C_MASTER_SCL_IO   (gpio_num_t)22
#define I2C_MASTER_FREQ_HZ  100000
//#define ESP_SLAVE_ADDR      0x28
#define DATA_LENGTH         64
#define BUFFER_LENGTH 1024
#define UART_NUM UART_NUM_1

/* !< SPI definitions */
/* Changed in lora.h lib*/
#define SCK 0
#define MISO 15
#define MOSI 2
#define SS 16
#define DIO0 17
#define RST 5


/* @I2C */
i2c_bus_handle_t i2cInit();

/* @SPI */
spi_device_handle_t spiInit();

/* @BME */
#include "bme280.h"
typedef struct{
    bme280_handle_t bmeHandle;
    float temperature;
    float humidity;
    float pressure;
} bme_structure_t;
void updateBME(bme280_handle_t* bme2080_h, bme_structure_t* bme);

/* @MPU */
/*
 * mpu6050.h is using the new i2c driver!
*/
#include "mpu6050.h"
typedef struct {
    mpu6050_handle_t mpuHandle;
    mpu6050_acce_value_t acceleration;
    mpu6050_gyro_value_t gyroscope;
    mpu6050_temp_value_t temperature;
} mpu_structure_t;
void updateMPU(mpu6050_handle_t* mpu6050_h, mpu_structure_t* mpu);

/* @GPS */
#define GPS_BUFFER 1024
#define GPS_PIN 13
void gpsInit(void);
void gpsUpdate(void);
typedef struct{
    float lat;
    float lon;
    float alt;
} gps_data_t;

/* @LORA */
typedef struct{
    int16_t temp_cx100;
    uint16_t humidity_px10;
    uint16_t pressure_hPax10;

    int16_t acce_x, acce_y, acce_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t mpu_temp_cx100;

    int32_t gps_lat_microdeg;
    int32_t gps_lon_microdeg;
    int32_t gps_alt_cm;

//    uint32_t timestamp_s;
    uint8_t crc8;
} __attribute__((packed)) sensor_packet_t;
#include "lora.h"
void taskTx(void *pvParameters);
sensor_packet_t build_sensor_packet(bme280_handle_t* bme_h,
                                    mpu6050_handle_t* mpu_h);
/* @SYSTEM OUTPUT */
#define START 0x1
#define END 0x2
#define ERROR 0x3
#define WARNING 0x4
#define SUCCESS 0x5

#define LED_BUILTIN 33
#define LED_GREEN 32
#define LED_RED 25
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

typedef struct {
    bme280_handle_t bme280_h;
    mpu6050_handle_t mpu6050_h;
} modules_handle_t;
int systemInitializaton(modules_handle_t* handles);

/* @Error handling */
void espHandleError(const char* tag, esp_err_t err);

/* @Compression */
uint8_t crc8(const uint8_t* data, size_t len);
/* @TaskNames */
static const char *sxTag = "sx127x";
static const char* bmeTag = "BME280";
static const char* mpuTag = "MPU6050";
static const char* gpsTag = "GPS";
static const char* loraTag = "LORA";

void app_main(void){
    char* mainTask = "Main";
    modules_handle_t handles = {};

    if(systemInitializaton(&handles) == 1){
        ESP_LOGI(mainTask, "System initialization successfull.");
        logLEDSequence("Init", SUCCESS);
    }else{
        ESP_LOGE(mainTask, "System initialization unsuccessfull.");
        logLEDSequence("Init", ERROR);
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
    xTaskCreate(&taskTx, "TX", 1024*3, &handles, 5, NULL);
}

int systemInitializaton(modules_handle_t* handles){
    esp_err_t err;
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
    i2c_bus_handle_t i2cBusHandle = i2cInit();
    TEST_ASSERT_NOT_NULL_MESSAGE(i2cInit(),
            "I2C Initialization failed.");
    // >! GPS Initialization
    gpsInit();
    // >>! BME280 Initializaton
    handles->bme280_h = bme280_create(i2cBusHandle,
                                        BME280_I2C_ADDRESS_DEFAULT);
    TEST_ASSERT_NOT_NULL_MESSAGE(handles->bme280_h,
            "BME280 Handle is NULL.");
    err = bme280_default_init(handles->bme280_h);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err,
            "BME280 Default initialization failed.");

    // >>! MPU6050 Initialization
    handles->mpu6050_h = mpu6050_create(i2cBusHandle,
                                                MPU6050_I2C_ADDRESS);
    TEST_ASSERT_NOT_NULL_MESSAGE(handles->mpu6050_h,
            "MPU6050 Handle is NULL.");
    err = mpu6050_config(handles->mpu6050_h,
            ACCE_FS_4G, GYRO_FS_500DPS);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err, "MPU6050 Config failed.");
    err = mpu6050_wake_up(handles->mpu6050_h);
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

void updateBME(bme280_handle_t* bme2080_h, bme_structure_t* bme){
    esp_err_t err;
    vTaskDelay(milliseconds(100));
    err = bme280_read_pressure(bme2080_h, &bme->pressure);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err,
            "BME280 Failed to read the pressure.");
    err = bme280_read_humidity(bme2080_h, &bme->humidity);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err,
            "BME280 Failed to read the humidity.");
    vTaskDelay(milliseconds(100));
    err = bme280_read_temperature(bme2080_h, &bme->temperature);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err,
            "BME280 Failed to read the temperature.");
    vTaskDelay(milliseconds(100));
}

void updateMPU(mpu6050_handle_t* mpu6050_h, mpu_structure_t* mpu){
    esp_err_t err;
    err = mpu6050_get_acce(mpu6050_h, &mpu->acceleration);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err,
            "MPU6050 Failed to read the acceleration.");
    if(err != ESP_OK){
        logLEDSequence("MPU acc", ERROR);
    }
    vTaskDelay(milliseconds(100));
    err = mpu6050_get_gyro(mpu6050_h, &mpu->gyroscope);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err,
            "MPU6050 Failed to read the gyroscope.");
    if(err != ESP_OK){
        logLEDSequence("MPU gyro", ERROR);
    }
    vTaskDelay(milliseconds(100));
    err = mpu6050_get_temp(mpu6050_h, &mpu->temperature);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err,
            "MPU6050 Failed to read the temperature.");
    if(err != ESP_OK){
        logLEDSequence("MPU temp", ERROR);
    }
    vTaskDelay(milliseconds(100));
}
void espHandleError(const char* tag, esp_err_t err){
    if(err != ESP_OK){
        ESP_LOGE(tag, " failed: %s", esp_err_to_name(err));
        logLEDSequence(tag, ERROR);
    }
}

void taskTx(void *pvParameters){
    ESP_LOGI(loraTag, "Transmit start");
    modules_handle_t* handles = (modules_handle_t*)pvParameters;
    while(1) {
        sensor_packet_t packet_tx = build_sensor_packet(handles->bme280_h, handles->mpu6050_h);
        lora_send_packet((uint8_t* )&packet_tx,
                sizeof(sensor_packet_t));
        logLEDSequence("LoraTX", SUCCESS);
        int lost = lora_packet_lost();
        if (lost != 0) {
            ESP_LOGW(loraTag, "%d packets lost", lost);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    ESP_LOGI(loraTag, "Transmit end");
    logLEDSequence("Lora", END);
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
    ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_PIN_NO_CHANGE, GPS_PIN,
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

sensor_packet_t build_sensor_packet(
        bme280_handle_t* bme_h,
        mpu6050_handle_t* mpu_h
        ){
    bme_structure_t bme_s;
    mpu_structure_t mpu_s;
    gps_data_t gps_s;
    updateBME(bme_h, &bme_s);
    updateMPU(mpu_h, &mpu_s);
#if LOG_MODULES
    ESP_LOGI(bmeTag, "Temperature: %f", bme_s.temperature);
    ESP_LOGI(bmeTag, "Humidity: %f", bme_s.humidity);
    ESP_LOGI(bmeTag, "Pressure: %f", bme_s.pressure);

    ESP_LOGI(mpuTag, "Temperature: %f",mpu_s.temperature.temp);
    ESP_LOGI(mpuTag, "acc_x: %f\tacc_y: %f\tacc_z: %f",
            mpu_s.acceleration.acce_x,
            mpu_s.acceleration.acce_y,
            mpu_s.acceleration.acce_z);
    ESP_LOGI(mpuTag, "gyro_x: %f\tgyro_y: %f\tgyro_z: %f",
            mpu_s.gyroscope.gyro_x,
            mpu_s.gyroscope.gyro_y,
            mpu_s.gyroscope.gyro_z);
#endif
    sensor_packet_t packet;
    packet.temp_cx100 = (int16_t)(bme_s.temperature*100);
    packet.humidity_px10 = (int16_t)(bme_s.humidity*10);
    packet.pressure_hPax10 = (int16_t)(bme_s.pressure*10);

    packet.acce_x = ((int16_t)mpu_s.acceleration.acce_x *1000);
    packet.acce_y = ((int16_t)mpu_s.acceleration.acce_y *1000);
    packet.acce_z = ((int16_t)mpu_s.acceleration.acce_z *1000);

    packet.gyro_x = ((int16_t)mpu_s.gyroscope.gyro_x*10);
    packet.gyro_y = ((int16_t)mpu_s.gyroscope.gyro_y*10);
    packet.gyro_z = ((int16_t)mpu_s.gyroscope.gyro_z*10);

    packet.gps_lat_microdeg = (int32_t)(gps_s.lat*1e6);
    packet.gps_lon_microdeg = (int32_t)(gps_s.lon*1e6);
    packet.gps_alt_cm = (int32_t)(gps_s.alt*100);

    //packet.timestamp_s = esp_timer_get_time() /1000000ULL;
    packet.crc8 = crc8((uint8_t*)&packet, sizeof(packet)-1);

    return packet;
}

// let's hope this is correct
uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; ++j) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
        }
    }
    return crc;
}

