/* pins in use */
#define SDA_PIN 21
#define SCL_PIN 22
#define SCK_PIN 0
#define MISO_PIN 15
#define MOSI_PIN 2
#define SS_PIN 16
#define DIO0_PIN 17
#define RST_PIN 5

#define GPS_TX_PIN 13
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
  ███                ███     cr8 check        ███  ███      █████     ███            ████         █████    ███             ███       ██████
  ███                ███             ███  ███        ████    ███            ████████████████      ███             ███         ███
                                                                                                                               █
*/
// This is the source code for a rocket build by the PARADASPACE team.
// It's purpose is to measure and provide
// useful data and emulate real life scenarios.


#include "hal/spi_types.h"
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


/* @Errors */
typedef enum{
    STATUS_OK,
    ERR_I2C_INIT_INVALID_ARG,
    ERR_I2C_INIT_DRIVER_INSTALL,
    ERR_BME_HANDLE_NULL,
    ERR_BME_DEF_INIT_FAIL,
    ERR_MPU_HANDLE_NULL,
    ERR_MPU_CONFIG_FAIL,
    ERR_MPU_WAKE_UP,
    ERR_BME_UPDATE_PRESSURE_READ_FAIL,
    ERR_BME_UPDATE_TEMPERATURE_READ_FAIL,
    ERR_BME_UPDATE_HUMIDITY_READ_FAIL,
    ERR_MPU_UPDATE_TEMPERATURE_READ_FAIL,
    ERR_MPU_UPDATE_GYROSCOPE_READ_FAIL,
    ERR_MPU_UPDATE_ACCELERATION_READ_FAIL

} error_status_t;
error_status_t GLOBAL_ERROR;
void checkStatus();

/* Custom headers */
#include "misc/tickConversion.h"
// lib author: https://github.com/kosma/minmea
#include "components/minmea/minmea.h"


#define I2C_MASTER_SDA_IO   (gpio_num_t)SDA_PIN
#define I2C_MASTER_SCL_IO   (gpio_num_t)SCL_PIN
#define I2C_MASTER_FREQ_HZ  100000
//#define ESP_SLAVE_ADDR      0x28
#define DATA_LENGTH         64
#define BUFFER_LENGTH 1024
#define UART_NUM UART_NUM_1




// <--------------------------------------------------------------------------->
/* !< SPI definitions */
/* Changed in lora.h lib*/
#define SCK SCK_PIN
#define MISO MISO_PIN
#define MOSI MOSI_PIN
#define SS SS_PIN
#define DIO0 DIO0_PIN
#define RST RST_PIN
// <--------------------------------------------------------------------------->


// <--------------------------------------------------------------------------->
/* @I2C */
i2c_bus_handle_t i2cInit();
// <--------------------------------------------------------------------------->




// <--------------------------------------------------------------------------->
/* @BME */
#include "bme280.h"
typedef struct{
    bme280_handle_t bmeHandle;
    float temperature;
    float humidity;
    float pressure;
} bme_structure_t;
void updateBME(bme280_handle_t bme2080_h, bme_structure_t* bme);
// <--------------------------------------------------------------------------->






// <--------------------------------------------------------------------------->
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
void updateMPU(mpu6050_handle_t mpu6050_h, mpu_structure_t* mpu);
// <--------------------------------------------------------------------------->










// <--------------------------------------------------------------------------->
/* @GPS */
#define GPS_BUFFER 1024
#define GPS_PIN GPS_TX_PIN
typedef struct{
    float lat;
    float lon;
    float alt;
} gps_data_t;
void gpsInit(void);
void updateGPS(gps_data_t* gps_s);
// <--------------------------------------------------------------------------->







// <--------------------------------------------------------------------------->
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
sensor_packet_t build_sensor_packet(bme280_handle_t bme_h,
                                    mpu6050_handle_t mpu_h);
// <--------------------------------------------------------------------------->



// <--------------------------------------------------------------------------->


typedef struct {
    bme280_handle_t bme280_h;
    mpu6050_handle_t mpu6050_h;
} modules_handle_t;
void systemInitializaton(modules_handle_t* handles);


/* @Error handling */
void espHandleError(const char* tag, esp_err_t err);

/* @Verification */
uint8_t crc8(const uint8_t* data, size_t len);





// <-------------------------------------------------------------------------->
/* @TaskNames */
static const char* bmeTag = "BME280";
static const char* mpuTag = "MPU6050";
static const char* gpsTag = "GPS";
static const char* loraTag = "LORA";
// <-------------------------------------------------------------------------->



//#define lora_tx
void app_main(void){
    // @SYS INIT
    static modules_handle_t handles = {};
    systemInitializaton(&handles);
    bme_structure_t bme_s = {};
    mpu_structure_t mpu_s = {};
    gps_data_t gps_s;
while(true){

    updateBME(handles.bme280_h, &bme_s);
    updateMPU(handles.mpu6050_h, &mpu_s);
    updateGPS(&gps_s);
#if LOG_MODULES
    ESP_LOGI(bmeTag, "Temperature: %.2f", bme_s.temperature);
    ESP_LOGI(bmeTag, "Humidity: %.2f", bme_s.humidity);
    ESP_LOGI(bmeTag, "Pressure: %.2f", bme_s.pressure);

    ESP_LOGI(mpuTag, "");
    ESP_LOGI(mpuTag, "MPU Temperature: %.2f",mpu_s.temperature.temp);
    ESP_LOGI(mpuTag, "acc_x: %.2f\tacc_y: %.2f\tacc_z: %.2f",
            mpu_s.acceleration.acce_x,
            mpu_s.acceleration.acce_y,
            mpu_s.acceleration.acce_z);
    ESP_LOGI(mpuTag, "gyro_x: %.2f\tgyro_y: %.2f\tgyro_z: %.2f",
            mpu_s.gyroscope.gyro_x,
            mpu_s.gyroscope.gyro_y,
            mpu_s.gyroscope.gyro_z);
    //ESP_LOGI(gpsTag, "GPS_LON: %f\tGPS_LAT: %f\tGPS_ALT: %f", gps_s.lon, gps_s.lat, gps_s.alt);
#endif
    vTaskDelay(seconds(1));
}

#ifdef lora_tx
   // @LORA SETUP
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
#endif
}

void systemInitializaton(modules_handle_t* handles){
// -----------------------------------------------------------
    esp_err_t err;
    GLOBAL_ERROR = STATUS_OK;
    char* sysInitTask = "System Initialization";
    ESP_LOGI(sysInitTask , "Starting Parada Initializaton...");
// -----------------------------------------------------------


    // >! I2C Initialization
// -----------------------------------------------------------
    i2c_bus_handle_t i2cBusHandle = i2cInit();
    checkStatus();
    GLOBAL_ERROR = STATUS_OK;


    // >! GPS Initialization
// -----------------------------------------------------------
    gpsInit();
    checkStatus();
    GLOBAL_ERROR = STATUS_OK;


    // >>! BME280 Initializaton
// -----------------------------------------------------------
    handles->bme280_h = bme280_create(i2cBusHandle,
                                        BME280_I2C_ADDRESS_DEFAULT);
    if(handles->bme280_h == NULL)
        GLOBAL_ERROR = ERR_BME_HANDLE_NULL;
    checkStatus();
    GLOBAL_ERROR = STATUS_OK;
    err = bme280_default_init(handles->bme280_h);
    if(err != ESP_OK)
        GLOBAL_ERROR = ERR_BME_DEF_INIT_FAIL;
    checkStatus();
    GLOBAL_ERROR = STATUS_OK;

    // >>! MPU6050 Initialization
// -----------------------------------------------------------
    handles->mpu6050_h = mpu6050_create(i2cBusHandle,
                                                MPU6050_I2C_ADDRESS);
    if(handles->mpu6050_h == NULL)
        GLOBAL_ERROR = ERR_MPU_HANDLE_NULL;
    checkStatus();
    GLOBAL_ERROR = STATUS_OK;
    err = mpu6050_config(handles->mpu6050_h,
            ACCE_FS_4G, GYRO_FS_500DPS);
    if(err != ESP_OK)
        GLOBAL_ERROR = ERR_MPU_CONFIG_FAIL;
    checkStatus();
    GLOBAL_ERROR = STATUS_OK;
    err = mpu6050_wake_up(handles->mpu6050_h);
    if(err != ESP_OK)
        GLOBAL_ERROR = ERR_MPU_WAKE_UP;
    checkStatus();
    GLOBAL_ERROR = STATUS_OK;
}







// <--------------------------------------------------------------------------->
/* @I2C definition */
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
    if(i2c_bus == NULL)
        GLOBAL_ERROR = ERR_I2C_INIT_DRIVER_INSTALL;
    return i2c_bus;
}
// <--------------------------------------------------------------------------->

void updateBME(bme280_handle_t bme2080_h, bme_structure_t* bme){
    GLOBAL_ERROR = STATUS_OK;
    esp_err_t err;
    vTaskDelay(milliseconds(300));

    ESP_LOGI(bmeTag, "Reading pressure...");
    err = bme280_read_pressure(bme2080_h, &bme->pressure);
    if(err != ESP_OK)
        GLOBAL_ERROR = ERR_BME_UPDATE_PRESSURE_READ_FAIL;
    checkStatus();
    GLOBAL_ERROR = STATUS_OK;

    ESP_LOGI(bmeTag, "Reading humidity...");
    err = bme280_read_humidity(bme2080_h, &bme->humidity);
    if(err != ESP_OK)
        GLOBAL_ERROR = ERR_BME_UPDATE_HUMIDITY_READ_FAIL;
    checkStatus();
    GLOBAL_ERROR = STATUS_OK;

    vTaskDelay(milliseconds(300));

    ESP_LOGI(bmeTag, "Reading temperature...");
    err = bme280_read_temperature(bme2080_h, &bme->temperature);
    if(err != ESP_OK)
        GLOBAL_ERROR = ERR_BME_UPDATE_TEMPERATURE_READ_FAIL;
    checkStatus();
    GLOBAL_ERROR = STATUS_OK;

    vTaskDelay(milliseconds(300));
    ESP_LOGI(bmeTag, "bme2080_h: %p", bme2080_h);

}
// <--------------------------------------------------------------------------->




void updateMPU(mpu6050_handle_t mpu6050_h, mpu_structure_t* mpu){
    esp_err_t err;
    err = mpu6050_get_acce(mpu6050_h, &mpu->acceleration);
    if(err != ESP_OK){
        GLOBAL_ERROR = ERR_MPU_UPDATE_ACCELERATION_READ_FAIL;
    }
    checkStatus();
    GLOBAL_ERROR = STATUS_OK;
    vTaskDelay(milliseconds(50));
    err = mpu6050_get_gyro(mpu6050_h, &mpu->gyroscope);
    if(err != ESP_OK){
        GLOBAL_ERROR = ERR_MPU_UPDATE_GYROSCOPE_READ_FAIL;
    }
    checkStatus();
    GLOBAL_ERROR = STATUS_OK;
    vTaskDelay(milliseconds(50));
    err = mpu6050_get_temp(mpu6050_h, &mpu->temperature);
    if(err != ESP_OK){
        GLOBAL_ERROR = ERR_MPU_UPDATE_TEMPERATURE_READ_FAIL;
    }
    checkStatus();
    GLOBAL_ERROR = STATUS_OK;
    vTaskDelay(milliseconds(50));
}

// <--------------------------------------------------------------------------->


void taskTx(void *pvParameters){
    ESP_LOGI(loraTag, "Transmit start");
    modules_handle_t* handles = (modules_handle_t*)pvParameters;
    while(1) {
        //ESP_LOGI(loraTag, "bme handle at TX loop: %p", handles->bme280_h);
        sensor_packet_t packet_tx = build_sensor_packet(handles->bme280_h, handles->mpu6050_h);
        lora_send_packet((uint8_t* )&packet_tx,
                sizeof(sensor_packet_t));
        ESP_LOGI(loraTag, "PACKET SENT SUCCESSFULLY!");
        int lost = lora_packet_lost();
        if (lost != 0) {
            ESP_LOGW(loraTag, "%d packets lost", lost);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    ESP_LOGI(loraTag, "Transmit end");
}



// <--------------------------------------------------------------------------->




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




// <--------------------------------------------------------------------------->




void updateGPS(gps_data_t* gps_s){
    uint8_t gpsBuffer[GPS_BUFFER];
    memset(gpsBuffer, 0, GPS_BUFFER);
    gps_data_t _gps_s;
    int len = uart_read_bytes(UART_NUM_2, gpsBuffer, GPS_BUFFER, 100/portTICK_PERIOD_MS);
    if (len > 0) {
        ESP_LOGI("GPS", "Received %d bytes from GPS:", len);
        gpsBuffer[len] = '\0';
        char *line = strtok((char*)gpsBuffer, "\n");
        while(line != NULL){
            printf("NMEA line: %s\n", line);
              if (minmea_check(line, false)) {
                if (minmea_sentence_id(line, false) == MINMEA_SENTENCE_GGA) {
                    struct minmea_sentence_gga frame;
                    if (minmea_parse_gga(&frame, line)) {
                        printf("Time: %02d:%02d:%02d\n", frame.time.hours, frame.time.minutes, frame.time.seconds);
                        printf("Lat: %f\n", minmea_tocoord(&frame.latitude));
                        printf("Lon: %f\n", minmea_tocoord(&frame.longitude));
                        printf("Altitude: %f meters\n", frame.altitude.value / 1000.0);
                    } else {
                        printf("Failed to parse GGA sentence\n");
                    }
                }
            } else {
                printf("Invalid checksum or format\n");
            }
            line = strtok(NULL, "\n");
        }
    }
}
sensor_packet_t build_sensor_packet(
        bme280_handle_t bme_h,
        mpu6050_handle_t mpu_h
        ){
    bme_structure_t bme_s = {};
    mpu_structure_t mpu_s = {};
    gps_data_t gps_s;
    updateBME(bme_h, &bme_s);
    updateMPU(mpu_h, &mpu_s);
    updateGPS(&gps_s);
#if LOG_MODULES
    ESP_LOGI(bmeTag, "Temperature: %.2f", bme_s.temperature);
    ESP_LOGI(bmeTag, "Humidity: %.2f", bme_s.humidity);
    ESP_LOGI(bmeTag, "Pressure: %.2f", bme_s.pressure);

    ESP_LOGI(mpuTag, "MPU Temperature: %.2f",mpu_s.temperature.temp);
    ESP_LOGI(mpuTag, "acc_x: %.2f\tacc_y: %.2f\tacc_z: %.2f",
            mpu_s.acceleration.acce_x,
            mpu_s.acceleration.acce_y,
            mpu_s.acceleration.acce_z);
    ESP_LOGI(mpuTag, "gyro_x: %.2f\tgyro_y: %.2f\tgyro_z: %.2f",
            mpu_s.gyroscope.gyro_x,
            mpu_s.gyroscope.gyro_y,
            mpu_s.gyroscope.gyro_z);
    ESP_LOGI(gpsTag, "GPS_LON: %f\tGPS_LAT: %f\tGPS_ALT: %f", gps_s.lon, gps_s.lat, gps_s.alt);
#endif
    sensor_packet_t packet = {
        .temp_cx100 = (int16_t)(bme_s.temperature*100),
        .humidity_px10 = (int16_t)(bme_s.humidity*10),
        .pressure_hPax10 = (int16_t)(bme_s.pressure*10),

        .acce_x = ((int16_t)mpu_s.acceleration.acce_x *1000),
        .acce_y = ((int16_t)mpu_s.acceleration.acce_y *1000),
        .acce_z = ((int16_t)mpu_s.acceleration.acce_z *1000),

        .gyro_x = ((int16_t)mpu_s.gyroscope.gyro_x*10),
        .gyro_y = ((int16_t)mpu_s.gyroscope.gyro_y*10),
        .gyro_z = ((int16_t)mpu_s.gyroscope.gyro_z*10),
        .mpu_temp_cx100 = (int16_t)(mpu_s.temperature.temp * 100),
        .gps_lat_microdeg = (int32_t)(gps_s.lat*1e6),
        .gps_lon_microdeg = (int32_t)(gps_s.lon*1e6),
        .gps_alt_cm = (int32_t)(gps_s.alt*100),


        //packet.timestamp_s = esp_timer_get_time() /1000000ULL;
    };
    packet.crc8 = crc8((uint8_t*)&packet, sizeof(packet)-1);

    return packet;
}
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
/*

#define CONFIG_DIO0_GPIO 26

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora.h"

void app_main(void) {
    printf("\n📡 Initializing LoRa...\n");

    lora_init();
    lora_set_frequency(433E6);        // Set frequency to 433 MHz
    lora_enable_crc();
    lora_set_tx_power(17);            // Set max transmit power

    while (1) {
        const char *message = "posilam !";
        printf("📤 Sending: %s\n", message);

        lora_send_packet((uint8_t *)message, strlen(message));

        vTaskDelay(pdMS_TO_TICKS(2000)); // wait 2 seconds
    }
}
*/
void checkStatus(){
    if(GLOBAL_ERROR != STATUS_OK){
        ESP_LOGE("ERROR", ,"",GLOBAL_ERROR);
    }
}
