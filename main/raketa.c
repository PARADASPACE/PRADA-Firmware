/* pins in use */
#include "unity_test_runner.h"
#define SDA_PIN 21
#define SCL_PIN 22
#define SCK_PIN 0
#define MISO_PIN 15
#define MOSI_PIN 2
#define SS_PIN 16
#define DIO0_PIN 17
#define RST_PIN 5

#define GPS_TX_PIN 13

#define EJECT_PIN 18
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

// --> Pošli svoje jméno do vesmíru <--
/* Viktorie Králová, Jakub Talián, Jiří Posavád, Robin Třeštík, Novos, Šimon, Matěj Šproch, Simona Rybářová,
 * Laura Žurovcová Josef Mičan, Eliška zápalková Tereza Staníková, Ellen Slívová, veronika scheyer, Sebastian Sačarovský, Pikolová Aneta, Julie Pustějovská, Julie Pustějovská,
 * Martin Molnár, vojtěch pustka, Kevin Trinh, Izabella Estera Lenczyková, Oliver Solčán, Kubina Vojtěch, Vojtěch Koliba, Daniel Ostárek,
 * alexander Vasak, Tadeáš Zbytek, Samuel Timčo, Štěpán Rotter, Adam zboch, Maryčka Popkova, Ema Pravdová, Božetěch Rak,
 * Matouš Zbytek, Oleksandr Sosonnyj , Sandra Bartonová, David Bednář, Xaver Buš, Adriana Jedličková, Laura Bilá, Chaewon Kim,
 * Ferdinand Cyrus, Klárka Černotíková, Klimánek Vít, Melanie Knapova, Nicolas Bilý, Sara Krejčí, Marek Tulej, Patrik Holeček,
 * Erik Krumpolc, Terka Šubertova, Michal Holinka, Jakub Hrstka, Ondřej Zlenko, Myslikovjan Marcus, Šimon Motloch, Ella Šiler,
 * Suchánek Dominik, Šimon Szotkowski, Pipi, Peter Norski, Majush Igam, Pepa Kolovrátek, Kvido Fišnar, Jan Chalupa,
 * Jonáš Kopřiva, Martin Kačer, Stefanie Šilarová, Alex Nevrkla, Klaudie Paseková, Marianna Pavlová, Michal Richter, Vít Morčinko,
 * Vojtěch Vývoda, Alena Šerá, Lukáš Hanzelka, Jakub Jastrezmbski, Emma Kovaříková, Vanesa Husárová, Daniel Zagóra, Adéla Horáková,
 * Lukáš Hanzelka, Vanesa Parišková, Daniel Jašek, Leontýna Hudcová, Vanessa Zahradníková, Veronika Klepáčová, Martin Vaverka, Adéla Havlíková,
 * Jitka Havlíková, Dana Holubová, Pravda Samuel, Honza Toman, Ctirad Obid , Michal Hrtoň, Nikola Veselá, Mariana Kuchařová, Barbora Kučerová,
 * Kopřivová Sofie Anna, Radim Strejček, Catherine Smith, Vojtěch Šafář, Kamila Baďurová, Fionn kilduff, Liubov Nikolenko , Velička jakub,
 * Roman Farkavec, Lukáš Hardyn, Lucie Špačková, Tomáš Zagóra, David Oprštěný, Veronika Jílková, Vilém Matějka, Dominika Nováková,
 * Dana Kaločová, Agáta Hanusová, Julie Ertlová, tereza havlikova, Eliška Řezníčková, Stella Gorková, Šarlota Jelínková, Mirek Řeha,
 * Adéla Bielčiková, Ondřej Jagoš, Jan Chlebek, Filip Kresta, Jakub Oborný, Dominik Hajdušek, Matěj Brychta, Daniil Zhulavskyi,
 * Filip Strašák, Jakub Gardoš, Michal Bury, Vladyslav Moskvin, Karin Witalová, Jakub Maďa, Petr Heczko, Matěj Martynek,
 * Šimon Diba, Darina Šuhajová, Jiří Sumbal, Simon Píštěk, Robin Balon, Libor Janouškovec, Martin Slaný, Vít Gelnar ,
 * Jindřich Měrka, Antonín Hyvnar, Marika Kačerová, Max Janda, Kristina Kupčíková, Vivien Švehlíková, Richard Vývoda, Jakub Zelenka,
 * Jindřich Weiss, Michaela Polanská, Isabela Rosová, Maruška Jandová, Johanna Palacká, Tadeáš Michálek, Maksym Ponomarov, Eva Lošáková,
 * Nela Sudková, Adéla Petrášová, Adéla Syrková, Ema Žáčková, Zuzana Řezníčková, Jačová Laura, tomáš Bajnar, Nikola Charlotte Rossi,
 * Pavel Alexa, Samoilenko Diana , Klaudie Skálová , Vojtěch Staňa, Tomáš Pivko, Daniel Jelič, Marie Natálie Kurucová, Ondřej Verner,
 * Šimon Ryba, Adéla Oprštěná, Kryštof Švehla, Anna Kološová, Veronika Kočvarová, Eva Žouželková, Lucie Koudelková, Eva Vylíčil Hradilová ,
 * Samuel Zajac, Aneta Dušková , Jakub Sedláček , Petra Hanáková , Jindřich Randa , Eliška Novotná , Karolína Jakubíková, Minka Kocsis,
 * Samko a pocitkovia, Vít Tech Holoubek , Erika Deanková, Stela Šaršonová, Martina Klvačová, Anna Koutná, Oravagamer, Martin Dryák,
 * Jonáš Neruda, Vojtěch Koukal, Mikuláš Kraj, Klára Dostálová, Patrícia Slivková, Roman Petružela, Julie Klímová, Valerie Lipková,
 * Karolína Kulová, Matyáš Černý, Andrea Marie Krocová, Bronislava Jandačková, Ellenka Smékalová, Julie Beritová, Tereza Jelínková, Peter Štefánik,
 * Marek Sotolář, Klára Drzewianá, Viola Pavlíková, Matěj Přibyl , Štěpán Pavlík, Matěj Sikora, Adéla Steinerová , Prokop Havlík,
 * Anna Revendová, Hana Macurová, Richard Břenek, Aleš Chrobok, Marek Klimonda, Jakub Látal, Veronika Bačová , Kristina Kirichenko,
 * Petr Fojtík, Martin Šerý, Lucie Koptíková, Ondřej Polka, Kalantajevskij Maksim, Ondra Charenza, Nela Hošková, Markéta Kadelová,
 * Sebastian Vojta, Damián Chalupa, Tomáš Komáčka, Štěpán Chalupa, Lucie Holotíková, Silvie Šimíčková, Adam Bezděk, Laura Pastrňáková,
 * Eliška Matějková, Jen Friedrich, Matyáš Eichner, Daniel Tužinský, Simona vyvialova, Jakub veelbil, Kristína Zimová, Jakub Gurecký,
 * Adam Přívara, Matyas Skacel, Adam Stýskal, Matěj Frydrych, Dominik Szkandera, Lina Mikulíková, Zbyněk Nytra, Karolína Sedláčková,
 * Adam Otáhal,Tobiáš Pitter, Matěj Gotzman , Tonda Kučera, Albert Bouchal, Alexandr Chovanec, Adam Fiala, Valerie Nitrová,
 * Milan Filgas, sarfad smrdi, Lenka Madryová, Jakub Chalupa, Vojtěch Brchaň, Teodor Fojtík, Jakub Turoň, Maxim Masare,
 * Marek Malčík, Daniel Tomis, Darina Šuhajová, Adam Hrtoň, Jonáš Hammer, Lukáš Gavel, Zuzana Bouchalová, Pavlína Smilkovska,
 * Martin Vošický, Samuel Hauer, Milan Kadeřábek, Vobecký Adam, Martin Pospíšil, Matěj Šimek, Jan Santarius, Simon Dobeš,
 * Anežka Kubenková, Silvie Říčanová, Lukas Sagan, Roman Kubaczka, Kateřina Nieslaníková, Bombardinokrokodilo, Tomáš Szczotka, Mikeš Kocour,
 * Ondrej Kazimir, Jakub Olszar, Tomáš Polok, Adam Kondrela, Eliška Janovská, Krystof Klecek, Tereza Marečková, Eliška Pohlová,
 * Jitka Roubalová, Mafynfv, Filip Zajacz, Matěj kusák, Filip Martinák, Mates Horák, Michael Mazur, Patrik Fiksek ,
 * Vojtěch Průcha, Adam Chemij, Elisabeth, Vontroba Max, Pavel Brutovsky, Vojtěch Ptáček, Michal Fadieiev, Valušek Michal,
 * Ottilie Kadeřábkováá, Tomáš Závodný, Hyvnar Samuel, Petr Maléř, David Michal Fencik, Emmanuel Ezeanyika, Vojtěch Dryák, Eva Dvořáková,
 * Veronika Seibertová, Natálie Klepáčová, Zuzana Honusová, Beáta Dryáková, Klára Huková , Jakub Holub, Jakub Farník, Daniel Kozel, Filip Kresta */


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
#include "math.h"


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

/* @Ejection charge */
void eject_parachute();




// <-------------------------------------------------------------------------->
/* @TaskNames */
static const char* bmeTag = "BME280";
static const char* mpuTag = "MPU6050";
static const char* gpsTag = "GPS";
static const char* loraTag = "LORA";
static const char* ejectionTag ="EJECTION";
// <-------------------------------------------------------------------------->


// <-------------------------------------------------------------------------->
/* @Ejection condition */
static int isHigh(const sensor_packet_t *packet);

// threshold needs to be configured!!!
const float threshold = 1.5f;

// alpha serves as a low-pass filter
const float alpha = 0.8f;

int launched = 0;
static float lastMin=0;
// <-------------------------------------------------------------------------->

#define lora_tx
void app_main(void){
    gpio_reset_pin(EJECT_PIN);
    gpio_set_direction(EJECT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(EJECT_PIN, 0);
    // @SYS INIT
    static modules_handle_t handles = {};
    systemInitializaton(&handles);

#ifndef lora_tx
    bme_structure_t bme_s = {};
    mpu_structure_t mpu_s = {};
    gps_data_t gps_s;
while(true){

    updateBME(handles.bme280_h, &bme_s);
    updateMPU(handles.mpu6050_h, &mpu_s);
    updateGPS(&gps_s);
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
    ESP_LOGI(gpsTag, "GPS_LON: %f\tGPS_LAT: %f\tGPS_ALT: %f", gps_s.lon, gps_s.lat, gps_s.alt);
    vTaskDelay(seconds(1));
}
#endif

#ifdef lora_tx
   // @LORA SETUP
   //
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

    xTaskCreate(&taskTx, "TX", 2048*3, &handles, 5, NULL);
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
    /*
    printf("\n📡 Initializing LoRa...\n");

    lora_init();
    lora_set_frequency(433E6);        // Set frequency to 433 MHz
    lora_enable_crc();
    lora_set_tx_power(17);            // Set max transmit power
    while (1) {
        **const char *message = "posilam !";
        printf("📤 Sending: %s\n", message);

        lora_send_packet((uint8_t *)message, strlen(message));

        vTaskDelay(pdMS_TO_TICKS(2000)); // wait 2 seconds
    }*/
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
        /* ejection charge is working no need to keep this code in here idk why
        vTaskDelay(pdMS_TO_TICKS(2000));
        gpio_set_level(EJECT_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(2000));
        gpio_set_level(EJECT_PIN, 0);
        */

        if(isHigh(&packet_tx) == 1){
            eject_parachute();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));

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
    uint8_t* gpsBuffer = malloc(GPS_BUFFER);
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
                        //printf("Time: %02d:%02d:%02d\n", frame.time.hours, frame.time.minutes, frame.time.seconds);
                        //printf("Lat: %f\n", minmea_tocoord(&frame.latitude));
                        //printf("Lon: %f\n", minmea_tocoord(&frame.longitude));
                        //printf("Altitude: %f meters\n", (double)frame.altitude.value);
                        _gps_s.alt = frame.altitude.value;
                        _gps_s.lat = minmea_tocoord(&frame.latitude);
                        _gps_s.lon = minmea_tocoord(&frame.longitude);
                    } else {
                       // printf("Failed to parse GGA sentence\n");
                    }
                }
            } else {
                printf("Invalid checksum or format\n");
                _gps_s.alt = 0;
                _gps_s.lat = 0;
                _gps_s.lon = 0;
            }
            line = strtok(NULL, "\n");
        }
    }
    free(gpsBuffer);
    *gps_s = _gps_s;
}


sensor_packet_t build_sensor_packet(
        bme280_handle_t bme_h,
        mpu6050_handle_t mpu_h
        ){
    static bme_structure_t bme_s = {};
    static mpu_structure_t mpu_s = {};
    static gps_data_t gps_s;
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

        .acce_x = ((int16_t)(mpu_s.acceleration.acce_x *1000)),
        .acce_y = ((int16_t)(mpu_s.acceleration.acce_y *1000)),
        .acce_z = ((int16_t)(mpu_s.acceleration.acce_z *1000)),

        .gyro_x = ((int16_t)(mpu_s.gyroscope.gyro_x*100)),
        .gyro_y = ((int16_t)(mpu_s.gyroscope.gyro_y*100)),
        .gyro_z = ((int16_t)(mpu_s.gyroscope.gyro_z*100)),
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

void checkStatus(){
    if(GLOBAL_ERROR != STATUS_OK){
        ESP_LOGE("ERROR", ,"",GLOBAL_ERROR);
    }
}
void eject_parachute(){
    gpio_set_level(EJECT_PIN, 1);
    ESP_LOGI(ejectionTag," ejection charge activated!!!");
}
#include "esp_timer.h"

static int isHigh(const sensor_packet_t *packet) {
    #define ACCEL_SCALE_FACTOR 16384.0f
    #define LAUNCH_ACCEL_THRESHOLD 0.4f   // g-force threshold for launch detection
    #define CHUTE_DEPLOY_DELAY_MS 13000

    // muzu prosim domu prosim prosim
    static float last_pressure = 0;
    static int falling_confirmed = 0;

    static int launch_detected = 0;
    static int chute_ready = 0;
    static int64_t launch_time = 0;

    // blah blh blah nerd
    float ax = packet->acce_x / ACCEL_SCALE_FACTOR;
    float ay = packet->acce_y / ACCEL_SCALE_FACTOR;
    float az = packet->acce_z / ACCEL_SCALE_FACTOR;

    float acc_mag = sqrtf(ax * ax + ay * ay + az * az);
    float pressure_hPa = packet->pressure_hPax10 / 10.0f;

    ESP_LOGI("FALL_CHECK", "Pressure: %.2f hPa, Acc Mag: %.2f g", pressure_hPa, acc_mag);

    // uz nemuzu asi se na to vyseru a jdu spat
    if (!launch_detected && acc_mag > LAUNCH_ACCEL_THRESHOLD) {
        launch_detected = 1;
        launch_time = esp_timer_get_time(); // microseconds
        ESP_LOGW("LAUNCH_DETECTED", "raketovy obed detekovan! Acc=%.2f g", acc_mag);
    }

    if (launch_detected && !chute_ready) {
        int64_t now = esp_timer_get_time();
        if ((now - launch_time) >= CHUTE_DEPLOY_DELAY_MS * 1000) {
            chute_ready = 1;
            ESP_LOGW("CHUTE_DEPLOY_READY", "zapinam po 13s padak");
        }
    }

    if (last_pressure == 0) {
        last_pressure = pressure_hPa;
        return 0;
    }

    float pressure_change = pressure_hPa - last_pressure;
    last_pressure = pressure_hPa;

    // jestli to nepujde tak se poseru na miste prisaham
    if (pressure_change > 0.4f && acc_mag < 0.3f) {
        ESP_LOGW("FALL_DETECTED", "raketka pada! ΔP=%.2f hPa, acc=%.2f g", pressure_change, acc_mag);
        falling_confirmed = 1;
    }

    return chute_ready;
}

