#include "BMP280_I2C_Driver_Header.h"
#include <Arduino.h>
#include <Wire.h>

//Power modes
#define SLEEP_MODE ((unsigned char)0x0)
#define FORCED_MODE ((unsigned char)0x1)
#define NORMAL_MODE ((unsigned char)0x3)

//BMP280_Registers
enum {
    TEMP_XLSB = 0xFC,
    TEMP_LSB = 0xFB,
    TEMP_MSB = 0xFA,
    PRESS_XLSB = 0xF9,
    PRESS_LSB = 0xF8,
    PRESS_MSB = 0xF7,
    CONFIG = 0xF5,
    CTRL_MEAS = 0xF4,
    STATUS = 0xF3,
    RESET = 0xE0,
    ID = 0xD0,
    CALIB00 = 0x88
};

//Compensation Values
static unsigned long DIG_T1;
static signed long DIG_T2;
static signed long DIG_T3;
static unsigned long DIG_P1;
static signed long  DIG_P2;
static signed long  DIG_P3;
static signed long  DIG_P4;
static signed long  DIG_P5;
static signed long  DIG_P6;
static signed long  DIG_P7;
static signed long  DIG_P8;
static signed long  DIG_P9;

typedef struct {
    BMP280_S32_t temp;
    BMP280_U32_t press;

    // bit 0     : temp enabled (1), 
    // bit 1     : press enabled (1), 
    // bit 2     : forced_mode enabled (1), 
    // bits 3-5  : temp meas oversampling 
    // bits 6-8  : press meas oversampling
    // bits 9-15 : undefined 
    uint16_t config;
} measurements_t;

static unsigned char data[6]; // max read size is the 6 bytes of data containing pressure and temperature values
static unsigned char cmds[2]; // atmost only 2 registers can be written continuously so only 2 addresses need to be provided

// Temperature, Pressure and Flag values
// Flag description above in type defintion 
static measurements_t m;
 
// Data sheet recommends burst reading and writing meaning that instead of writing to/reading from one location at a time and 
//  terminating transmission you write/read to multiple at once using the BMP280s autoincrementing functionality. When reading the 
//  BMP280 will increment address automatically
static const unsigned char slave_address = 0x76; // 0b111 0110

static void bmp280_burst_read_reg(unsigned char * cmds, unsigned char * data, uint8_t data_size);
static void bmp280_burst_write_reg(unsigned char * cmds, uint8_t num_of_cmds, unsigned char * data);

// Load calibration values into compensation_words array for later use in calibrating adc values
static void bmp280_get_calibration_values(void);

// Assembles bytes --- data sheet specifices that 
static BMP280_S32_t assemble_measurement(unsigned char MSB, unsigned BYTE1, unsigned char LSB);

// Compensation temperature and pressure functions provided by data sheet
static BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T);
static BMP280_U32_t bmp280_compensate_P_int64(BMP280_S32_t adc_P);

// I2C Functions
static void bmp_i2c_init(void);
static void bmp280_i2c_transmit(unsigned char * cmds, uint8_t num_of_cmds, unsigned char * data);
static void bmp280_i2c_receive(unsigned char * cmds, unsigned char * data, uint8_t data_size);

void init_bmp280(Bmp280_config_t * cfg){
    // inititializes i2c communication
    bmp_i2c_init();

    // determines whether temp measurements, press measurements and forced mode are enabled  
    if (cfg->temp_measurement){
        m.config |= 0x01; 
    }
    if (cfg->press_measurement){
        m.config |= 0x02;
    }
    if (cfg->normal_or_forced_mode){
        m.config |= 0x04; 
    }
    m.config |= ((cfg->press_measurement) << 3);
    m.config |= ((cfg->temp_measurement) << 6);

    // When setting the initial mode if sensor is intended to be used in forced mode you start in sleep mode. If sensor 
    // will be used in normal mode you start it off in normal mode. When using forced mode you have to switch sensor to 
    // forced mode before taking a measurement. After every measurement the sensor returns to sleep mode. Measurements 
    // can be read once sensor is back in sleep mode.
    cmds[0] = CTRL_MEAS;
    data[0] |= ((cfg->temp_measurement << 5) | (cfg->press_measurement << 2) | ((cfg->normal_or_forced_mode) ? SLEEP_MODE : NORMAL_MODE));
    cmds[1] = CONFIG;
    data[1] |= (((cfg->normal_or_forced_mode) ? 0x0 : cfg->standby_time << 5) | (cfg->iir_filter << 2));
   
    bmp280_burst_write_reg(cmds, 2, data);
    
    //Check whether write was successful
    //bmp280_burst_read_reg(cmds, data, 2);

    // Ensures that the calibration values are updated before we starting reading from their registers.
    cmds[0] = STATUS;
    do {
        bmp280_burst_read_reg(cmds, data, 1);
    } while(*(data) & 0x01);
     
    bmp280_get_calibration_values();
}

void request_measurements(void){
    if (m.config & 0x04){
        cmds[0] = CTRL_MEAS;
        data[0] = (unsigned char)(((m.config >> 1) & 0xFC) | FORCED_MODE); 
        bmp280_burst_write_reg(cmds, 1, data);
    }

    // Ensures that measurements are ready to be read from temp and press registers
    cmds[0] = STATUS;
    do {
        bmp280_burst_read_reg(cmds, data, 1);
    } while(*(data) & 0x08);
    
    switch (m.config & 0x03){
        case 0x01: 
            cmds[0] = TEMP_MSB;
            bmp280_burst_read_reg(cmds, data, 3);
            m.temp = bmp280_compensate_T_int32(assemble_measurement(data[0], data[1], data[2])); // MSB : data[0], LSB : data[2] 
            break;
        case 0x02:
            cmds[0] = PRESS_MSB;
            bmp280_burst_read_reg(cmds, data, 3);

            m.press = bmp280_compensate_P_int64(assemble_measurement(data[0], data[1], data[2])); // MSB : data[0], LSB : data[2] 
            break;
        case 0x03:
            cmds[0] = PRESS_MSB;
            bmp280_burst_read_reg(cmds, data, 6);
            
            m.press = bmp280_compensate_P_int64(assemble_measurement(data[0], data[1], data[2])); // MSB : data[0], LSB : data[2]
            m.temp = bmp280_compensate_T_int32(assemble_measurement(data[3], data[4], data[5])); // MSB : data[0], LSB : data[2]
            break;
        default:
            break;
    }
}

BMP280_S32_t bmp280_get_temperature(void){
    return ((m.config & 0x01) ? (m.temp / 100) : 0x00);
}

BMP280_U32_t bmp280_get_pressure(void){
    return ((m.config & 0x02) ? (m.press / 25600): 0x00);
}

void bmp280_reset(void){
    cmds[0] = RESET;
    data[0] = 0xB6;
    bmp280_burst_write_reg(cmds, 1, data);
}

void bmp280_sleep(void){
    cmds[0] = CTRL_MEAS;
    data[0] = (unsigned char)(((m.config >> 1) & 0xFC) | SLEEP_MODE);
    bmp280_burst_write_reg(cmds, 1, data);
}

void bmp280_normal(void){
    cmds[0] = CTRL_MEAS;
    data[0] = (unsigned char)(((m.config >> 1) & 0xFC) | NORMAL_MODE);
    bmp280_burst_write_reg(cmds, 1, data);
}

void bmp280_focred(void){
    cmds[0] = CTRL_MEAS;
    data[0] = (unsigned char)(((m.config >> 1) & 0xFC) | FORCED_MODE);
    bmp280_burst_write_reg(cmds, 1, data);
} 

static void bmp280_burst_read_reg(unsigned char * cmds, unsigned char * data, uint8_t data_size){
    bmp280_i2c_receive(cmds, data, data_size);
}

static void bmp280_burst_write_reg(unsigned char * cmds, uint8_t num_of_cmds, unsigned char * data){
    bmp280_i2c_transmit(cmds, num_of_cmds, data);
}

static void bmp280_get_calibration_values(void){
    // gets calibration values from bmp280 
    // uses bmp280 built in auto increment for reading all values continously starting from CALIB00 
    unsigned char calibration_values[24];
    cmds[0] = CALIB00;
    bmp280_burst_read_reg(cmds, calibration_values, 24);

    DIG_T1 = (calibration_values[1] << 8) | calibration_values[0];
    DIG_T2 = (calibration_values[3] << 8) | calibration_values[2];
    DIG_T3 = (calibration_values[5] << 8) | calibration_values[4];
    DIG_P1 = ((calibration_values[7] << 8) | calibration_values[6]) & 0xFFFF;
    DIG_P2 = (calibration_values[9] << 8) | calibration_values[8];
    DIG_P3 = (calibration_values[11] << 8) | calibration_values[10];
    DIG_P4 = (calibration_values[13] << 8) | calibration_values[12];
    DIG_P5 = (calibration_values[15] << 8) | calibration_values[14];
    DIG_P6 = (calibration_values[17] << 8) | calibration_values[16];
    DIG_P7 = (calibration_values[19] << 8) | calibration_values[18];
    DIG_P8 = (calibration_values[21] << 8) | calibration_values[20];
    DIG_P9 = (calibration_values[23] << 8) | calibration_values[22];
}

// The datasheet expects the conversion from unsigned 20 bit int to signed long. The 8 bytes are assembled into a unsigned 24 
//  bit value which is then bitmasked to 20, since only 20 bits contain data. The value is then explicitly converted into a 
//  signed 32 long which is the type expected by the compensation functions.
static BMP280_S32_t assemble_measurement(unsigned char MSB, unsigned BYTE1, unsigned char LSB){ // big endian -- MSB to LSB
    return ((BMP280_S32_t)MSB << 12) | ((BMP280_S32_t)BYTE1 << 4) | ((BMP280_S32_t)LSB >> 4);
    
    // BUG !!!! 
    //return (MSB << 12 | BYTE1 << 4 | LSB >> 4);
}

// ======================================================== //
// ===== Compensation functions provided by datasheet ===== //
// ======================================================== //
static BMP280_S32_t t_fine;
static BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T){
        BMP280_S32_t var1, var2, T;
    var1 = ((((adc_T>>3) - ((BMP280_S32_t)DIG_T1<<1))) * ((BMP280_S32_t)DIG_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((BMP280_S32_t)DIG_T1)) * ((adc_T>>4) - ((BMP280_S32_t)DIG_T1)))>> 12) *((BMP280_S32_t)DIG_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

static BMP280_U32_t bmp280_compensate_P_int64(BMP280_S32_t adc_P){
    BMP280_S64_t var1, var2, p;
    var1 = ((BMP280_S64_t)t_fine) - 128000;
    var2 = var1 * var1 * (BMP280_S64_t)DIG_P6;
    var2 = var2 + ((var1*(BMP280_S64_t)DIG_P5)<<17);
    var2 = var2 + (((BMP280_S64_t)DIG_P4)<<35);
    var1 = ((var1 * var1 * (BMP280_S64_t)DIG_P3)>>8) + ((var1 * (BMP280_S64_t)DIG_P2)<<12);
    var1 = (((((BMP280_S64_t)1)<<47)+var1))*((BMP280_S64_t)DIG_P1)>>33;
    
    if (var1 == 0){
        return 0; // avoid exception caused by division by zero
    }

    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((BMP280_S64_t)DIG_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((BMP280_S64_t)DIG_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((BMP280_S64_t)DIG_P7)<<4);

    return (BMP280_U32_t)p;
}
// ======================================================== //
// ======================================================== //

static void bmp_i2c_init(void){
    Wire.begin();
    Serial.begin(9600);
}

static void bmp280_i2c_transmit(unsigned char * cmds, uint8_t num_of_cmds, unsigned char * data){
    Wire.beginTransmission(slave_address);

    for (int i = 0; i < num_of_cmds; i++){
        Wire.write(*(cmds+i));
        Wire.write(*(data+i));
    }

    Wire.endTransmission();
}

static void bmp280_i2c_receive(unsigned char * cmds, unsigned char * data, uint8_t data_size){
    // to read you must first send write address with desired register to read from and then 
    // send address in read mode and prepare for read.
    Wire.beginTransmission(slave_address);
    Wire.write(*cmds);
    Wire.endTransmission();

    Wire.requestFrom(slave_address, data_size); // request data_size number of bytes from peripheral device with address slave address

    int i = 0;
    while (Wire.available() && i < data_size){ // Wire.available returns number of bytes for reading or 0 if none are available
        *(data+i++) = Wire.read();
    }
}





