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
enum {
    DIG_T1,
    DIG_T2,
    DIG_T3,
    DIG_P1,
    DIG_P2,
    DIG_P3,
    DIG_P4,
    DIG_P5,
    DIG_P6,
    DIG_P7,
    DIG_P8,
    DIG_P9
};

typedef struct {
    BMP280_S32_t temp;
    BMP280_U32_t press;
    uint8_t flags; // 0 : temp enabled (1), 1 : press enabled (1), 2-7 undefined 
} measurements_t;
// defining 2 bools will take up 2 bytes, so using bit flags will save a byte 


// Arrays for burst read and write
static signed long compensation_words[12]; // compensation words indexed by compensation values
static unsigned char data[6]; // max read size is the 6 bytes of data containing pressure and temperature values
static unsigned char cmds[2]; // atmost only 2 registers can be written continuously so only 2 addresses need to be provided

// Temperature and Pressure values for 
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

    // determines whether temp and press measurements are enabled
    if (cfg->temp_measurement){
        m.flags |= 0x01; 
    }
    if (cfg->press_measurement){
        m.flags |= 0x02;
    }

    cmds[0] = CTRL_MEAS;
    data[0] |= ((cfg->temp_measurement << 5) | (cfg->press_measurement << 2));
    cmds[1] = CONFIG;
    data[1] |= (((cfg->normal_or_forced_mode) ? 0x0 : cfg->standby_time << 5) | (cfg->iir_filter << 2)); 

    bmp280_burst_write_reg(cmds, 2, data);
    
    //bmp280_burst_read_reg(cmds, data, 2);   
}

void request_measurements(void){
    if (m.flags == 0x01){
        cmds[0] = TEMP_MSB;
        bmp280_burst_read_reg(cmds, data, 3);

        m.temp = bmp280_compensate_T_int32(assemble_measurement(data[0], data[1], data[2])); // MSB : data[0], LSB : data[2] 
    }
    else if (m.flags == 0x02){
        cmds[0] = PRESS_MSB;
        bmp280_burst_read_reg(cmds, data, 3);

        m.press = bmp280_compensate_P_int64(assemble_measurement(data[0], data[1], data[2])); // MSB : data[0], LSB : data[2] 
    }
    else if (m.flags == 0x03){
        cmds[0] = PRESS_MSB;
        bmp280_burst_read_reg(cmds, data, 6);
        
        m.press = bmp280_compensate_P_int64(assemble_measurement(data[0], data[1], data[2])); // MSB : data[0], LSB : data[2]
        m.temp = bmp280_compensate_T_int32(assemble_measurement(data[3], data[4], data[5])); // MSB : data[0], LSB : data[2]
    }
}

BMP280_S32_t bmp280_get_temperature(void){
    return ((m.flags & 0x01) ? m.temp: 0x00);
}

BMP280_U32_t bmp280_get_pressure(void){
    return ((m.flags & 0x02) ? m.press: 0x00);
}

void bmp280_reset(void){
    cmds[0] = RESET;
    data[0] = 0xB6;
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

    // ====================================================== //
    // == implicit cast from unsigned short to signed long == //
    // ====================================================== //
    // investigate ...
    // ====================================================== //
    for (int i = 0, j = 0; i < 24; i+=2){
        compensation_words[j++] = calibration_values[i+1] << 8 | calibration_values[i];
    }
    // ============================================= //
    // ============================================= //
}

// The datasheet expects the conversion from unsigned 20 bit int to signed long. The 8 bytes are assembled into a unsigned 24 
//  bit value which is then bitmasked to 20, since only 20 bits contain data. The value is then explicitly converted into a 
//  signed 32 long which is the type expected by the compensation functions.
static BMP280_S32_t assemble_measurement(unsigned char MSB, unsigned BYTE1, unsigned char LSB){ // big endian -- MSB to LSB
    return (BMP280_S32_t)((MSB << 16 | BYTE1 << 8 | LSB) & 0x0FFFFF);
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





