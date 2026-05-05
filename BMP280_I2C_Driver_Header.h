#ifndef BMP280_DRIVER_HEADER_H
#define BMP280_DRIVER_HEADER_H

typedef signed long BMP280_S32_t;
typedef unsigned long BMP280_U32_t;
typedef signed long long BMP280_S64_t;

// BMP280 configuration union
typedef union {
    struct {
        unsigned int normal_or_forced_mode : 1; // 0 : normal, 1 : forced 
        unsigned int temp_measurement : 3;
        unsigned int press_measurement : 3;
        unsigned int iir_filter : 3;
        unsigned int standby_time : 3;
    };
    unsigned short all_flags;
} Bmp280_config_t;

void init_bmp280(Bmp280_config_t * cfg);
void request_measurements(void);
BMP280_S32_t bmp280_get_temperature(void);
BMP280_U32_t bmp280_get_pressure(void);
void bmp280_reset(void);
void bmp280_sleep(void);
void bmp280_normal(void);


#endif /* BMP280_DRIVER_HEADER_H */
