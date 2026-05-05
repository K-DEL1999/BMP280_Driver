#include "BMP280_I2C_Driver_Header.h"

static Bmp280_config_t cfg; 
int i = 0;

void setup() {
    cfg = { 
        {
        .normal_or_forced_mode = 0x00,
        .temp_measurement = 0x01,
        .press_measurement = 0x01, 
        .iir_filter = 0x01,
        .standby_time = 0x01
        }
    };
    
    init_bmp280(&cfg);

}

void loop() {
    request_measurements();
    
    char buffer[64];
    sprintf(buffer, "temp: %ld C \tpress: %lu hPa\n", bmp280_get_temperature(), bmp280_get_pressure());
    Serial.print(buffer);
    
    if (!cfg.normal_or_forced_mode){
        (i++ % 2) ? bmp280_sleep() : bmp280_normal();
    }
    delay(1000);
}


