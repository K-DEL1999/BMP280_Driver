#include "BMP280_I2C_Driver_Header.h"

void setup() {
    Bmp280_config_t cfg = { 
        {
        .normal_or_forced_mode = 0,
        .temp_measurement = 0x01,
        .press_measurement = 0x01, 
        .iir_filter = 0x01,
        .standby_time = 0x01
        }
    };
    
    init_bmp280(&cfg);
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    
    request_measurements();

    printf("temp: %ld\tpress: %lud\n", bmp280_get_temperature(), bmp280_get_pressure());
}
