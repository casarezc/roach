//vr_telem.h , VelociRoACH specific telemetry packet format header

#include <stdint.h>

// Data structure type

typedef struct {
    int32_t posL1; // Hall angle position
    int32_t posR1;
    int32_t composL1; // Commanded Hall angle position
    int32_t composR1;
    int32_t yaw_input;
    int16_t dcL1; // PWM duty cycle
    int16_t dcR1;
    int16_t dcL2;
    int16_t dcR2;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t bemfL1;
    int16_t bemfR1;
    int16_t bemfL2;
    int16_t bemfR2;
    int16_t Vbatt; // battery voltage
} vrTelemStruct_t;

//void vrTelemGetData(unsigned char* ptr);
void vrTelemGetData(vrTelemStruct_t* ptr);

unsigned int vrTelemGetSize();