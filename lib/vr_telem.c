
// Contents of this file are copyright Andrew Pullin, 2013

//or_telem.c , OctoRoACH specific telemetry packet format




#include <xc.h>
#include "vr_telem.h"
#include "ams-enc.h"
#include "mpu6000.h"
#include "adc_pid.h"
#include "tih.h"
#include "pid-ip2.5.h"

// TODO (apullin) : Remove externs by adding getters to other modules
//extern pidObj motor_pidObjs[NUM_MOTOR_PIDS];
//extern int bemf[NUM_MOTOR_PIDS];

//externs added back in for VR telem porting (pullin 10/9/14)
extern int bemf[NUM_PIDS];
extern int bemfextra[2];
extern pidPos pidObjs[NUM_PIDS];
extern piWinch piObjs[NUM_PI_NO_AMS];
extern int ADXL377[3];

//void vrTelemGetData(unsigned char* ptr) {
void vrTelemGetData(vrTelemStruct_t* ptr) {
    
    //vrTelemStruct_t* tptr;
    //tptr = (vrTelemStruct_t*) ptr;

    int gdata[3];   //gyrodata
    int xldata[3];  // accelerometer data
    /////// Get XL data
    mpuGetGyro(gdata);
    mpuGetXl(xldata);

    //Motion control
    ptr->posL = pidObjs[0].p_state;
    ptr->posR = pidObjs[1].p_state;
    ptr->composL = pidObjs[0].p_input + pidObjs[0].interpolate;
    ptr->composR = pidObjs[1].p_input + pidObjs[1].interpolate;
    ptr->dcL = -(pidObjs[0].pwmDes < 0)*(PDC1) +(pidObjs[0].pwmDes > 0)*(PDC1); // left
    ptr->dcR = -(pidObjs[1].pwmDes < 0)*(PDC2) +(pidObjs[1].pwmDes > 0)*(PDC2); // right
    ptr->dcC = -(piObjs[0].pwmDes < 0)*(PDC3) +(piObjs[0].pwmDes > 0)*(PDC3);//stopgap measure to help Gwangpil
    ptr->dcD = PDC4;
    ptr->bemfL = bemf[0];
    ptr->bemfR = bemf[1];
    ptr->bemfC = bemfextra[0];
    ptr->bemfD = bemfextra[1];

    //gyro and XL
    ptr->gyroX = gdata[0];
    ptr->gyroY = gdata[1];
    ptr->gyroZ = gdata[2];
    ptr->accelX = xldata[0];
    ptr->accelY = xldata[1];
    ptr->accelZ = xldata[2];

    //ADXL377 channels
    ptr->ADXL377X = ADXL377[0];
    ptr->ADXL377Y = ADXL377[1];
    ptr->ADXL377Z = ADXL377[2];

    //Battery
    ptr->Vbatt = (int) adcGetVbatt();
}

//This may be unneccesary, since the telemtry type isn't totally anonymous

unsigned int orTelemGetSize() {
    return sizeof (vrTelemStruct_t);
}