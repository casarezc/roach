
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
extern pidPos pidObjs[NUM_PIDS];

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
    ptr->posL1 = pidObjs[0].p_state;
    ptr->posR1 = pidObjs[1].p_state;
    ptr->composL1 = pidObjs[0].p_input + pidObjs[0].interpolate;
    ptr->composR1 = pidObjs[1].p_input + pidObjs[1].interpolate;
    ptr->posL2 = pidObjs[2].p_state;
    ptr->posR2 = pidObjs[3].p_state;
    ptr->composL2 = pidObjs[2].p_input + pidObjs[2].interpolate;
    ptr->composR2 = pidObjs[3].p_input + pidObjs[3].interpolate;
    ptr->dcL1 = pidObjs[0].output; // left 1
    ptr->dcR1 = pidObjs[1].output; // right 1
    ptr->dcL2 = pidObjs[2].output; // left 2
    ptr->dcR2 = pidObjs[3].output; // right 2
    ptr->bemfL1 = bemf[0];
    ptr->bemfR1 = bemf[1];
    ptr->bemfL2 = bemf[2];
    ptr->bemfR2 = bemf[3];

    //gyro and XL
    ptr->gyroX = gdata[0];
    ptr->gyroY = gdata[1];
    ptr->gyroZ = gdata[2];
    ptr->accelX = xldata[0];
    ptr->accelY = xldata[1];
    ptr->accelZ = xldata[2];

    //Battery
    ptr->Vbatt = (int) adcGetVbatt();
}

//This may be unneccesary, since the telemtry type isn't totally anonymous

unsigned int orTelemGetSize() {
    return sizeof (vrTelemStruct_t);
}