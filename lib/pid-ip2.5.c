/*
 * Name: UpdatePID.c
 * Desc: Control code to compute the new input to the plant
 * Date: 2009-04-03
 * Author: AMH
   modified to include hall effect sensor by RSF.
 * modified Dec. 2011 to include telemetry capture
 * modified Jan. 2012 to include median filter on back emf
 * modified Jan. 2013 to include AMS Hall encoder, and MPU 6000 gyro
 */
#include <xc.h>
#include "timer.h"
#include "settings.h"
#include "pid-ip2.5.h"
#include "dfmem.h"
#include "adc_pid.h"
#include "pwm.h"
#include "led.h"
#include "adc.h"
#include "sclock.h"
#include "ams-enc.h"
#include "tih.h"
#include "mpu6000.h"
#include "uart_driver.h"
#include "battery.h"
#include "ppool.h"
#include "dfmem.h"
#include "telem.h"

#include <stdlib.h> // for malloc
#include "init.h"  // for Timer1


#define MC_CHANNEL_PWM1     1
#define MC_CHANNEL_PWM2     2
#define MC_CHANNEL_PWM3     3
#define MC_CHANNEL_PWM4     4

//#define HALFTHROT 10000
#define HALFTHROT 2000
#define FULLTHROT 2*HALFTHROT
// MAXTHROT has to allow enough time at end of PWM for back emf measurement
// was 3976
#define MAXTHROT 3800

#define ABS(my_val) ((my_val) < 0) ? -(my_val) : (my_val)

// PID control structure
pidPos pidObjs[NUM_PIDS];

// Tail PID control structure
pidTail tailObjs;

// Steering PID control structure
strCtrl pidSteer;

// Body pose structure
poseEstimateStruct bodyPose;

// structure for reference velocity for leg
pidVelLUT pidVel[NUM_PIDS*NUM_BUFF];
pidVelLUT* activePID[NUM_PIDS]; //Pointer arrays for stride buffering
pidVelLUT* nextPID[NUM_PIDS];

#define T1_MAX 0xffffff  // max before rollover of 1 ms counter
// may be glitch in longer missions at rollover
volatile unsigned long t1_ticks;
unsigned long lastMoveTime;
unsigned long lastMoveTimeT;
int seqIndex;

//for battery voltage:
char calib_flag = 0;   // flag is set if doing calibration
long offsetAccumulatorPID[NUM_PIDS];
long offsetAccumulatorTail;
unsigned int offsetAccumulatorCounter;

// 2 last readings for median filter
int measLast1[NUM_PIDS];
int measLast2[NUM_PIDS];

int measTLast1;
int measTLast2;

// Declaration of bemf variables
int bemf[NUM_PIDS];
int bemfTail;

// Righting calculation variables
long pitch_mod;
long roll_mod;

// Euler angle initialization accumulator variables
long angle_acc[2] = { 0 };
long goffset_acc[3] = { 0 };
long AX;
long I;
long Q;

// Pose reset variables
char poseResetFlag = 0;
int poseResetCounter = 0;

// Repeat wait variables
char repeatWaitFlag = 0;
char xlzTriggerFlag = 0;
int repeatWaitCounter = 0;

// Temp variable to store p_input, v_input
long tail_p_input_temp;
int tail_v_input_temp;

// Constants for steering control
char first_DD_NoTI = 1; // If high, indicates the first instance of differential drive control for which tail impact is not triggered (only checked if switch_tail_impact is high)
char first_TI = 1;      // If high, indicates the first instance of tail impact control being triggered (bang-bang)
char first_TD = 1;      // If high, indicates the first instance of tail drag control being executed

int counter_TD = 0;     // Counter used for duty cycle modulation of tail drag contact
char tail_contact_flag = 0;  // Boolean indicating tail contact with the ground during tail drag


// ==== Initialization =======================================================================================
// ===========================================================================================================
void pidSetup()
{
	int i;

        // Initialize pidObjs for leg control and tailObjs for tail control
	for(i = 0; i < NUM_PIDS; i++){
		initPIDObjPos( &(pidObjs[i]), DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_KAW, DEFAULT_FF); 
	}

        initTailObj( &(tailObjs), DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_KAW, DEFAULT_FF);

        initBodyPose( &(bodyPose) );

        initStrCtrl( &(pidSteer), DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_KAW);

        //Initialize fields in pidObjs for gait control, as well as pidVelLUT structures
	initPIDVelProfile();
	SetupTimer1();  // main interrupt used for leg motor PID

	lastMoveTime = 0;
        lastMoveTimeT = 0;

        //Configure channels and motor/control directions for each control object
        pidObjs[LEFT_LEGS_PID_NUM].output_channel  = LEFT_LEGS_TIH_CHAN;
        pidObjs[LEFT_LEGS_PID_NUM].p_state_flip    = LEFT_LEGS_ENC_FLIP;
        pidObjs[LEFT_LEGS_PID_NUM].encoder_num     = LEFT_LEGS_ENC_NUM;
        pidObjs[LEFT_LEGS_PID_NUM].pwm_flip        = LEFT_LEGS_PWM_FLIP;

        pidObjs[RIGHT_LEGS_PID_NUM].output_channel = RIGHT_LEGS_TIH_CHAN;
        pidObjs[RIGHT_LEGS_PID_NUM].p_state_flip   = RIGHT_LEGS_ENC_FLIP;
        pidObjs[RIGHT_LEGS_PID_NUM].encoder_num    = RIGHT_LEGS_ENC_NUM;
        pidObjs[RIGHT_LEGS_PID_NUM].pwm_flip       = RIGHT_LEGS_PWM_FLIP;

        tailObjs.output_channel = TAIL_TIH_CHAN;
        tailObjs.p_state_flip   = TAIL_ENC_FLIP;
        tailObjs.encoder_num    = TAIL_ENC_NUM;
        tailObjs.pwm_flip       = TAIL_PWM_FLIP;

        // Initialize PID structures before starting Timer1
        pidSetInput(LEFT_LEGS_PID_NUM, 0);
        pidSetInput(RIGHT_LEGS_PID_NUM, 0);

        // Initialize input of tail PID structure
        tailSetPInput(0);

	
	EnableIntT1; // turn on pid interrupts

	calibBatteryOffset(100); //???This is broken for 2.5
}

/*****************************************************************************************/
/*****************************************************************************************/
/*********************** Stop Motor and Interrupts *********************************************/
/*****************************************************************************************/

/*****************************************************************************************/
void EmergencyStop(void)
{
    int j;
    for (j = 0; j < NUM_PIDS; j++) {
        pidSetInput(j, 0);
    }
    tailSetPInput(0);

    DisableIntT1; // turn off pid interrupts
    SetDCMCPWM(MC_CHANNEL_PWM1, 0, 0);    // set PWM to zero
    SetDCMCPWM(MC_CHANNEL_PWM2, 0, 0);
    SetDCMCPWM(MC_CHANNEL_PWM3, 0, 0);
    SetDCMCPWM(MC_CHANNEL_PWM4, 0, 0);
}

// calibrate A/D offset, using PWM synchronized A/D reads inside
// timer 1 interrupt loop
// BATTERY CHANGED FOR IP2.5 ***** need to fix

void calibBatteryOffset(int spindown_ms) {
    long temp; // could be + or -
    unsigned int battery_voltage;
    int j;
    // save current PWM config
    int tempPDC1 = PDC1;
    int tempPDC2 = PDC2;
    int tempPDC3 = PDC3;
    int tempPDC4 = PDC4;
    PDC1 = 0;
    PDC2 = 0;
    PDC3 = 0;
    PDC4 = 0; /* SFR for PWM? */

    // save current PID status, and turn off PID control
    short tempPidObjsOnOff[NUM_PIDS];
    for (j = 0; j < NUM_PIDS; j++) {
        tempPidObjsOnOff[j] = pidObjs[j].onoff;
        pidObjs[j].onoff = PID_OFF;
    }
    // save current tail PID status, and turn off tail PID control
    short tempTailObjsOnOff;
    tempTailObjsOnOff = tailObjs.onoff;
    tailObjs.onoff = PID_OFF;

    delay_ms(spindown_ms); //motor spin-down
    LED_RED = 1;

    for (j = 0; j < NUM_PIDS; j++) {
        offsetAccumulatorPID[j] = 0;
    }
    offsetAccumulatorTail = 0;

    offsetAccumulatorCounter = 0; // updated inside servo loop

    calib_flag = 1; // enable calibration
    while (offsetAccumulatorCounter < 100); // wait for 100 samples
    calib_flag = 0; // turn off calibration

    battery_voltage = adcGetVbatt();

    //Cycle through pid indices and set offsets
    for (j = 0; j < NUM_PIDS; j++) {
        temp = offsetAccumulatorPID[j];
        temp = temp / (long) offsetAccumulatorCounter;
        pidObjs[j].inputOffset = (int) temp;
    }
    temp = offsetAccumulatorTail;
    temp = temp / (long) offsetAccumulatorCounter;
    tailObjs.inputOffset = (int) temp;

    LED_RED = 0;
    // restore PID values
    PDC1 = tempPDC1;
    PDC2 = tempPDC2;
    PDC3 = tempPDC3;
    PDC4 = tempPDC4;

    for (j = 0; j < NUM_PIDS; j++) {
        pidObjs[j].onoff = tempPidObjsOnOff[j];
    }
    tailObjs.onoff = tempTailObjsOnOff;
}


// -------------------------   control  loop section  -------------------------------

/*********************** Motor Control Interrupt *********************************************/
/*****************************************************************************************/
/*****************************************************************************************/

/* update setpoint  only leg which has run_time + start_time > t1_ticks */
/* turn off when all PIDs have finished */
static volatile unsigned char interrupt_count = 0;
static volatile unsigned char telemetry_count = 0;
extern volatile MacPacket uart_tx_packet;
extern volatile unsigned char uart_tx_flag;

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    int i;
    int j;
    LED_3 = 1;
    interrupt_count++;

    //Telemetry save, at 1Khz
    //TODO: Break coupling between PID module and telemetry triggering
    if (interrupt_count == 3) {
        telemSaveNow();
    }
    //Update IMU and encoder
    //TODO: Break coupling between PID module and IMU update
    if (interrupt_count == 4) {
        mpuBeginUpdate();
        if((tailObjs.mode==TAIL_MODE_RIGHTING)||(pidSteer.onoff == PID_ON)){
            computeEulerAngles();
        }
        amsEncoderStartAsyncRead();
    }        //PID controller update
    else if (interrupt_count == 5) {
        interrupt_count = 0;

        if (t1_ticks == T1_MAX) t1_ticks = 0;
        t1_ticks++;
        pidGetState(); // always update pid leg state, even if motor is coasting
        for (j = 0; j < NUM_PIDS; j++) {
            // only update tracking setpoint if time has not yet expired
            if (pidObjs[j].onoff) {
                if (pidObjs[j].timeFlag) {
                    if (pidObjs[j].start_time + pidObjs[j].run_time >= t1_ticks) {
                        if((pidSteer.onoff == PID_OFF)||(pidSteer.mode == STEER_MODE_TAILDRAG)){
                            pidGetSetpoint(j);
                        }
                    }
                    if (t1_ticks > lastMoveTime) { // turn off if done running all legs
                        for (i = 0; i < NUM_PIDS; i++) {
                            pidObjs[i].onoff = 0;
                        }
                        repeatWaitFlag = 1;
                    }
                }
                else {
                    if((pidSteer.onoff == PID_OFF)||(pidSteer.mode == STEER_MODE_TAILDRAG)){
                        pidGetSetpoint(j);
                    }
                }
            }
        }
        
        // Set steering control before leg and tail PID controllers because it will affect setpoints
        strCtrlSetControl();
        pidSetControl();

        tailGetState(); // always update tail leg state, even if motor is coasting
        if (tailObjs.onoff) {
            if (tailObjs.timeFlag) {
                if (t1_ticks > lastMoveTimeT) { // turn off if done running all legs
                    tailObjs.onoff = 0;
                }
            }
        }
        tailSetControl(); // set tail control from on/off state and mode


//        if(pidObjs[0].onoff) {
            //telemGetPID();
//            telemSaveNow();
            //TODO: Telemetry save should not be tied to the on/off state of the PID controller. Removed for now. needs to be checked. (ronf, pullin, dhaldane)

            // uart_tx_packet = ppoolRequestFullPacket(sizeof(telemStruct_t));
            // if(uart_tx_packet != NULL) {
            //     //time|Left pstate|Right pstate|Commanded Left pstate| Commanded Right pstate|DCR|DCL|RBEMF|LBEMF|Gyrox|Gyroy|Gyroz|Ax|Ay|Az
            //     //bytes: 4,4,4,4,4,2,2,2,2,2,2,2,2,2,2
            //     paySetType(uart_tx_packet->payload, CMD_PID_TELEMETRY);
            //     paySetStatus(uart_tx_packet->payload, 0);
            //     paySetData(uart_tx_packet->payload, sizeof(telemStruct_t), (unsigned char *) &telemPIDdata);
            //     uart_tx_flag = 1;
//        }
    }
    LED_3 = 0;
    _T1IF = 0;
}

// ==== Leg PID Commands =======================================================================================
// =============================================================================================================

//Returns pointer to non-active buffer

pidVelLUT* otherBuff(pidVelLUT* array, pidVelLUT* ptr) {
    if (ptr >= &(array[NUM_PIDS])) {
        return ptr - NUM_PIDS;
    } else {
        return ptr + NUM_PIDS;
    }
}

// ----------   all the initializations  -------------------------
// set expire time for first segment in pidSetInput - use start time from MoveClosedLoop
// set points and velocities for one revolution of leg
// called from pidSetup()

void initPIDVelProfile() {
    int i, j;
    pidVelLUT* tempPID;
    for (j = 0; j < NUM_PIDS; j++) {
        pidObjs[j].index = 0; // point to first velocity
        pidObjs[j].interpolate = 0;
        pidObjs[j].leg_stride = 0; // set initial leg count
        activePID[j] = &(pidVel[j]); //Initialize buffer pointers
        nextPID[j] = NULL;
        tempPID = otherBuff(pidVel, activePID[j]);
        for (i = 0; i < NUM_VELS; i++) {
            tempPID->interval[i] = 100;
            tempPID->delta[i] = 0;
            tempPID->vel[i] = 0;
        }
        tempPID->onceFlag = 0;
        nextPID[j] = tempPID;
        pidObjs[j].p_input = 0; // initialize first set point 
        pidObjs[j].v_input = (int) (((long) pidVel[j].vel[0] * K_EMF) >> 8); //initialize first velocity, scaled
    }
}


// called from cmd.c

void setPIDVelProfile(int pid_num, int *interval, int *delta, int *vel, int onceFlag) {
    // Set exact repeating intrastride profile for use with all leg control except for differential drive steering
    pidVelLUT* tempPID;
    int i;
    int interval_net = 0;
    long avg_vel_temp;

    tempPID = otherBuff(pidVel, activePID[pid_num]);
    for (i = 0; i < NUM_VELS; i++) {
        tempPID->interval[i] = interval[i];
        tempPID->delta[i] = delta[i];
        tempPID->vel[i] = vel[i];
        interval_net += interval[i];
    }
    tempPID->onceFlag = onceFlag;
    if (activePID[pid_num]->onceFlag == 0) {
        nextPID[pid_num] = tempPID;
    }

    avg_vel_temp = LEGS_FULL_REV/( (long) interval_net);
    pidObjs[pid_num].avg_vel = (int) avg_vel_temp;
}

// called from pidSetup()

void initPIDObjPos(pidPos *pid, int Kp, int Ki, int Kd, int Kaw, int ff) {
    pid->p_input = 0;
    pid->v_input = 0;
    pid->p = 0;
    pid->i = 0;
    pid->d = 0;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Kaw = Kaw; 
    pid->feedforward = ff;
    pid->output = 0;
    pid->onoff = 0;
    pid->p_error = 0;
    pid->v_error = 0;
    pid->i_error = 0;

    pid->p_state_flip = 0; //default to no flip
    pid->output_channel = 0;
}

// called from set thrust closed loop, etc. Thrust

void pidSetInput(int pid_num, int input_val) {
    unsigned long temp;
    temp = t1_ticks; // changed this to atomic read t1_ticks since t1_ticks is no longer reset to 0 in pidOn
    /*      ******   use velocity setpoint + throttle for compatibility between Hall and Pullin code *****/
    /* otherwise, miss first velocity set point */
    pidObjs[pid_num].v_input = input_val + (int) (((long) pidVel[pid_num].vel[0] * K_EMF) >> 8); //initialize first velocity ;
    pidObjs[pid_num].start_time = temp;
    //zero out running PID values
    pidObjs[pid_num].i_error = 0;
    pidObjs[pid_num].p = 0;
    pidObjs[pid_num].i = 0;
    pidObjs[pid_num].d = 0;
    //Seed the median filter
    measLast1[pid_num] = input_val;
    measLast2[pid_num] = input_val;

    // set initial time for next move set point
    /*   need to set index =0 initial values */
    /* position setpoints start at 0 (index=0), then interpolate until setpoint 1 (index =1), etc */
    pidObjs[pid_num].expire = temp + (long) pidVel[pid_num].interval[0]; // end of first interval 
    pidObjs[pid_num].interpolate = 0;
    /*	pidObjs[pid_num].p_input += pidVel[pid_num].delta[0];	//update to first set point
     ***  this should be set only after first .expire time to avoid initial transients */
    pidObjs[pid_num].index = 0; // reset setpoint index
    // set first move at t = 0
    //	pidVel[0].expire = temp;   // right side
    //	pidVel[1].expire = temp;   // left side

}

void pidStartTimedTrial(unsigned int run_time){
    unsigned long temp;
    int j;

    temp = t1_ticks; // need atomic read due to interrupt

    for (j = 0; j < NUM_PIDS; j++) {
        pidObjs[j].run_time = run_time;
        pidObjs[j].start_time = temp;
    }

    if ((temp + (unsigned long) run_time) > lastMoveTime){
        lastMoveTime = temp + (unsigned long) run_time;
    }  // set run time to max requested time
}

// from cmd.c  PID set gains

void pidSetGains(int pid_num, int Kp, int Ki, int Kd, int Kaw, int ff) {
    pidObjs[pid_num].Kp = Kp;
    pidObjs[pid_num].Ki = Ki;
    pidObjs[pid_num].Kd = Kd;
    pidObjs[pid_num].Kaw = Kaw;
    pidObjs[pid_num].feedforward = ff;
}

void pidOn(int pid_num) {
    pidObjs[pid_num].onoff = PID_ON;
//    t1_ticks = 0;
}

void pidOff(int pid_num) {
    pidObjs[pid_num].onoff = PID_OFF;
//    t1_ticks = 0;
}

// zero position setpoint for both motors (avoids big offset errors)

void pidZeroPos(int pid_num) {
    // disable interrupts to reset state variables
    DisableIntT1; // turn off pid interrupts
    amsEncoderResetPos(pidObjs[pid_num].encoder_num); //  reinitialize rev count and relative zero encoder position for the specified encoder
    pidObjs[pid_num].p_state = 0;
    // reset position setpoint as well
    pidObjs[pid_num].p_input = 0;
    pidObjs[pid_num].v_input = 0;
    pidObjs[pid_num].leg_stride = 0; // strides also reset
    EnableIntT1; // turn on pid interrupts
}

// update desired velocity and position tracking setpoints for each leg

void pidGetSetpoint(int j) {
    int index;
    index = pidObjs[j].index;
    // update desired position between setpoints, scaled by 256
    pidObjs[j].interpolate += (long) activePID[j]->vel[index];

    if (t1_ticks >= pidObjs[j].expire) { // time to reach previous setpoint has passed
        pidObjs[j].interpolate = 0;
        pidObjs[j].p_input += activePID[j]->delta[index]; //update to next set point
        pidObjs[j].index++;

        if (pidObjs[j].index >= NUM_VELS) {
            pidObjs[j].index = 0;
            pidObjs[j].leg_stride++; // one full leg revolution
            /**** maybe need to handle round off in position set point ***/
            checkSwapBuff(j);
        }
        pidObjs[j].expire += activePID[j]->interval[pidObjs[j].index]; // expire time for next interval
        pidObjs[j].v_input = (activePID[j]->vel[pidObjs[j].index]); //update to next velocity
    }
}

void checkSwapBuff(int j){
    if(nextPID[j] != NULL){    //Swap pointer if not null
       if(nextPID[j]->onceFlag == 1){
           pidVelLUT* tempPID;
           CRITICAL_SECTION_START;
           tempPID = activePID[j];
           activePID[j] = nextPID[j];
           nextPID[j] = tempPID;
           CRITICAL_SECTION_END;
       } else {
           CRITICAL_SECTION_START;
           activePID[j] = nextPID[j];
           nextPID[j] = NULL;
           CRITICAL_SECTION_END;
       }
    }
}

/* update state variables including motor position and velocity */

void pidGetState() {
    int i;
    long p_state;
    int enc_num;
    int encPosition, encOticks;
    unsigned int encOffset;

    unsigned long time_start, time_end;
    //	calib_flag = 0;  //BEMF disable
    // get diff amp offset with motor off at startup time
    if (calib_flag) {
        for (i = 0; i < NUM_PIDS; i++) {
            offsetAccumulatorPID[i] += adcGetMotor(pidObjs[i].output_channel);
        }
        offsetAccumulatorCounter++;
    }

    // choose velocity estimate
#if VEL_BEMF == 0    // use first difference on position for velocity estimate
    long oldpos[NUM_PIDS], velocity;
    for (i = 0; i < NUM_PIDS; i++) {
        oldpos[i] = pidObjs[i].p_state;
    }
#endif

    time_start = sclockGetTime();
    // only works to +-32K revs- might reset after certain number of steps? Should wrap around properly
    for (i = 0; i < NUM_PIDS; i++) {
        bemf[i] = pidObjs[i].inputOffset - adcGetMotor(pidObjs[i].output_channel); // watch sign for A/D? unsigned int -> signed?

        enc_num = pidObjs[i].encoder_num;

        encPosition = amsEncoderGetPos(enc_num);
        encOticks = amsEncoderGetOticks(enc_num);
        encOffset = amsEncoderGetOffset(enc_num);

        p_state =  (long)encPosition << 2; // pos 14 bits 0x0 -> 0x3fff
        p_state = p_state - ((long)encOffset << 2); // subtract offset to get zero position
        p_state = p_state + ((long)encOticks << 16);

        pidObjs[i].p_state = p_state;

        if(pidObjs[i].p_state_flip){
            pidObjs[i].p_state = -pidObjs[i].p_state;
        }

    }

    time_end = sclockGetTime() - time_start;


#if VEL_BEMF == 0    // use first difference on position for velocity estimate
    for (i = 0; i < NUM_PIDS; i++) {
        velocity = pidObjs[i].p_state - oldpos[i]; // Encoder ticks per ms
        if (velocity > 0x7fff) velocity = 0x7fff; // saturate to half cycle
        if (velocity < -0x7fff) velocity = -0x7fff;
        pidObjs[i].v_state = (int) velocity;
    }
#endif

    // choose velocity estimate

#if VEL_BEMF == 1
    int measurements[NUM_PIDS];
    // Battery: AN0, MotorA AN8, MotorB AN9, MotorC AN10, MotorD AN11
    for (i = 0; i < NUM_PIDS; i++) {
        measurements[i] = bemf[i]; // watch sign for A/D? unsigned int -> signed?


        //Get motor speed reading on every interrupt - A/D conversion triggered by PWM timer to read Vm when transistor is off
        // when motor is loaded, sometimes see motor short so that  bemf=offset voltage
        // get zero sometimes - open circuit brush? Hence try median filter
        if (measurements[i] > measLast1[i]) {
            if (measLast1[i] > measLast2[i]) {
                bemf[i] = measLast1[i];
            }// middle value is median
            else // middle is smallest
            {
                if (measurements[i] > measLast2[i]) {
                    bemf[i] = measLast2[i];
                }// 3rd value is median
                else {
                    bemf[i] = measurements[i];
                } // first value is median
            }
        }
        else // first is not biggest
        {
            if (measLast1[i] < measLast2[i]) {
                bemf[i] = measLast1[i];
            }// middle value is median
            else // middle is biggest
            {
                if (measurements[i] < measLast2[i]) {
                    bemf[i] = measLast2[i];
                }// 3rd value is median
                else {
                    bemf[i] = measurements[i]; // first value is median
                }
            }
        }
    // store old values
    measLast2[i] = measLast1[i];
    measLast1[i] = measurements[i];
    pidObjs[i].v_state = bemf[i];
    }
    //if((measurements[0] > 0) || (measurements[1] > 0)) {
    if ((measurements[0] > 0)) {
        LED_BLUE = 1;
    } else {
        LED_BLUE = 0;
    }
#endif
}

void pidSetControl() {
    int j;
    long p_mod_error;
    // 0 = right side
    for (j = 0; j < NUM_PIDS; j++) { //pidobjs[0] : right side
        // p_input has scaled velocity interpolation to make smoother
        // p_state is [16].[16]
        if ((pidObjs[j].mode == PID_MODE_CONTROLLED) && (pidObjs[j].onoff == PID_ON)) {
            pidObjs[j].p_error = pidObjs[j].p_input + pidObjs[j].interpolate - pidObjs[j].p_state;

            p_mod_error = (pidObjs[j].p_error & 0x0000ffff); // Clobber the MSBs corresponding to full revolutions

            if (p_mod_error < ERR_FWD_BOUND) {
                pidObjs[j].p_error = p_mod_error; //Take the mod 2 pi error if you're behind in the cycle
            } else if (p_mod_error > ERR_BWD_BOUND) {
                pidObjs[j].p_error = p_mod_error - 65535; //Take the mod 2 pi error shifted down by 2 pi if you're ahead in the cycle
            } else {
                pidObjs[j].p_error = 0; // Wait for reference if you're too far away from it
            }

            pidObjs[j].v_error = pidObjs[j].v_input - pidObjs[j].v_state; // v_input should be revs/sec
            //Update values
            UpdatePID(&(pidObjs[j]));

            if (pidObjs[j].pwm_flip) {
                tiHSetDC(pidObjs[j].output_channel, -pidObjs[j].output);
            } else {
                tiHSetDC(pidObjs[j].output_channel, pidObjs[j].output);
            }
        } else if ((pidObjs[j].mode == PID_MODE_PWMPASS) && (pidObjs[j].onoff == PID_ON)) {
            tiHSetDC(pidObjs[j].output_channel, pidObjs[j].pwmDes);
        } else {
            pidObjs[j].output = 0;
            tiHSetDC(pidObjs[j].output_channel, 0);
        }
    }// end of for(j)

}

void UpdatePID(pidPos *pid) {
    pid->p = ((long) pid->Kp * pid->p_error) >> 12; // scale so doesn't over flow
    pid->i = (long) pid->Ki * pid->i_error >> 12;
    pid->d = (long) pid->Kd * (long) pid->v_error;
    // better check scale factors

    pid->preSat = pid->feedforward + pid->p +
            ((pid->i) >> 4) + // divide by 16
            (pid->d >> 4); // divide by 16
    pid->output = pid->preSat;

    /* i_error say up to 1 rev error 0x10000, X 256 ms would be 0x1 00 00 00
        scale p_error by 16, so get 12 bit angle value*/
    pid-> i_error = (long) pid-> i_error + ((long) pid->p_error >> 4); // integrate error
    // saturate output - assume only worry about >0 for now
    // apply anti-windup to integrator
    if (pid->preSat > MAXTHROT) {
        pid->output = MAXTHROT;
        pid->i_error = (long) pid->i_error +
                (long) (pid->Kaw) * ((long) (MAXTHROT) - (long) (pid->preSat))
                / ((long) GAIN_SCALER);
    }
    if (pid->preSat < -MAXTHROT) {
        pid->output = -MAXTHROT;
        pid->i_error = (long) pid->i_error +
                (long) (pid->Kaw) * ((long) (-MAXTHROT) - (long) (pid->preSat))
                / ((long) GAIN_SCALER);
    }
}

long pidGetPState(unsigned int channel) {
    if (channel < NUM_PIDS) {
        return pidObjs[channel].p_state;
    } else {
        return 0;
    }
}

void pidSetPInput(unsigned int channel, long p_input) {
    if (channel < NUM_PIDS) {
        pidObjs[channel].p_input = p_input + P_OFFSET_INTRASTRIDE;
    }
}


//TODO: Controller design, this function was created specifically to remove existing externs.

void pidStartMotor(unsigned int channel) {
    if (channel < NUM_PIDS) {
        pidObjs[channel].timeFlag = 0;
        pidSetInput(channel, 0);
        pidObjs[channel].p_input = pidObjs[channel].p_state;
        pidOn(channel);
    }

}

//TODO: Controller design, this function was created specifically to remove existing externs.

void pidSetTimeFlag(unsigned int channel, char val) {
    if (channel < NUM_PIDS) {
        pidObjs[channel].timeFlag = val;
    }
}

//TODO: Controller design, this function was created specifically to remove existing externs.

void pidSetMode(unsigned int channel, char mode) {
    if (channel < NUM_PIDS) {
        pidObjs[channel].mode = mode;
    }
}

void pidSetPWMDes(unsigned int channel, int pwm) {
    if (channel < NUM_PIDS) {
        pidObjs[channel].pwmDes = pwm;
    }
}

// ==== Tail Commands =======================================================================================
// ==========================================================================================================

void initTailObj(pidTail *pid, int Kp, int Ki, int Kd, int Kaw, int ff) {
    pid->p_input = 0;
    pid->v_input = 0;
    pid->p = 0;
    pid->i = 0;
    pid->d = 0;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Kaw = Kaw;
    pid->feedforward = ff;
    pid->output = 0;
    pid->onoff = 0;
    pid->p_error = 0;
    pid->v_error = 0;
    pid->i_error = 0;

    pid->rev_count = 0;
    pid->zero_val = 0;
    pid->zero_state = 0;

    pid->p_state_flip = 0; //default to no flip
    pid->output_channel = 0;
}

// Used to set position reference for tail in control mode 0
// input_val is in units of degrees
void tailSetPInput(int input_val) {
    tailObjs.mode = TAIL_MODE_POSITION; //position control mode
    tailObjs.p_input = (((long) input_val)*TAIL_FULL_REV)/(TAIL_DEG_GR_SCALER); //constant position setpoint converted from degrees overflows at ~ 91 revolutions
    tailObjs.p_interpolate = 0;
    tailObjs.v_input = 0; //zero velocity setpoint
    tailObjs.start_time = t1_ticks;
    //zero out running PID values
    tailObjs.i_error = 0;
    tailObjs.p = 0;
    tailObjs.i = 0;
    tailObjs.d = 0;
    //Seed the median filter
    measTLast1 = 0;
    measTLast2 = 0;

}

// Used to set velocity reference for tail in control mode 1
// input_val is in units of output shaft counts per time interval (1 ms)
void tailSetVInput(int input_val) {
    tailObjs.mode = TAIL_MODE_VELOCITY; //velocity control mode
    tailObjs.p_input = tailObjs.p_state; //initialize position setpoint at current position
    tailObjs.p_interpolate = 0;
    tailObjs.v_input = input_val*TAIL_GEAR_RATIO; //velocity setpoint
    tailObjs.start_time = t1_ticks;
    //zero out running PID values
    tailObjs.i_error = 0;
    tailObjs.p = 0;
    tailObjs.i = 0;
    tailObjs.d = 0;
    //Seed the median filter
    measTLast1 = 0;
    measTLast2 = 0;

}

// Used to set position amplitude, swing period of autonomous righting control
// p_input is in units of output shaft counts, where 2^15 counts is 180 degrees of rotation
// swing_period is in units of milliseconds
void tailSetRightingInput(long p_input, int swing_period) {
    tailObjs.mode = TAIL_MODE_RIGHTING; //righting control mode
    tailObjs.p_input = p_input*TAIL_GEAR_RATIO; //constant position setpoint
    tailObjs.p_interpolate = 0;
    tailObjs.v_input = 0; //zero velocity setpoint
    tailObjs.start_time = t1_ticks;
    //zero out running PID values
    tailObjs.i_error = 0;
    tailObjs.p = 0;
    tailObjs.i = 0;
    tailObjs.d = 0;
    //Seed the median filter
    measTLast1 = 0;
    measTLast2 = 0;

    //Set swing period
    tailObjs.swing_period = swing_period;

}

// Used to set position amplitude, position bias, swing period of periodic tail motion
// p_amp, p_bias are in units of output shaft counts, where 2^15 counts is 180 degrees of rotation
// swing_period is in units of milliseconds
void tailSetPeriodicInput(int p_amp, int p_bias, int swing_period) {
    tailObjs.mode = TAIL_MODE_PERIODIC; //periodic control mode
    tailObjs.p_input = (long) (p_bias - p_amp); //first position setpoint
    tailObjs.p_input = tailObjs.p_input*2*TAIL_GEAR_RATIO; //scale up to correct units
    tailObjs.p_interpolate = 0;
    tailObjs.v_input = 0; //zero velocity setpoint
    tailObjs.start_time = t1_ticks;
    //zero out running PID values
    tailObjs.i_error = 0;
    tailObjs.p = 0;
    tailObjs.i = 0;
    tailObjs.d = 0;
    //Seed the median filter
    measTLast1 = 0;
    measTLast2 = 0;

    //Initialize swing sign and store amplitude, bias, period
    tailObjs.swing_sign = 0;
    tailObjs.p_amp = p_amp;
    tailObjs.p_bias = p_bias;
    tailObjs.swing_period = swing_period;

}

void tailStartTimedTrial(unsigned int run_time){
    unsigned long temp;

    temp = t1_ticks; // need atomic read due to interrupt

    tailObjs.run_time = run_time;
    tailObjs.start_time = temp;

    if ((temp + (unsigned long) run_time) > lastMoveTimeT){
        lastMoveTimeT = temp + (unsigned long) run_time;
    }  // set run time to max requested time
}

void tailSetGains(int Kp, int Ki, int Kd, int Kaw, int ff) {
    tailObjs.Kp = Kp;
    tailObjs.Ki = Ki;
    tailObjs.Kd = Kd;
    tailObjs.Kaw = Kaw;
    tailObjs.feedforward = ff;
}

void tailOn() {
    tailObjs.onoff = PID_ON;
//    t1_ticks = 0;
}

void tailOff() {
    tailObjs.onoff = PID_OFF;
//    t1_ticks = 0;
}

void tailZeroPos() {
    // disable interrupts to reset state variables
    DisableIntT1; // turn off pid interrupts
    amsEncoderResetPos(tailObjs.encoder_num); //  reinitialize rev count and zero position for tail encoder
    tailObjs.p_state = 0;
    // reset inputs as well
    tailObjs.p_input = 0;
    tailObjs.v_input = 0;

    tailObjs.zero_val = 0;
    tailObjs.rev_count = 0;
    EnableIntT1; // turn on pid interrupts
}

void tailGetState() {
    long p_state;
    int enc_num;
    int encPosition, encOticks;
    unsigned int encOffset;

    unsigned int tail_zero_input;


    unsigned long time_start, time_end;
    //	calib_flag = 0;  //BEMF disable
    // get diff amp offset with motor off at startup time
    if (calib_flag) {
        offsetAccumulatorTail += adcGetMotor(tailObjs.output_channel);
    }

    // choose velocity estimate
    #if VEL_BEMF == 0    // use first difference on position for velocity estimate
        long oldpos, velocity;
        oldpos = tailObjs.p_state;
    #endif

    time_start = sclockGetTime();
    // only works to +-32K revs- might reset after certain number of steps? Should wrap around properly
    bemfTail = tailObjs.inputOffset - adcGetMotor(tailObjs.output_channel); // watch sign for A/D? unsigned int -> signed?

    enc_num = tailObjs.encoder_num;

    encPosition = amsEncoderGetPos(enc_num);
    encOticks = amsEncoderGetOticks(enc_num);
    encOffset = amsEncoderGetOffset(enc_num);

    p_state =  (long)encPosition << 2; // pos 14 bits 0x0 -> 0x3fff
    p_state = p_state - ((long)encOffset << 2); // subtract offset to get zero position
    p_state = p_state + ((long)encOticks << 16);

    // Flip position state if p state flip is high
    if(tailObjs.p_state_flip){
        p_state = -p_state;
    }

    p_state = p_state + ((long) tailObjs.rev_count)*TAIL_FULL_REV*TAIL_GEAR_RATIO; // Add full revolutions stored after zeroing tail
    p_state = p_state + ((long) tailObjs.zero_val)*TAIL_GEAR_RATIO; // Add zeroed values to calibrated hall trigger

    tailObjs.p_state = p_state;

    time_end = sclockGetTime() - time_start;


#if VEL_BEMF == 0    // use first difference on position for velocity estimate
    velocity = tailObjs.p_state - oldpos; // Encoder ticks per ms
    if (velocity > 0x7fff) velocity = 0x7fff; // saturate to int -> the same as saturating to half revolution in 1 ms
    if (velocity < -0x7fff) velocity = -0x7fff;
    tailObjs.v_state = (int) velocity;
#endif

    // choose velocity estimate

#if VEL_BEMF == 1
    int measurements;
    // Battery: AN0, MotorA AN8, MotorB AN9, MotorC AN10, MotorD AN11
    measurements = bemfTail; // watch sign for A/D? unsigned int -> signed?

    //Get motor speed reading on every interrupt - A/D conversion triggered by PWM timer to read Vm when transistor is off
    // when motor is loaded, sometimes see motor short so that  bemf=offset voltage
    // get zero sometimes - open circuit brush? Hence try median filter
    if (measurements > measTLast1) {
        if (measTLast1 > measTLast2) {
            bemfTail = measTLast1;
        }// middle value is median
        else // middle is smallest
        {
            if (measurements > measTLast2) {
                bemfTail = measTLast2;
            }// 3rd value is median
            else {
                bemfTail = measurements;
            } // first value is median
        }
    }
    else // first is not biggest
    {
        if (measTLast1 < measTLast2) {
            bemfTail = measTLast1;
        }// middle value is median
        else // middle is biggest
        {
            if (measurements < measTLast2) {
                bemfTail = measTLast2;
            }// 3rd value is median
            else {
                bemfTail = measurements; // first value is median
            }
        }
    }
    // store old values
    measTLast2 = measTLast1;
    measTLast1 = measurements;
    tailObjs.v_state = bemfTail;

    if ((measurements[0] > 0)) {
        LED_BLUE = 1;
    } else {
        LED_BLUE = 0;
    }
#endif

    // Get tail zero input value
    tail_zero_input = (unsigned int) (TAIL_ZERO_INPUT==0);

    // Tail zeroing routine wait for at least a dozen consecutive high readings and then a low (falling edge detection) to zero tail
    tailObjs.zero_state = (tailObjs.zero_state << 1)|(tail_zero_input)|(0xe000);

    if (tailObjs.zero_state == 0xfffe){

        // Add a tolerance to capture almost full revolutions
        if (p_state > 0){
            p_state = p_state + TAIL_REV_TOL;
        }
        else{
            p_state = p_state - TAIL_REV_TOL;
        }


        // Set different zero points based on tail velocity
        if (tailObjs.v_state >= 0) {
            tailObjs.zero_val = TAIL_ZERO_POS;
        }
        else {
            tailObjs.zero_val = TAIL_ZERO_NEG;
        }

        // Store revolution counter
        p_state = p_state/(TAIL_FULL_REV*TAIL_GEAR_RATIO);
        tailObjs.rev_count = (int) p_state;

        // Toggle red LED to indicate zeroing event
        LED_RED = ~LED_RED;

        // Reset encoder accumulation
        amsEncoderResetPos(tailObjs.encoder_num); //  reinitialize rev count and zero position for tail encoder
    }


}

void tailSetControl() {

    long p_mod = 0;

    // control to position setpoint
    // p_state is [16].[16]
    if ((tailObjs.mode == TAIL_MODE_POSITION) && (tailObjs.onoff == PID_ON)) {
        tailObjs.p_error = tailObjs.p_input - tailObjs.p_state;
        tailObjs.v_error = tailObjs.v_input - tailObjs.v_state; // note that this will be executed when v_input=0

        //Update values
        UpdateTailPID(&(tailObjs));

        if (tailObjs.pwm_flip) {
            tiHSetDC(tailObjs.output_channel, -tailObjs.output);
        } else {
            tiHSetDC(tailObjs.output_channel, tailObjs.output);
        }
    }
    // control to velocity setpoint
    // update interpolate position setpoint based on constant velocity
    else if ((tailObjs.mode == TAIL_MODE_VELOCITY) && (tailObjs.onoff == PID_ON)) {
        tailObjs.p_interpolate += (long) tailObjs.v_input;
        tailObjs.p_error = tailObjs.p_input + tailObjs.p_interpolate - tailObjs.p_state;
        tailObjs.v_error = tailObjs.v_input - tailObjs.v_state;

        //Update values
        UpdateTailPID(&(tailObjs));

        if (tailObjs.pwm_flip) {
            tiHSetDC(tailObjs.output_channel, -tailObjs.output);
        } else {
            tiHSetDC(tailObjs.output_channel, tailObjs.output);
        }
    }

    else if ((tailObjs.mode == TAIL_MODE_RIGHTING) && (tailObjs.onoff == PID_ON)) {
        //Compute orientation vector
        pitch_mod = bodyPose.pitch - (bodyPose.pitch/PITIMES2)*PITIMES2;
        roll_mod = bodyPose.roll - (bodyPose.roll/PITIMES2)*PITIMES2;

        pitch_mod = ABS(pitch_mod);
        roll_mod = ABS(roll_mod);

        if (((pitch_mod>=PIBY2)&&(pitch_mod<=(PI+PIBY2)))!=((roll_mod>=PIBY2)&&(roll_mod<=(PI+PIBY2)))){
            // Increment counter
            tailObjs.counter++;

            // Set p_state to mod pm PI 
            p_mod = tailObjs.p_state - (tailObjs.p_state/PITIMES2)*PITIMES2;
            p_mod = p_mod - (p_mod/PI)*PITIMES2;


            // If counter is above swing period, swing tail to opposite side
            if (tailObjs.counter >= tailObjs.swing_period){
                tailObjs.counter = 0;
                if (((p_mod>=0)&&(tailObjs.p_input>=0))||((p_mod<0)&&(tailObjs.p_input<0))){
                    tailObjs.p_input = -tailObjs.p_input;
                }
            }

            // Update error values
            tailObjs.p_error = tailObjs.p_input - p_mod;
            tailObjs.v_error = tailObjs.v_input - tailObjs.v_state; // note that this will be executed when v_input=0

            //Update PID values
            UpdateTailPID(&(tailObjs));

            if (tailObjs.pwm_flip) {
                tiHSetDC(tailObjs.output_channel, -tailObjs.output);
            } else {
                tiHSetDC(tailObjs.output_channel, tailObjs.output);
            }
 
        } else{ // Turn tail off, zero controller errors
            tailObjs.p_error = 0;
            tailObjs.i_error = 0;
            tailObjs.v_error = 0;

            tailObjs.output = 0;
            tiHSetDC(tailObjs.output_channel, 0);
        }

    }
    else if ((tailObjs.mode == TAIL_MODE_PERIODIC) && (tailObjs.onoff == PID_ON)) {

        // Increment counter
        tailObjs.counter++;

        // If counter is above swing period, switch tail position
        if (tailObjs.counter >= tailObjs.swing_period){
            tailObjs.counter = 0;

            if (tailObjs.swing_sign == 0){
                tailObjs.swing_sign = 1;
                tailObjs.p_input = (long) (tailObjs.p_bias + tailObjs.p_amp); //Note that these are both in units of 2^15 counts per 180 degrees
            } else {
                tailObjs.swing_sign = 0;
                tailObjs.p_input = (long) (tailObjs.p_bias - tailObjs.p_amp);
            }

            tailObjs.p_input = tailObjs.p_input*2*TAIL_GEAR_RATIO; //Scale to correct p_input units
        }

        // Update error values
        tailObjs.p_error = tailObjs.p_input - tailObjs.p_state;
        tailObjs.v_error = tailObjs.v_input - tailObjs.v_state; // note that this will be executed when v_input=0

        //Update PID values
        UpdateTailPID(&(tailObjs));

        if (tailObjs.pwm_flip) {
            tiHSetDC(tailObjs.output_channel, -tailObjs.output);
        } else {
            tiHSetDC(tailObjs.output_channel, tailObjs.output);
        }
    } else {
        tailObjs.output = 0;
        tiHSetDC(tailObjs.output_channel, 0);
    }

}

void UpdateTailPID(pidTail *pid) {
    pid->p = ((long) pid->Kp * pid->p_error) >> 12; // scale so doesn't over flow
    pid->i = (long) pid->Ki * pid->i_error >> 12;
    pid->d = (long) pid->Kd * (long) pid->v_error;
    // better check scale factors

    pid->preSat = pid->feedforward + pid->p +
            ((pid->i) >> 4) + // divide by 16
            (pid->d >> 4); // divide by 16
    pid->output = pid->preSat;

    /* i_error say up to 1 rev error 0x10000, X 256 ms would be 0x1 00 00 00
        scale p_error by 16, so get 12 bit angle value*/
    pid-> i_error = (long) pid-> i_error + ((long) pid->p_error >> 4); // integrate error
    // saturate output - assume only worry about >0 for now
    // apply anti-windup to integrator
    if (pid->preSat > MAXTHROT) {
        pid->output = MAXTHROT;
        pid->i_error = (long) pid->i_error +
                (long) (pid->Kaw) * ((long) (MAXTHROT) - (long) (pid->preSat))
                / ((long) GAIN_SCALER);
    }
    if (pid->preSat < -MAXTHROT) {
        pid->output = -MAXTHROT;
        pid->i_error = (long) pid->i_error +
                (long) (pid->Kaw) * ((long) (-MAXTHROT) - (long) (pid->preSat))
                / ((long) GAIN_SCALER);
    }
}

long tailGetPState() {
    return tailObjs.p_state;
}

void tailStartMotor() {
    tailObjs.timeFlag = 0;
    tailOn();
}

void tailSetTimeFlag(char val) {
    tailObjs.timeFlag = val;
}

// ==== Steering Control Commands ===========================================================================
// ==========================================================================================================

//Initialize the steering control structure
void initStrCtrl(strCtrl *pid, int Kp, int Ki, int Kd, int Kaw)
{
    pid->onoff = 0;
    pid->mode = 0;
    pid->switch_tail_impact = 0;
    
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kaw = Kaw;
    pid->Kd = Kd;
    
    pid->tail_drag_ccw = 0;
    pid->tail_drag_cw = 0;
    pid->tail_drag_delta = 0;
    
    pid->tail_impact_yaw_thresh = 0;
    pid->tail_impact_vel = 0;
    
    pid->yaw_input = 0;
    pid->yaw_error = 0;

    pid->vel_input = 0;
    pid->vel_error = 0;
    
    pid->i_error = 0;

    pid->p = 0;
    pid->i = 0;
    pid->d = 0;

    pid->preSat = 0;
    pid->output = 0;
}

void strCtrlSetGains(int Kp, int Ki, int Kd, int Kaw)
{
    pidSteer.Kp = Kp;
    pidSteer.Ki = Ki;
    pidSteer.Kd = Kd;
    pidSteer.Kaw = Kaw;
}

void strCtrlSetTailParams(int TD_ccw, int TD_cw, int TD_delta, int TI_yaw_thresh, int TI_vel)
{

    // Convert degrees to tail position units as in tailSetPInput when used in strCtrlSetControl
    pidSteer.tail_drag_ccw = TD_ccw;
    pidSteer.tail_drag_cw = TD_cw;
    pidSteer.tail_drag_delta = TD_delta;

    // This comes from cmd.c as in integer with degrees as the unit.
    // Need to cast into a long and multiply by DEG2COUNTS to convert to integrated gyro units.
    pidSteer.tail_impact_yaw_thresh = ((long) TI_yaw_thresh)*DEG2COUNTS;

    // Scale by TAIL_GEAR_RATIO to have correct velocity units as in tailSetVInput. This overflows at around 16 Hz 
    pidSteer.tail_impact_vel = TI_vel*TAIL_GEAR_RATIO;
}

void strCtrlSetMode(char mode, char switch_TI)
{
    pidSteer.mode = mode;
    pidSteer.switch_tail_impact = switch_TI;
}

void strCtrlSetPInput(int input_val)
{
    pidSteer.yaw_input = ((long) input_val)*DEG2COUNTS;
    pidSteer.vel_input = 0;

    pidSteer.p = 0;
    pidSteer.i = 0;
    pidSteer.d = 0;
    pidSteer.i_error = 0;

    pidSteer.onoff = 1;
}

void strCtrlSetVInput(int input_val)
{
    pidSteer.yaw_input = bodyPose.yaw;
    pidSteer.vel_input = input_val;

    pidSteer.p = 0;
    pidSteer.i = 0;
    pidSteer.d = 0;
    pidSteer.i_error = 0;

    pidSteer.onoff = 1;
}

void strCtrlZeroYaw()
{
    bodyPose.yaw = 0;
}

void strCtrlOff()
{
    pidSteer.p = 0;
    pidSteer.i = 0;
    pidSteer.d = 0;
    pidSteer.i_error = 0;

    pidSteer.output = 0;
    
    pidSteer.onoff = 0;
}

void strCtrlSetControl()
{
    // Declare local variables
    long current_yaw_error; // Absolute value of current yaw error used for tail impact control switching

    int output_abs_TD;      // Absolute value of the control output in tail drag counter units

    long p_mod_tail;        // Modulo 2 pi of the current tail state, wrapper to pm pi, used for determining whether to swing the tail another full revolution before hitting the ground
    int rev_count_temp;     // Gives the number of revolutions, adjusted to avoid tail impact with the ground if necessary

    
    // If steering control is on, compute PID updates and apply output based on control mode
    if (pidSteer.onoff == PID_ON){
        
        // Propagate yaw_input if velocity input is nonzero
        pidSteer.yaw_input += ((long) pidSteer.vel_input);

        // Get current values of yaw_error and vel_error
        pidSteer.yaw_error = pidSteer.yaw_input - bodyPose.yaw;
        pidSteer.vel_error = ((long) pidSteer.vel_input) - bodyPose.yaw_vel;

        //Update PID values (for now, we compute this even with tail impact enabled, this will build up tail drag response without implementing it yet)
        UpdateSteerPID(&(pidSteer));
        
        current_yaw_error = ABS(pidSteer.yaw_error);
        // Switch between control cases
        // First check if yaw error is below threshold or if tail impact is disabled, then tail impact cannot happen and we execute diff drive or tail drag
        if ((current_yaw_error < pidSteer.tail_impact_yaw_thresh)||(pidSteer.switch_tail_impact == 0)){

            // Execute differential drive steering
            if (pidSteer.mode == STEER_MODE_DIFFDRIVE){

                // If tail impact switch is enabled and this is the first time entering this mode from tail impact, hold the tail upright
                if((pidSteer.switch_tail_impact == 1)&&(first_DD_NoTI == 1)){
                    // Switch off the first differential drive flag
                    first_DD_NoTI = 0;
                    // Switch on first tail impact flag
                    first_TI = 1;

                    // Compute modulo 2 PI of p_state pm 180 degrees for use with tail input setting
                    p_mod_tail = tailObjs.p_state - (tailObjs.p_state/PITIMES2)*PITIMES2;
                    p_mod_tail = p_mod_tail - (p_mod_tail/PI)*PITIMES2;

                    // If modulo tail state is less than zero and rev_count is positive, swing the tail another revolution ccw before touching down (this prevents tail swinging underneath the robot)
                    if((p_mod_tail < 0)&&(tailObjs.rev_count > 0)){
                        rev_count_temp = tailObjs.rev_count + 1;
                    }
                    // If modulo tail state is greater than zero and rev_count is negative, swing the tail one more revolution cw before touching down (this prevents tail swinging underneath the robot)
                    else if((p_mod_tail > 0)&&(tailObjs.rev_count < 0)){
                        rev_count_temp = tailObjs.rev_count - 1;
                    }
                    else{
                        rev_count_temp = tailObjs.rev_count;
                    }

                    // Set tail control mode to position control and zero out velocity input, interpolate setting, while setting tail position to upright
                    tailObjs.mode = TAIL_MODE_POSITION;
                    tailObjs.p_input = ((long) rev_count_temp)*TAIL_FULL_REV*TAIL_GEAR_RATIO;
                    tailObjs.v_input = 0;
                    tailObjs.p_interpolate = 0;
                    tailObjs.i_error = 0;
                    tailObjs.p = 0;
                    tailObjs.i = 0;
                    tailObjs.d = 0;
                }

                // If the output of the controller is greater than 0, turn counter-clockwise by subtracting velocity from the left legs
                if (pidSteer.output > 0){
                    pidObjs[LEFT_LEGS_PID_NUM].v_input = pidObjs[LEFT_LEGS_PID_NUM].avg_vel - pidSteer.output/LEGVEL2STEER;
                    pidObjs[RIGHT_LEGS_PID_NUM].v_input = pidObjs[RIGHT_LEGS_PID_NUM].avg_vel;
                }
                // Otherwise, the output is less than 0, turn clockwise by subtracting velocity from the right legs
                else{
                    pidObjs[LEFT_LEGS_PID_NUM].v_input = pidObjs[LEFT_LEGS_PID_NUM].avg_vel;
                    pidObjs[RIGHT_LEGS_PID_NUM].v_input = pidObjs[RIGHT_LEGS_PID_NUM].avg_vel + pidSteer.output/LEGVEL2STEER;
                }

                // Propagate forward the position input of each leg set
                pidObjs[LEFT_LEGS_PID_NUM].interpolate += (long) pidObjs[LEFT_LEGS_PID_NUM].v_input;
                pidObjs[RIGHT_LEGS_PID_NUM].interpolate += (long) pidObjs[RIGHT_LEGS_PID_NUM].v_input;
            }
            // Execute tail drag steering
            else{

                // Set tail control mode to position control and zero out velocity input, interpolate setting if this is the first time executing this loop
                if(first_TD == 1){
                    tailObjs.mode = TAIL_MODE_POSITION;
                    tailObjs.v_input = 0;
                    tailObjs.p_interpolate = 0;
                    tailObjs.i_error = 0;
                    tailObjs.p = 0;
                    tailObjs.i = 0;
                    tailObjs.d = 0;

                    // Initialize counter
                    counter_TD = 0;

                    // Set first tail impact flag high and first tail drag flag low
                    first_TI = 1;
                    first_TD = 0;
                }

                // Compute the absolute value of the output in tail drag counter units
                output_abs_TD = ABS(pidSteer.output/TD2STEER);

                // Increment counter indicating the time spent in the tail drag loop
                counter_TD++;

                // If tail is up off the ground
                if(tail_contact_flag == 0){
                    // If counter exceeds limit, set tail down
                    if(counter_TD > (TD_COUNTER_MAX - output_abs_TD)){

                        // Switch tail contact flag to indicate contact and re-zero counter
                        counter_TD = 0;
                        tail_contact_flag = 1;

                        // Compute modulo 2 PI of p_state pm 180 degrees for use with tail input setting
                        p_mod_tail = tailObjs.p_state - (tailObjs.p_state/PITIMES2)*PITIMES2;
                        p_mod_tail = p_mod_tail - (p_mod_tail/PI)*PITIMES2;

                        // If modulo tail state is less than zero and rev_count is positive, swing the tail another revolution ccw before touching down (this prevents tail swinging underneath the robot)
                        if((p_mod_tail < 0)&&(tailObjs.rev_count > 0)){
                            rev_count_temp = tailObjs.rev_count + 1;
                        }
                        // If modulo tail state is greater than zero and rev_count is negative, swing the tail one more revolution cw before touching down (this prevents tail swinging underneath the robot)
                        else if((p_mod_tail > 0)&&(tailObjs.rev_count < 0)){
                            rev_count_temp = tailObjs.rev_count - 1;
                        }
                        else{
                            rev_count_temp = tailObjs.rev_count;
                        }

                        // If output sign is greater than zero, turn counter-clockwise by touching the tail counter-clockwise
                        if(pidSteer.output > 0){
                            // tail_drag_ccw has units of degrees
                            tailObjs.p_input = (((long) pidSteer.tail_drag_ccw)*TAIL_FULL_REV)/TAIL_DEG_GR_SCALER +  ((long) (rev_count_temp))*TAIL_FULL_REV*TAIL_GEAR_RATIO;
                        }
                        // Otherwise turn clockwise by touching the tail clockwise
                        else{
                            // tail_drag_cw has units of degrees
                            tailObjs.p_input = (((long) pidSteer.tail_drag_cw)*TAIL_FULL_REV)/TAIL_DEG_GR_SCALER +  ((long) rev_count_temp)*TAIL_FULL_REV*TAIL_GEAR_RATIO;
                        }
                    }
                }
                // Otherwise tail is contacting the ground
                else{
                    // If counter exceeds limit, pick tail up
                    if(counter_TD > (TD_COUNTER_MIN + output_abs_TD)){
                        // Switch tail contact flag to indicate loss of contact and re-zero counter
                        counter_TD = 0;
                        tail_contact_flag = 0;

                        // Compute modulo 2 PI of p_state pm 180 degrees for use with tail input setting
                        p_mod_tail = tailObjs.p_state - (tailObjs.p_state/PITIMES2)*PITIMES2;
                        p_mod_tail = p_mod_tail - (p_mod_tail/PI)*PITIMES2;

                        // If modulo tail state is less than zero and rev_count is positive, swing the tail another revolution ccw before touching down (this prevents tail swinging underneath the robot)
                        if((p_mod_tail < 0)&&(tailObjs.rev_count > 0)){
                            rev_count_temp = tailObjs.rev_count + 1;
                        }
                        // If modulo tail state is greater than zero and rev_count is negative, swing the tail one more revolution cw before touching down (this prevents tail swinging underneath the robot)
                        else if((p_mod_tail > 0)&&(tailObjs.rev_count < 0)){
                            rev_count_temp = tailObjs.rev_count - 1;
                        }
                        else{
                            rev_count_temp = tailObjs.rev_count;
                        }

                        // If output sign is greater than zero, prepare for turning counter-clockwise by holding the tail above the counter-clockwise position
                        if(pidSteer.output > 0){
                            // tail_drag_ccw is assumed to be in the range [0 180] deg, tail_drag_delta is positive
                            tailObjs.p_input = (((long) pidSteer.tail_drag_ccw - (long) pidSteer.tail_drag_delta)*TAIL_FULL_REV)/TAIL_DEG_GR_SCALER +  ((long) rev_count_temp)*TAIL_FULL_REV*TAIL_GEAR_RATIO;
                        }
                        // Otherwise prepare for turning clockwise by holding tail above the clockwise contact position
                        else{
                            // tail_drag_cw has units of degrees and is assumed to be in the range [-180 0] deg
                            tailObjs.p_input = (((long) pidSteer.tail_drag_cw + (long) pidSteer.tail_drag_delta)*TAIL_FULL_REV)/TAIL_DEG_GR_SCALER +  ((long) rev_count_temp)*TAIL_FULL_REV*TAIL_GEAR_RATIO;
                        }
                    }
                }
            }
        }
        // Execute tail impact steering
        else{
            // If this is the first time entering the tail impact loop, set mode to tail velocity, set p_input to current p_state, and set p_interpolate to zero
            if(first_TI == 1){
                tailObjs.mode = TAIL_MODE_VELOCITY;
                tailObjs.p_input = tailObjs.p_state;
                tailObjs.p_interpolate = 0;
                tailObjs.i_error = 0;
                tailObjs.p = 0;
                tailObjs.i = 0;
                tailObjs.d = 0;
                
                // Set first tail impact flag low and other flags high
                first_TI = 0;
                first_DD_NoTI = 1;
                first_TD = 1;

                // Set tail velocity positive or negative based on the sign of the yaw error
                if(pidSteer.yaw_error < 0){
                    // tail_impact vel units are TAIL_FULL_REV*TAIL_GEAR_RATIO/1000 counts per Hz
                    tailObjs.v_input = pidSteer.tail_impact_vel;
                }
                else{
                    tailObjs.v_input = -pidSteer.tail_impact_vel;
                }
            }

            // Execute differential drive steering as well if this steering mode is engaged
            if (pidSteer.mode == STEER_MODE_DIFFDRIVE){

                // If the output of the controller is greater than 0, turn counter-clockwise by subtracting velocity from the left legs
                if (pidSteer.output>0){
                    pidObjs[LEFT_LEGS_PID_NUM].v_input = pidObjs[LEFT_LEGS_PID_NUM].avg_vel - pidSteer.output/LEGVEL2STEER;
                    pidObjs[RIGHT_LEGS_PID_NUM].v_input = pidObjs[RIGHT_LEGS_PID_NUM].avg_vel;
                }
                // Otherwise, the output is less than 0, turn clockwise by subtracting velocity from the right legs
                else{
                    pidObjs[LEFT_LEGS_PID_NUM].v_input = pidObjs[LEFT_LEGS_PID_NUM].avg_vel;
                    pidObjs[RIGHT_LEGS_PID_NUM].v_input = pidObjs[RIGHT_LEGS_PID_NUM].avg_vel + pidSteer.output/LEGVEL2STEER;
                }

                // Propagate forward the position input of each leg set
                pidObjs[LEFT_LEGS_PID_NUM].interpolate += (long) pidObjs[LEFT_LEGS_PID_NUM].v_input;
                pidObjs[RIGHT_LEGS_PID_NUM].interpolate += (long) pidObjs[RIGHT_LEGS_PID_NUM].v_input;
            }
        }
    }
    // If steering control is off, make sure p, i, d, terms of steering controller are zeroed out and output is set to zero
    else{
        pidSteer.p = 0;
        pidSteer.i = 0;
        pidSteer.d = 0;
        pidSteer.i_error = 0;

        pidSteer.output = 0;
    }
}

void UpdateSteerPID(strCtrl *pid) {
    // Units of yaw_error are 2^14 counts per degree.
    // If yaw_error is 60 degrees and the p term is divided by 2^16 in total, then Kp = 1000 would almost saturate the output with p if saturation is at 2^14
    pid->p = ((long) pid->Kp) * (pid->yaw_error >> 12);
    // i_error accumulates by scaled yaw_error every 1 ms.
    // If yaw_error is 30 degrees for 500 ms and the i term is divided by 2^20 in total, then Ki = 70 saturates the output with i.
    pid->i = (long) pid->Ki * (pid->i_error >> 12);
    // Units of vel_error are 16.384 counts per deg/s
    // If vel_error is 500 deg/s and the d term is divided by 2^8 in total, then Kd = 500 would almost saturate the output with d
    pid->d = (long) pid->Kd * (long) pid->vel_error;

    pid->preSat = (pid->p >> 4) + // divide by 16
            ((pid->i) >> 4) + // divide by 16
            (pid->d >> 8); // divide by 256
    pid->output = pid->preSat;

    // If yaw_error is held steady 30 deg, then i_error would overflow after 70 seconds
    pid-> i_error = (long) pid-> i_error + ((long) pid->yaw_error >> 4); // integrate error, divide yaw_error by 16 before accumulating to prevent premature overflow
    // saturate output
    // apply anti-windup to integrator
    if (pid->preSat > STEER_MAX) {
        pid->output = STEER_MAX;
        // Say i_error = 30*16384*500/(2^4)*2 = 30720000, which is twice the amount that saturates the output assuming Ki = 70
        // Then STEER_MAX - preSat = -16384 and Kaw = 2000 would desaturate the signal in 100 ms
        pid->i_error = (long) pid->i_error +
                (long) (pid->Kaw) * ((long) (STEER_MAX) - (long) (pid->preSat))
                / ((long) GAIN_SCALER);
    }
    if (pid->preSat < -STEER_MAX) {
        pid->output = -STEER_MAX;
        pid->i_error = (long) pid->i_error +
                (long) (pid->Kaw) * ((long) (-STEER_MAX) - (long) (pid->preSat))
                / ((long) GAIN_SCALER);
    }
}

// ==== Orientation Commands ================================================================================
// ==========================================================================================================

void initBodyPose(poseEstimateStruct *pose){
    pose->pitch = 0;
    pose->roll = 0;
    pose->yaw = 0;
    
    pose->gx_offset = 0;
    pose->gy_offset = 0;
    pose->gz_offset = 0;
    
    pose->count = 0;
}

// Computes sine mapped to -2^9 to 2^9 for input angles in units of counts
long computeSINApprox(long x){
    long x_mod;
    long y;
    long y2;
    long result;

    // Take the modulo 2PI of the angle, wrap from -180 to +180
    x_mod = x - (x/PITIMES2)*PITIMES2;
    if (x_mod > PI){
        x_mod-=PITIMES2;
    }
    else if (x_mod  < (-PI)){
        x_mod+=PITIMES2;
    }

    // Bitshift x_mod to prevent overflow and compute its squared value
    y = x_mod >> 8;
    y2 = y*y;

    // Sine approximation by Bhaskara I
    // https://en.wikipedia.org/wiki/Bhaskara_I%27s_sine_approximation_formula
    if (x_mod >= 0){
        result = ((180*y - (y2 >> 6)) << 9)/(648000 - 45*y + (y2 >> 8));
    }
    else{
        result = ((180*y + (y2 >> 6)) << 9)/(648000 + 45*y + (y2 >> 8));
    }
    
    return result;

}

void computeEulerAngles(){
    // Local variables for computation
    int xldata[3];
    int gdata[3];

    // Initial roll computation
    long AX2;
    long numR;
    long denR;

    // Initial pitch computation
    long Q2;
    long I2;
    long numP;
    long denP;

    // Euler angle propagation based on body-fixed angular velocities
    long c_pitch;
    long s_pitch;
    long c_yaw;
    long s_yaw;
    long wx;
    long wy;
    long wz;

    int i;

    // If there is no motor action, initialize Euler angles using the accelerometer,
    // and get gyro zero offsets
    if((pidObjs[0].onoff==0)&&(tailObjs.onoff==0)){
        mpuGetXl(xldata);
        mpuGetGyro(gdata);

        // Increment accumulation counter
        bodyPose.count++;

        // If pose reset flag is triggered, increment counter
        if (poseResetFlag == 1){
            poseResetCounter++;

            // If pose reset counter exceeds count limit, flip flag off and turn motors back on
            if (poseResetCounter>POSERESETCOUNTS){
                poseResetFlag = 0;
                tailObjs.onoff = 1;
                pidObjs[0].onoff = 1;
                pidObjs[1].onoff = 1;
                poseResetCounter = 0;
            }
        }

        // If repeat wait flag is triggered, restart trial if xldata[2] falls below threshold
        if ((repeatWaitFlag == 1)&&(xldata[2]<=AZTHRESH)){
            // xlzTriggerFlag goes high
            xlzTriggerFlag = 1;
        }

        if (xlzTriggerFlag == 1){
            // Increment repeat wait counter
            repeatWaitCounter++;

            if (repeatWaitCounter>REPEATCOUNTS){
                // Zero flags and counter
                repeatWaitFlag = 0;
                xlzTriggerFlag = 0;
                repeatWaitCounter = 0;

                // Reset timed trial vars drive PID obj 0
                pidSetTimeFlag(0,1);
                pidSetInput(0, 0);
                checkSwapBuff(0);
                pidOn(0);

                // Reset timed trial vars drive PID obj 1
                pidSetTimeFlag(1,1);
                pidSetInput(1, 0);
                checkSwapBuff(1);
                pidOn(1);

                // Start drive motors timed run
                pidStartTimedTrial(pidObjs[0].run_time);

                // Zero tail
               // tail_p_input_temp = tailObjs.p_input;
               // tail_v_input_temp = tailObjs.v_input;
                //tailZeroPos();
               // tailObjs.p_input = tail_p_input_temp;
               // tailObjs.v_input = tail_v_input_temp;

                // Reset timed trial vars tail PID obj
                tailSetTimeFlag(1);
                tailOn();

                // Start tail motor timed run
                tailStartTimedTrial(tailObjs.run_time);
            }
        }

        // Accumulate gyro offset
        for (i = 0; i < 3; i++) {
            goffset_acc[i] += (long) gdata[i];
        }

        // Accumulate roll angle estimate based on Pade approximation to arcsine
        AX = (long) xldata[0];
        AX2 = AX*AX;

        numR = AX*GCONST*SCALEROLL_1 - ((SCALEROLL_1*AX*AX)/GCONST)*((17*AX)/60);
        denR = ((long) GCONST)*GCONST - (9*AX2)/20;
        // Flip the sign of the roll angle calculation because of the robot coordinates
        angle_acc[0] -= (numR/denR)*SCALEROLL_2;

        // Accumulate pitch angle estimate based on the eight-octant approximation to arctangent below
        // http://www.embedded.com/design/other/4216719/Performing-efficient-arctangent-approximation
        I = (long) xldata[2];
        Q = (long) xldata[1];

        if((Q <= I)&&(Q >= -I)) { // Octants 1 or 8
            Q2 = Q*Q;
            numP = I*Q*SCALEPITCH_1;
            denP = I*I + (Q2 >> 2) + (Q2 >> 5);
            angle_acc[1] += (numP/denP)*SCALEPITCH_2;
        }
        else if ((Q >= I)&&(Q >= -I)){ // Octants 2 or 3
            I2 = I*I;
            numP = I*Q*SCALEPITCH_1;
            denP = Q*Q + (I2 >> 2) + (I2 >> 5);
            angle_acc[1] += PIBY2 - (numP/denP)*SCALEPITCH_2;
        }
        else if ((Q >= I)&&(Q <= -I)){ //Octants 4 or 5
            Q2 = Q*Q;
            numP = I*Q*SCALEPITCH_1;
            denP = I*I + (Q2 >> 2) + (Q2 >> 5);
            angle_acc[1] += PI + (numP/denP)*SCALEPITCH_2;
        }
        else if ((Q <= I)&&(Q <= -I)){ //Octants 6 or 7
            I2 = I*I;
            numP = I*Q*SCALEPITCH_1;
            denP = Q*Q + (I2 >> 2) + (I2 >> 5);
            if (I<=0){ //Shift octant 6 approximation up by 2 pi to make wrapping point at -90, 270
                angle_acc[1] += (PI+PIBY2) - (numP/denP)*SCALEPITCH_2;
            }
            else{
                angle_acc[1] += -PIBY2 - (numP/denP)*SCALEPITCH_2;
            }
        }

        // After a set amount of time, average the accumulated Euler angles and gyro offsets
        if (bodyPose.count>=AVGWINDOW){
            bodyPose.roll = angle_acc[0]/((long) bodyPose.count);
            bodyPose.pitch = angle_acc[1]/((long) bodyPose.count);

            bodyPose.gx_offset = (int) (goffset_acc[0]/((long) bodyPose.count));
            bodyPose.gy_offset = (int) (goffset_acc[1]/((long) bodyPose.count));
            bodyPose.gz_offset =  (int) (goffset_acc[2]/((long) bodyPose.count));

            angle_acc[0] = 0;
            angle_acc[1] = 0;

            goffset_acc[0] = 0;
            goffset_acc[1] = 0;
            goffset_acc[2] = 0;

            bodyPose.count = 0;
//            // Set pid objects on for debugging
//            pidObjs[0].onoff = 1;
//            tailObjs.onoff = 1;
        }
    }
    // Otherwise use integrated gyro to measure pitch angle
    else{
        mpuGetGyro(gdata);
        // Adjust grabbed angular velocities by offset; convert to long
        wx = (long) (gdata[0] - bodyPose.gx_offset);
        wy = (long) (gdata[1] - bodyPose.gy_offset);
        wz = (long) (gdata[2] - bodyPose.gz_offset);

        // Compute sine and cosine values of pitch and roll angles
        s_pitch = computeSINApprox(bodyPose.pitch);
        c_pitch = computeSINApprox(bodyPose.pitch + PIBY2);
        s_yaw = computeSINApprox(bodyPose.yaw);
        c_yaw = computeSINApprox(bodyPose.yaw + PIBY2);

        // Prevent divide by zero cases when cosine of pitch is equal to zero
        if (c_pitch == 0){
            c_pitch = 1;
        }

        // If cosine of pitch becomes too small, then raise flag to toggle motors off to recalibrate pose with accelerometer
        if ((c_pitch>=-COSTHRESH)&&(c_pitch<=COSTHRESH)){
            poseResetFlag = 1;
            tailObjs.onoff = 0;
            pidObjs[0].onoff = 0;
            pidObjs[1].onoff = 0;
            bodyPose.count = 0;
        }

        // Propagate Euler angles based on projection of body-fixed basis vectors onto Euler basis, store yaw velocity for use with steering controller
        bodyPose.roll += (s_yaw*wx + c_yaw*wy)/c_pitch;
        bodyPose.pitch += ((c_yaw*wx - s_yaw*wy) >> 9);
        bodyPose.yaw_vel = (((s_pitch*((s_yaw*wx + c_yaw*wy) >> 3))/c_pitch) >> 6) + wz;
        bodyPose.yaw += bodyPose.yaw_vel;
    }

}