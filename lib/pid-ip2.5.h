#ifndef __PID_H
#define __PID_H

// better to turn gains to zero until initialized by command
#define DEFAULT_KP  0
#define DEFAULT_KI  0
#define DEFAULT_KD  0
#define DEFAULT_KAW 0
#define DEFAULT_FF  0

#define GAIN_SCALER         100
#define NUM_PIDS	4
#define NUM_VELS	4 // 8 velocity setpoints per cycle
#define NUM_BUFF 	2 // Number of strides buffered in to get setpoint


/* The back emf constant can be measured by measuring velocity from Hall encoder 
* 80 rad/sec = 12.5 rev/sec = 834 encPos[].pos/sec
* 80 rad/sec gives 140 A/D units
* v_input should be in A/D units
* thus v_input = 140 * vel[i] / 834  
* scale by 256 to get resolution in constant */
// A/D units per encoder change per ms scaled by >> 8 
// K_EMF = ((256 * 140) / 834)  =43
#define K_EMF 43

//#ifndef ADC_MAX
//#define ADC_MAX             1024
//#endif

//Structures and enums
#define PID_OFF     0
#define PID_ON      1

#define PID_MODE_CONTROLLED  0
#define PID_MODE_PWMPASS     1
#define PID_MODE_CLSTEER     2

//constants for steering control
#define GZ_AVG_WINDOW      50

// pid type for leg control
typedef struct
{
	long p_input;                   // reference position input - [16].[16]
	long p_state;                   // current position
	long p_error;                   // position error
	int v_input;                    // reference velocity input
	int v_state;                    // current velocity
	int v_error;                    // velocity error
	long i_error;                   // integral error
	long  p, i, d;                  // control contributions from position, integral, and derivative gains respectively
  	long preSat;                    // output value before saturations
	int  output;                    //  control output u
 	char onoff;                     //boolean
        //TODO: Replace mode with 'bypass' bit?
 	char mode;                      //Motor mode: 1 iff PWM open loop control
 	int pwmDes;                     // Desired PWM
 	char timeFlag;
	unsigned long run_time;
	unsigned long start_time;
	int inputOffset;                // BEMF setpoint offset
	int feedforward;
        int Kp, Ki, Kd;
	int Kaw;                        // anti-windup gain
	//Leg control variables
	long interpolate;  		// intermediate value between setpoints
	unsigned long expire;		// end of current segment
	int index;			// right index to moves
	int leg_stride;
        unsigned char p_state_flip;     //boolean; flip or do not flip
        unsigned char output_channel;
        unsigned char encoder_num;
        unsigned char pwm_flip;
} pidPos;

// structure for velocity control of leg cycle

typedef struct
{ 
	int interval[NUM_VELS];	// number of ticks between intervals
	int delta[NUM_VELS];   // increments for right setpoint
	int vel[NUM_VELS];     // velocity increments to setpoint, >>8
	int onceFlag;
} pidVelLUT;

// structure for steering control (phase deviation from alternating tripod)
typedef struct
{
    char onoff; //boolean toggling steering control
    int Kp, Ki; //proportional, integral gains (control goal is to maintain turn rate)
    int feedforward; //feedforward
    int thrust_nom; //nominal PWM
    long yaw_input; //reference yaw angle to track
    long yaw_state; //angle state from integrated gyro
    long yaw_error; //angle error
    int vel_input; //angular velocity input
    int vel_state; //velocity state from gyro
    int vel_error; //velocity error
    long p, i; //control contributions from proportional, integral gains
    long preSat; //output value before saturation
    int output; //output is PWM differential
} strCtrl;

//Defaults for leg config switches
//TODO: Check these for consistency with the default wiring diagram for VR
//Left legs robot 1
#ifndef LEFT_1_PID_NUM
#define LEFT_1_PID_NUM       0       //PID module index is 0-3
#endif
#ifndef LEFT_1_ENC_NUM
#define LEFT_1_ENC_NUM       0       //amsEnc module index is 0-3
#endif
#ifndef LEFT_1_ENC_FLIP
#define LEFT_1_ENC_FLIP      0       //"forward" normal for left
#endif
#ifndef LEFT_1_PWM_FLIP
#define LEFT_1_PWM_FLIP      0
#endif
#ifndef LEFT_1_TIH_CHAN
#define LEFT_1_TIH_CHAN      1       //tiH module index is 1-4
#endif

//Right legs robot 1
#ifndef RIGHT_1_PID_NUM
#define RIGHT_1_PID_NUM      1       //PID module index is 0-3
#endif
#ifndef RIGHT_1_ENC_NUM
#define RIGHT_1_ENC_NUM      1       //amsEnc module index is 0-3
#endif
#ifndef RIGHT_1_ENC_FLIP
#define RIGHT_1_ENC_FLIP     1       //"forward" reversed for right
#endif
#ifndef RIGHT_1_PWM_FLIP
#define RIGHT_1_PWM_FLIP     0
#endif
#ifndef RIGHT_1_TIH_CHAN
#define RIGHT_1_TIH_CHAN     2       //tiH module index is 1-4
#endif

//Left legs robot 2
#ifndef LEFT_2_PID_NUM
#define LEFT_2_PID_NUM       2       //PID module index is 0-3
#endif
#ifndef LEFT_2_ENC_NUM
#define LEFT_2_ENC_NUM       2       //amsEnc module index is 0-3
#endif
#ifndef LEFT_2_ENC_FLIP
#define LEFT_2_ENC_FLIP      0       //"forward" normal for left
#endif
#ifndef LEFT_2_PWM_FLIP
#define LEFT_2_PWM_FLIP      0
#endif
#ifndef LEFT_2_TIH_CHAN
#define LEFT_2_TIH_CHAN      3       //tiH module index is 1-4
#endif

//Right legs robot 2
#ifndef RIGHT_2_PID_NUM
#define RIGHT_2_PID_NUM      3       //PID module index is 0-3
#endif
#ifndef RIGHT_2_ENC_NUM
#define RIGHT_2_ENC_NUM      3       //amsEnc module index is 0-3
#endif
#ifndef RIGHT_2_ENC_FLIP
#define RIGHT_2_ENC_FLIP     1       //"forward" reversed for right
#endif
#ifndef RIGHT_2_PWM_FLIP
#define RIGHT_2_PWM_FLIP     0
#endif
#ifndef RIGHT_2_TIH_CHAN
#define RIGHT_2_TIH_CHAN     4       //tiH module index is 1-4
#endif


//Functions
void UpdatePID(pidPos *pid);
void pidSetup();
void initPIDVelProfile();
void setPIDVelProfile(int pid_num, int *interval, int *delta, int *vel, int onceFlag);
void initPIDObjPos(pidPos *pid, int Kp, int Ki, int Kd, int Kaw, int ff);
//void SetupTimer1(void);
void pidStartTimedTrial(unsigned int run_time);
void pidSetInput(int pid_num, int input_val);
void pidSetGains(int pid_num, int Kp, int Ki, int Kd, int Kaw, int ff);
void pidGetState(); // update state vector from bemf and Hall angle
void pidGetSetpoint(int j);
void checkSwapBuff(int j);
void pidSetControl();
void EmergencyStop(void);
unsigned char* pidGetTelemetry(void);
void pidOn(int pid_num);
void pidOff(int pid_num);
void pidZeroPos(int pid_num);
void calibBatteryOffset(int spindown_ms);
long pidGetPState(unsigned int channel);
void pidSetPInput(unsigned int channel, long p_input);
void pidStartMotor(unsigned int channel);
void pidSetTimeFlag(unsigned int channel, char val);
void pidSetMode(unsigned int channel, char mode);
void pidSetPWMDes(unsigned int channel, int pwm);

//Steering control functions
void initStrCtrl(strCtrl *pi, int Kp, int Kd, int Kff);
void strCtrlSetGains(int Kp, int Kd, int Kff, int thrust);
void strCtrlSetInput(int input_val);
void strCtrlOff();
void strCtrlGetState();
void strCtrlSetControl();
void UpdatePI(strCtrl *pi);

#endif // __PID_H
