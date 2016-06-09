#ifndef __PID_H
#define __PID_H

// better to turn gains to zero until initialized by command
#define DEFAULT_KP  0
#define DEFAULT_KI  0
#define DEFAULT_KD  0
#define DEFAULT_KAW 0
#define DEFAULT_FF  0

#define GAIN_SCALER     100
// Load cell calibration
#define K_LOAD_CELL_1   4      //hundreth of a gram per count
#define K_LOAD_CELL_2   34
#define SWITCH_LOAD_CELL 640
#define NUM_PIDS	2
#define NUM_PI_NO_AMS   1
#define NUM_VELS	4 // 8 velocity setpoints per cycle
#define NUM_BUFF 	2 // Number of strides buffered in to get setpoint

// Set constants of lower and upper bounds to respond to errors; for now set at 3*pi/2 and 7*pi/4
#define ERR_FWD_BOUND 49151
#define ERR_BWD_BOUND 57337


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

#define PID_MODE_CONTROLED  0
#define PID_MODE_PWMPASS    1

// pid type for leg control
typedef struct
{
	long p_input;	// reference position input - [16].[16]
	long p_state;	// current position
	long p_error;  // position error
	int v_input; // reference velocity input
	int v_state; // current velocity
	int v_error; // velocity error
	long i_error; // integral error
	long  p, i, d;   // control contributions from position, integral, and derivative gains respectively
  	long preSat; // output value before saturations
	int  output;	 //  control output u
 	char onoff; //boolean
 	char mode; //Motor mode: 1 iff PWM open loop control
        char angle_trig; //Angle trigger: 0 if inactive, 1 if rising, 2 if falling
        char angle_setpt;
 	int pwmDes; // Desired PWM
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

// pi type for winch control
typedef struct
{
	long p_input; // reference load input
//      long v_input;
//      long v_state; // back emf state
	long p_state; // load state
	long p_error; // load error
	long i_error; // integrated load error
	long  p, i;   // control contributions from position, integral gains respectively
  	long preSat; // output value before saturations
	int  output;	 //  control output u
 	char onoff; //boolean
 	char mode; //Motor mode: 1 iff unwind on/off control
 	char timeFlag;
	unsigned long run_time;
	unsigned long start_time;
	int bemfOffset;  // BEMF setpoint offset
        int sensorOffset; // Load cell ambient light offset
	int feedforward;
        int Kp, Ki;
	int Kaw;  // anti-windup gain
	//Configuration parameters
        unsigned char output_channel;
        unsigned char pwm_flip;
} piWinch;

// structure for velocity control of leg cycle

typedef struct
{ 
	int interval[NUM_VELS];	// number of ticks between intervals
	int delta[NUM_VELS];   // increments for right setpoint
	int vel[NUM_VELS];     // velocity increments to setpoint, >>8
	int onceFlag;
} pidVelLUT;

//Defaults for leg config switches
//TODO: Check these for consistency with the default wiring diagram for VR
//Left legs
#ifndef LEFT_LEGS_PID_NUM
#define LEFT_LEGS_PID_NUM       0       //PID module index is 0-3
#endif
#ifndef LEFT_LEGS_ENC_NUM
#define LEFT_LEGS_ENC_NUM       0       //amsEnc module index is 0-3
#endif
#ifndef LEFT_LEGS_ENC_FLIP
#define LEFT_LEGS_ENC_FLIP      0       //"forward" normal for left
#endif
#ifndef LEFT_LEGS_PWM_FLIP
#define LEFT_LEGS_PWM_FLIP      0
#endif
#ifndef LEFT_LEGS_TIH_CHAN
#define LEFT_LEGS_TIH_CHAN      1       //tiH module index is 1-4
#endif
//Right legs
#ifndef RIGHT_LEGS_PID_NUM
#define RIGHT_LEGS_PID_NUM      1       //PID module index is 0-3
#endif
#ifndef RIGHT_LEGS_ENC_NUM
#define RIGHT_LEGS_ENC_NUM      1       //amsEnc module index is 0-3
#endif
#ifndef RIGHT_LEGS_ENC_FLIP
#define RIGHT_LEGS_ENC_FLIP     1       //"forward" reversed for right
#endif
#ifndef RIGHT_LEGS_PWM_FLIP
#define RIGHT_LEGS_PWM_FLIP     0
#endif
#ifndef RIGHT_LEGS_TIH_CHAN
#define RIGHT_LEGS_TIH_CHAN     2       //tiH module index is 1-4
#endif

//Winch
#ifndef WINCH_PI_NUM
#define WINCH_PI_NUM      0       //PI module index is 0-3
#endif
#ifndef WINCH_PWM_FLIP
#define WINCH_PWM_FLIP    0
#endif
#ifndef WINCH_TIH_CHAN
#define WINCH_TIH_CHAN    3       //tiH module index is 1-4
#endif


//Functions
void UpdatePID(pidPos *pid);
void UpdatePI(piWinch *pi);
void pidSetup();
void initPIDVelProfile();
void setPIDVelProfile(int pid_num, int *interval, int *delta, int *vel, int onceFlag);
void initPIDObjPos(pidPos *pid, int Kp, int Ki, int Kd, int Kaw, int ff);
void initPIObjPos(piWinch *pi, int Kp, int Ki, int Kaw, int ff);
//void SetupTimer1(void);
void pidStartTimedTrial(unsigned int run_time);
void pidSetInput(int pid_num, int input_val);
void piSetInput(int pi_num, int input_val);
void pidSetGains(int pid_num, int Kp, int Ki, int Kd, int Kaw, int ff);
void piSetGains(int pi_num, int Kp, int Ki, int Kaw, int ff);
void pidGetState(); // update state vector from bemf and Hall angle
void piGetState();
void pidGetSetpoint(int j);
void checkSwapBuff(int j);
void pidSetControl();
void piSetControl();
void checkPitchStopCondition();
void EmergencyStop(void);
unsigned char* pidGetTelemetry(void);
void pidOn(int pid_num);
void pidOff(int pid_num);
void piOn(int pi_num);
void piOff(int pi_num);
void pidZeroPos(int pid_num);
void piSetSensorOffset(int pi_num);
void calibBatteryOffset(int spindown_ms);
long pidGetPState(unsigned int channel);
void pidSetPInput(unsigned int channel, long p_input);
void pidStartMotor(unsigned int channel);
void pidSetTimeFlag(unsigned int channel, char val);
void pidSetMode(unsigned int channel, char mode);
void pidSetPWMDes(unsigned int channel, int pwm);

void pidSetPitchThresh(unsigned int channel, char angle);
void pidSetPitchTrigger(unsigned int channel, char mode);

int piGetSensorOffset(unsigned int channel);
void piSetTimeFlag(unsigned int channel, char val);
void piSetMode(unsigned int channel, char mode);

#endif // __PID_H
