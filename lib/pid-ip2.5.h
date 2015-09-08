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
#define K_LOAD_CELL_1   24      //hundreth of a gram per count
#define K_LOAD_CELL_2   68
#define SWITCH_LOAD_CELL 673
#define NUM_PIDS	2
#define NUM_PI_NO_AMS   1
#define NUM_VELS	4 // 8 velocity setpoints per cycle
#define NUM_BUFF 	2 // Number of strides buffered in to get setpoint

// Set constants of lower and upper bounds to respond to errors; for now set at pi/2 and 3pi/2
#define ERR_FWD_BOUND 16384
#define ERR_BWD_BOUND 49151


/* The back emf constant can be measured by measuring velocity from Hall encoder 
* 80 rad/sec = 12.5 rev/sec = 834 encPos[].pos/sec
* 80 rad/sec gives 140 A/D units
* v_input should be in A/D units
* thus v_input = 140 * vel[i] / 834  
* scale by 256 to get resolution in constant */
// A/D units per encoder change per ms scaled by >> 8 
// K_EMF = ((256 * 140) / 834)  =43
#define K_EMF 43

#ifndef ADC_MAX
#define ADC_MAX             1024
#endif

//Structures and enums

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
	int inputOffset;  // BEMF setpoint offset
	int feedforward;
    int Kp, Ki, Kd;
	int Kaw;  // anti-windup gain
	//Leg control variables
	long interpolate;  				// intermediate value between setpoints
	unsigned long expire;		// end of current segment
	int index;					// right index to moves
	int leg_stride;
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
	//Leg control variables
} piWinch;

// structure for velocity control of leg cycle

typedef struct
{ 
	int interval[NUM_VELS];	// number of ticks between intervals
	int delta[NUM_VELS];   // increments for right setpoint
	int vel[NUM_VELS];     // velocity increments to setpoint, >>8
	int onceFlag;
} pidVelLUT;

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
void piOn(int pid_num);
void pidZeroPos(int pid_num);
void piSetSensorOffset(int pi_num);
void calibBatteryOffset(int spindown_ms);

#endif // __PID_H
