#ifndef RB_SHARED_MEMORY_H
#define RB_SHARED_MEMORY_H

#define RBCORE_SHM_NAME       "RBCORE_SHARED_MEMORY"

#define MAX_JOINT   2
#define MAX_MC      29
#define MAX_FT      4
#define MAX_IMU     1
#define MAX_SP      1
#define MAX_OF      2
#define MAX_AL      20

#define MAX_MANUAL_CAN          30
#define MAX_COMMAND_DATA        20

#define MOTOR_1CH               1
#define MOTOR_2CH               2


#define COMMAND_CANID           0x01
#define SENSOR_REQUEST_CANID    0x03    //0x02
#define RT_TIMER_PERIOD_MS      5

#define RBCORE_PODO_NO          0

#define ENABLE					1
#define DISABLE					0

#define RBCORE_PI               3.141592f


#define MAX_LASER_DATA          1080

typedef enum{
    MANUALCAN_NEW = 0,
    MANUALCAN_EMPTY,
    MANUALCAN_WRITING
} MANUALCAN_STATUS;
typedef	struct _MANUAL_CAN_{
    unsigned char		channel;
    unsigned int		id;
    unsigned char		data[8];
    unsigned char		dlc;
    MANUALCAN_STATUS	status;
} MANUAL_CAN;

typedef struct _VISION2MOTION_
{
    float pos[3];
    float ori[4];
} VISION2MOTION, *pVISION2MOTION;

typedef union{
    struct{
        unsigned    HIP:1;	 	// Open loop(PWM) Control mode
        unsigned    RUN:1;		// Control ON
        unsigned    MOD:1;		// Control mode
        unsigned    LIM:1;		// Limit sensor
        unsigned    HME:4;		// Limit Sensor mode during Homing

        unsigned    JAM:1;		// JAM error
        unsigned    PWM:1;		// PWM saturation
        unsigned    BIG:1;		// Big Position error
        unsigned    INP:1;              // Big Input error
        unsigned    FLT:1;		// FET Driver Fault 1= Fault
        unsigned    ENC:1;              // encoder fail
        unsigned    CUR:1;		// big current difference <<== used for CAN connection check
        unsigned    TMP:1;		// Temperature warning

        unsigned    PS1:1;		// Position Limit 1
        unsigned    PS2:1;		// Position Limit 2
        unsigned    SWM:1;		// switching mode (1:complementary, 0:non-complementary)
        unsigned    GOV:1;		// gain override
        unsigned    FRC:1;		// friction compensation
        unsigned    REF:1;		// reference mode (1:incremental, 0:absolute)
        unsigned    CAN:1;		// CAN has been recovered

        unsigned    RPL:1;      // reply ok
    }b;
    unsigned char B[3];
}mSTAT;

typedef union{
    struct{
        unsigned    onoff:1;
        unsigned    level:7;

    }b;
    unsigned char B;
}audioSTAT;



typedef struct _ENCODER_SENSOR_
{
    int     BoardConnection;
    float   CurrentReference;
    float   CurrentPosition;
    float   CurrentVelocity;
}ENCODER_SENSOR;

typedef struct _FT_SENSOR_
{
    int     BoardConnection;
    float   Mx;
    float   My;
    float   Mz;
    float   Fx;
    float   Fy;
    float   Fz;
    float   RollVel;
    float   PitchVel;
    float   Roll;
    float   Pitch;
}FT_SENSOR;

typedef struct _IMU_SENSOR_
{
    int     BoardConnection;
    float   Roll;
    float   Pitch;
    float   Yaw;
    float   RollVel;
    float   PitchVel;
    float   YawVel;
    float   AccX;
    float   AccY;
    float   AccZ;

    float   Roll_offset;
    float   Pitch_offset;
}IMU_SENSOR;

typedef struct _OF_SENSOR_
{
    int     BoardConnection;
    int     AccumX[2];
    int     AccumY[2];
    int     DeltaX[2];
    int     DeltaY[2];
    float   Xmea;
    float   Ymea;
    float   thmea;
    float   Pitmea;
}OF_SENSOR;

typedef struct _SMART_POWER_
{
    int     BoardConnection;

    float   Voltage;
    float   Current;
}SMART_POWER;

typedef struct _FOG_SENSOR_
{
    float   Roll;
    float   Pitch;
    float   Yaw;
    float   RollVel;
    float   PitchVel;
    float   YawVel;
    int     Warning;
}FOG_SENSOR;

typedef struct _COMMAND_STRUCT_
{
    int     USER_COMMAND;
    char    USER_PARA_CHAR[MAX_COMMAND_DATA];
    int	    USER_PARA_INT[MAX_COMMAND_DATA];
    float   USER_PARA_FLOAT[MAX_COMMAND_DATA];
    double  USER_PARA_DOUBLE[MAX_COMMAND_DATA];
} COMMAND_STRUCT, *pCOMMAND_STRUCT;

typedef struct _USER_COMMAND_
{
    int             COMMAND_TARGET;
    COMMAND_STRUCT  COMMAND_DATA;
} USER_COMMAND, *pUSER_COMMAND;

//======================================
// Sensor Shared Memory <Read Only>
typedef struct _RBCORE_SHM_
{
    // reference -------------------------
    float           JointReference[MAX_AL][MAX_MC][MOTOR_2CH];
    float           JointRef[34];
    MANUAL_CAN      ManualCAN[MAX_MANUAL_CAN];


    // command ---------------------------
    int             SYNC_SIGNAL[MAX_AL];
    int             ACK_SIGNAL[MAX_MC][MOTOR_2CH];

    int             MotionOwner[MAX_MC][MOTOR_2CH];
    COMMAND_STRUCT  COMMAND[MAX_AL];


    // sensor ----------------------------
    char            PODO_AL_WORKING[MAX_AL];
    mSTAT           MCStatus[MAX_MC][MOTOR_2CH];
    float           MCTemperature[MAX_MC];
    float           MotorTemperature[MAX_MC][MOTOR_2CH];

    ENCODER_SENSOR  ENCODER[MAX_MC][MOTOR_2CH];
    FT_SENSOR       FT[MAX_FT];
    IMU_SENSOR      IMU[MAX_IMU];
    FOG_SENSOR      FOG;
    OF_SENSOR       OF;
    SMART_POWER     SP[MAX_SP];

    int             CAN_Enabled;
    int             REF_Enabled;
    int             SEN_Enabled;
    int             ENC_Enabled;
    int             EXF_R_Enabled;  // Extra Finger (R)
    int             EXF_L_Enabled;  // Extra Finger (L)
    float           EXF_R_Modifier[5];
    float           EXF_L_Modifier[5];

    int             Sim_Time_sec;
    int             Sim_Time_nsec;



    // State Machine Return Data ---------------
    int             STATE_COMMAND;
    char            STATE_PARA_CHAR[10];
    int             STATE_PARA_INT[10];
    float           STATE_PARA_FLOAT[10];


    float HANDLE_ANGLE;
    char ACC_STATE;
    audioSTAT   audioData;

    int             LaserDistanceData[MAX_LASER_DATA];
    int             LaserLength;
    float           CarRef[4];

    //VisionData
    VISION2MOTION V2M;
}RBCORE_SHM, *pRBCORE_SHM;
//======================================



typedef enum _COMMAND_SET_{
    NO_ACT = 0,
    // For process handle
    DAEMON_PROCESS_CREATE,
    DAEMON_PROCESS_KILL,
    // For initialize
    DAEMON_INIT_CHECK_DEVICE,
    DAEMON_INIT_FIND_HOME,
    DAEMON_INIT_FET_ONOFF,
    DAEMON_INIT_CONTROL_ONOFF,
    DAEMON_INIT_SET_FINGER_MODIFIER,
    // For attribute
    DAEMON_ATTR_SWITCHING_MODE,
    DAEMON_ATTR_FRICTION_COMPENSATION,
    DAEMON_ATTR_CONTROL_MODE,
    // For sensor
    DAEMON_SENSOR_ENCODER_RESET,
    DAEMON_SENSOR_ENCODER_ONOFF,
    DAEMON_SENSOR_SENSOR_ONOFF,
    DAEMON_SENSOR_FT_NULL,
    DAEMON_SENSOR_IMU_NULL,
    DAEMON_SENSOR_IMU_OFFSET_SET,
    DAEMON_SENSOR_FOG_NULL,
    DAEMON_SENSOR_FOG_USB_RESET,
    DAEMON_SENSOR_OF_NULL,
    DAEMON_SENSOR_OF_LAMP_ONOFF,
    // For motion
    DAEMON_MOTION_REF_ONOFF,
    DAEMON_MOTION_MOVE,
    DAEMON_MOTION_GAIN_OVERRIDE,
    DAEMON_MOTION_ERROR_CLEAR,
    // For CAN
    DAEMON_CAN_ENABLE_DISABLE,


    //
    POWER_CONTROL,

    // DRC SPECIFIC
    REQUEST_AL_NUM,
    SENSOR_OPT_NULL,
    SENSOR_OPT_LAMPON,
    SENSOR_OPT_LAMPOFF,
    FOG_USB_RESET
} COMMAND_SET;

#endif // RB_SHARED_MEMORY_H

