/*******************************************************************************
* FILE NAME: ws_includes.h
*
* DESCRIPTION:
*  General structures & enumerations
*
*******************************************************************************/

#ifndef __ws_includes_h_
#define __ws_includes_h_

/**************************** DEBUG DEFINES ***********************************/

/*
 * The following defines run in the slow loop
 */
#define NEWLINE_IN_TELEOP      1
#define NEWLINE_IN_DISABLED    0
#define NEWLINE_IN_AUTON       1

/* print raw joystick values on drive stick */
#define JOYSTICK_PRINTS        0
/* debugs related to drive inputs & outputs */
#define DRIVE_PRINTS           0
/* debugs theta correction */
#define THETA_PRINTS           0
/* print the data coming back from the CC */
#define PRINT_CC_DATA          0
/* print CC communication errors */
#define PRINT_CC_ERRORS        1
/* debugs for lift state machine */
#define DEBUG_LIFT_STATES      0
/* debugs for roller state machine */
#define DEBUG_ROLLER_PRINTS    0
/* debugs for autonomous lockin  */
#define DEBUG_AUTO_LOCKIN      0

/**** The following debugs run in the fast loop ****/
#define NEWLINE_IN_TELEOP_SPIN   0
#define NEWLINE_IN_DISABLED_SPIN 0
#define NEWLINE_IN_AUTON_SPIN    0

/* debugs related to crab feedback */
#define CRAB_FEEDBACK_PRINTS     0
/* print ADC results */
#define ADC_RESULT_PRINTS        0
#define AUTON_ADC_RESULT_PRINTS  0
/* debug pot rollover */
#define DEBUG_POT_ROLLOVER       0
/* debugs for dead pot check */
#define DEBUGGING_POT_CHECK      0


/******************************* TYPEDEFS *************************************/

typedef enum
{
  CC_BALL_POS_UNKNOWN = 0,
  CC_BALL_POS_BLUE_RED_EMPTY,
  CC_BALL_POS_BLUE_EMPTY_RED,
  CC_BALL_POS_EMPTY_BLUE_RED,
  CC_BALL_POS_RED_BLUE_EMPTY,
  CC_BALL_POS_RED_EMPTY_BLUE,
  CC_BALL_POS_EMPTY_RED_BLUE,
  CC_BALL_POS_LEFT,
  CC_BALL_POS_CENTER,
  CC_BALL_POS_RIGHT
} BallPosType;

typedef enum
{
  LED_ON,
  LED_OFF,
  LED_UNCH
} LedState;

typedef enum
{
  RELAY_OFF = 0,
  RELAY_FWD,
  RELAY_REV
} RelayValsType;

typedef enum
{
  PUMP_OFF = 0,
  PUMP_ON
} PumpRunType;

typedef enum brake_mode_t_
{
  BRAKE_MODE_ON = 0,
  BRAKE_MODE_OFF
} BrakeModeType;

typedef enum
{
  ACCUM_TILT_IN = 0,
  ACCUM_TILT_OUT
} AccumTiltType;

typedef enum
{
  NEST_TILT_IN = 0,
  NEST_TILT_OUT
} NestTiltType;

typedef enum
{
  SLAPPER_DOWN = 0,
  SLAPPER_UP
} SlapperType;

typedef enum
{
  KICKER_IN = 0,
  KICKER_OUT
} KickerType;

typedef enum
{
  LANDING_GEAR_UP = 0,
  LANDING_GEAR_DOWN
} LandingGearType;

typedef enum
{
  LIFT_NONE = 0,
  LIFT_DOWN,
  LIFT_UP
} LiftType;

typedef enum
{
  NEST_LEVEL_NONE = 0,
  NEST_LEVEL_DOWN,
  NEST_LEVEL_UP
} NestLevelType;

typedef enum
{
  LADDER_NONE = 0,
  LADDER_DOWN,
  LADDER_UP
} LadderType;

typedef enum
{
  CALIBRATION_UNKNOWN = -1,
  CALIBRATION_GOOD = 0, /* the code requires that this value is 0 */
  CALIBRATION_BAD_RC_DATA,
  CALIBRATION_BAD_CC_DATA
} CalibrationState;

/* type of data requested/returned from CC */
typedef enum
{
  CC_REQ_UNINIT = 0,
  CC_REQ_ENCODER,
  CC_REQ_EEPROM
} CcReqDataType;


/******************************** MACROS **************************************/

#define MIN_MAX(variable,min_val,max_val)  MIN((variable),(min_val)); MAX((variable),(max_val))

#define MAX(variable, max_val)  if (variable > (max_val)) variable = (max_val)
#define MIN(variable, min_val)  if (variable < (min_val)) variable = (min_val)

#define MAX_RETURN(value, max_val) ((value) > (max_val) ? (max_val) : (value))
#define MIN_RETURN(value, min_val) ((value) < (min_val) ? (min_val) : (value))

#define DEADZONE(var, zone)  if ((var >= (127 - (zone))) && \
                                 (var <= (127 + (zone)))) \
                               var = 127

#define OUTSIDE_DEADZONE(val, zone)  (((val) > (127 + (zone))) || \
                                      ((val) < (127 - (zone))))


#define HAT_RANGE_CHECK(hat, value) \
   ((((hat) >= ((int)(value) - HAT_RANGE)) && \
     ((hat) <= ((int)(value) + HAT_RANGE))) ? \
    TRUE : FALSE)

#define ABS(var) (((var) < 0) ? (0 - (var)) : (var))

/***************************** DEFINITIONS ************************************/

/* Use 270deg blue pot instead of digital pot */
#define USE_DIGITAL_POT 1

#define SUCCESS 1
#define FAIL    0

#define ROBOT_ENABLED  0
#define ROBOT_DISABLED 1

#define AUTO_ENABLED   1
#define AUTO_DISABLED  0

#define NUM_PACKETS_PER_SEC 40

#define HAT_RANGE    10

#define HAT_NONE     127
#define HAT_UP       HAT_RANGE
#define HAT_DOWN     254-HAT_RANGE
#define HAT_LEFT     185
#define HAT_RIGHT    65

/* initial spiral position on robot startup until the multi-turn pot is
   validated */
#define LIMIT_CRAB_90 1

#if LIMIT_CRAB_90
#define SPIRAL_POS_MIN  4
#define SPIRAL_POS_MAX  4
#define SPIRAL_POS_INIT 4
#else
#define SPIRAL_POS_MIN  1
#define SPIRAL_POS_MAX  7
#define SPIRAL_POS_INIT 4
#endif

#define NUM_EEPROM_BYTES 15

/****************************** STRUCTURES ************************************/
typedef struct motor_vals_t_
{
  INT8                  left_drive;
  INT8                  right_drive;
  INT8                  back_crab;
  INT8                  front_crab;
  INT8                  roller;
  LiftType              lift_1;
  LiftType              lift_2;
  LadderType            ladder;
  NestLevelType         nest_level;
  NestTiltType          nest_tilt;
  AccumTiltType         accum_tilt;
  SlapperType           slapper;
  KickerType            kicker;
  LandingGearType       landing_gear;
  PumpRunType           pump;
  BrakeModeType         brake_mode;
} MotorValsType;

typedef struct
{
  INT8 left;
  INT8 right;
  INT16 orient;
  BallPosType ball_near_pos;
  BallPosType ball_far_pos;
} EncoderValsType;

typedef struct
{
  CcReqDataType type;
  union
  {
    EncoderValsType encoder;
    UINT8           eeprom[NUM_EEPROM_BYTES];
  } data;
} CcDataType;

typedef struct crab_t_
{
  UINT16  pot_mid;
#if USE_DIGITAL_POT
#else
  UINT16  pot_left;
  UINT16  pot_right;
#endif
} CrabType;

typedef struct calibration_vals_t_
{
  CalibrationState state;
  CrabType front_crab;
  CrabType back_crab;
} CalibrationValsType;

typedef struct
{
#if USE_DIGITAL_POT
  UINT8  is_init;      /* A flag to let us know if we're initialized or not */
  UINT8  spiral_count; /* Ring level of current value */
  UINT16 raw_val;      /* Raw value of pos pot, 0-POT_RES */
  UINT16 raw_val_prev; /* Previous Raw Value */
#endif
  UINT16 abs_val;      /* (spiral_count * POT_RES) + raw_val */
} PotDataType;

typedef struct pot_vals_t_
{
  PotDataType  front_crab;
  PotDataType  back_crab;
} PotValsType;


extern MotorValsType motor_vals;
extern CalibrationValsType calibration_vals;
extern PotValsType pot_vals;
extern CcDataType cc_data;
extern UINT8 rc_eeprom[];
extern UINT8 rc_eeprom_dirty;

/************************* FUNCTION PROTOTYPES ********************************/

#endif /* __ws_includes_h_ */

