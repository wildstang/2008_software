/*******************************************************************************
* FILE NAME: ws_includes.h
*
* DESCRIPTION:
*  General structures & enumerations
*
*******************************************************************************/

#ifndef __ws_includes_h_
#define __ws_includes_h_

/******************************* TYPEDEFS *************************************/

typedef enum
{
  LED_ON,
  LED_OFF,
  LED_UNCH
} LedState;

typedef enum
{
  FDBK_ENABLED,
  FDBK_DISABLED
} FeedbackState;

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

typedef enum
{
  CRAB_ARC_NONE = 0,
  CRAB_ARC_CW,
  CRAB_ARC_CCW
} CrabArcType;

typedef enum gripper_state_t_
{
  GRIPPER_CLOSED,
  GRIPPER_OPEN,
} GripperStateType;

typedef enum telescope_state_t_
{
  TELESCOPE_IN,
  TELESCOPE_OUT,
} TelescopeStateType;

typedef enum constraint_t_
{
  CONSTRAINT_ALLOW_NONE,
  CONSTRAINT_ALLOW_REV,
  CONSTRAINT_ALLOW_FWD,
  CONSTRAINT_ALLOW_ALL,
  CONSTRAINT_DISALLOW = CONSTRAINT_ALLOW_NONE,
  CONSTRAINT_ALLOW = CONSTRAINT_ALLOW_ALL
} ConstraintType;

typedef enum landing_gear
{
  LANDING_GEAR_UP,
  LANDING_GEAR_DOWN,
} LandingGearStateType;

typedef enum brake_mode_t_
{
  BRAKE_MODE_ON,
  BRAKE_MODE_OFF,
} BrakeModeType;

typedef enum ramp_lock_type_t_
{
  RAMP_UNLOCKED,
  RAMP_LOCKED,
} RampLockType;

typedef enum ramp_release_type_t_
{
  RAMP_NO_RELEASE,
  RAMP_DEPLOY,
} RampReleaseType;

typedef enum tower_release_type_t_
{
  TOWER_NO_RELEASE,
  TOWER_RELEASE,
} TowerReleaseType;

typedef enum
{
  SONAR_STM_RET_NOT_DONE,
  SONAR_STM_RET_DONE_SUCCESS,
  SONAR_STM_RET_DONE_FAIL
} SonarStmRetValType;


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


#define GET_ANALOG_VALUE_SHIFT(a) (Get_Analog_Value(a) >> 2)

#define HAT_RANGE_CHECK(hat, value) \
   ((((hat) >= ((int)(value) - HAT_RANGE)) && \
     ((hat) <= ((int)(value) + HAT_RANGE))) ? \
    TRUE : FALSE)

/***************************** DEFINITIONS ************************************/

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

/****************************** STRUCTURES ************************************/
typedef struct motor_vals_t_
{
  INT8                  left_drive;
  INT8                  right_drive;
  INT8                  back_crab;
  INT8                  front_crab;
  INT8                  enable_crab_arc;
  INT8                  shoulder;
  INT8                  rotate;
  INT8                  elbow;
  GripperStateType      gripper_top;
  GripperStateType      gripper_bottom;
  TelescopeStateType    telescope;
  PumpRunType           pump;
  LandingGearStateType  landing_gear;
  BrakeModeType         brake_mode;
  RampLockType          ramp_lock;
  RampReleaseType       ramp_release;
  TowerReleaseType      tower_release;
} MotorValsType;

typedef struct
{
  INT8 left_front;
  INT8 left_back;
  INT8 right_front;
  INT8 right_back;
  INT16 orient;
  UINT16 sonar;
} EncoderValsType;

typedef struct crab_t_
{
  UINT8    left;
  UINT8    mid;
  UINT8    right;
} CrabType;

typedef struct calibration_vals_t_
{
  CrabType front_crab;
  CrabType back_crab;
  UINT16   shoulder_front;
  UINT16   shoulder_horizontal;
  UINT16   shoulder_back;
  UINT16   elbow_top;
  UINT16   elbow_horizontal;
  UINT16   elbow_bottom;
  UINT16   rotate_left;
  UINT16   rotate_horizontal;
  UINT16   rotate_right;
} CalibrationValsType;

typedef struct arm_position_t_
{
  UINT8 shoulder;
  UINT8 elbow;
  UINT8 rotate;
  TelescopeStateType telescope;
  GripperStateType gripper_top;
  GripperStateType gripper_bottom;
  UINT8    feedback_enabled;
  UINT8    mask;
} ArmPosType;

typedef struct arm_constraint_t_
{
  ConstraintType shoulder;
  ConstraintType elbow;
  ConstraintType rotate;
  ConstraintType telescope;
} ArmConstraintType;

typedef struct _sonar_stm_param
{
  UINT8 use_theta_correct;

  UINT8 raise_arm_count_init;

  UINT16 drive_to_rack_dist_count;
  UINT8  drive_to_rack_crab_joystick;
  UINT8  drive_to_rack_speed;
  UINT16 drive_to_rack_no_sonar_dist_count;
  UINT16 drive_to_rack_timeout_loop_count;

  UINT8  pause_at_rack_loop_count;

  UINT16 crab_at_rack_loop_count;

  UINT8  find_spider_speed;
  UINT8  find_spider_timeout_loop_count;

  UINT8  found_pause_loop_count;

  UINT8  drive_to_spider_speed;
  UINT8  drive_to_spider_timeout_loop_count;

  UINT8  blind_drive_dist;
  UINT8  blind_drive_speed;

  UINT8  pause_at_spider_loop_count;

  CrabArcType arc_direction;
} SonarStmParamType;

extern MotorValsType motor_vals;
extern CalibrationValsType calibration_vals;
extern EncoderValsType g_encoder_vals;
extern UINT8 g_cc_encoder_ret_val;
extern INT16 g_elbow_speed;
extern UINT8 g_crab_arc_ccw;

/************************* FUNCTION PROTOTYPES ********************************/

#endif /* __ws_includes_h_ */

