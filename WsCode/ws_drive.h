/*******************************************************************************
* FILE NAME: ws_drive.h
*
* DESCRIPTION:
*  This is the include file which corresponds to ws_drive.c
*
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __ws_drive_h_
#define __ws_drive_h_

/******************************* TYPEDEFS *************************************/

/******************************** MACROS **************************************/

/* if the crabs are turned outside the sprial limits, bring them back */
#define ENFORCE_SPIRAL_LIMITS(spiral_pos, min, max) \
  if (spiral_pos > max) \
    spiral_pos = max; \
  else if (spiral_pos < min) \
    spiral_pos = min; \

/***************************** DEFINITIONS ************************************/
#define CRAB_TURN_DISABLE_VAL_X  20
#define CRAB_TURN_DISABLE_VAL_Y  20
#define CRAB_DEAD_X 20
#define CRAB_DEAD_Y 20
#define CRAB_MANU_FB_RANGE 34
#define MONSTER_DRIVE_DEADZONE 4
#define TANK_DRIVE_DEADZONE 2
#define DRIVE_SCALE_NUM 1
#define DRIVE_SCALE_DENOM 2
#define ACCEL_RATE 35
#define ACCEL_RATE_TRACTION 2

#define BRAD_TO_JOY_MAX_X  (100)
#define BRAD_TO_JOY_MAX_Y  (100)


/* BRAKE_MODE_THRESHOLD is in encoder ticks per loop */
#define BRAKE_MODE_THRESHOLD 6

/* MAX_MONSTER_OFFSET is in brads */
#define MAX_MONSTER_OFFSET 18
#define MONSTER_SPEED_SCALE_NUM 1
#define MONSTER_SPEED_SCALE_DENOM 3

/* Dead pot check constants */
#define POT_CHECK_MOTOR_FULL     110
#define POT_CHECK_MOTOR_STOP      20
#define POT_CHECK_REV_MIN_DELTA  -15
#define POT_CHECK_REV_MAX_DELTA   -3
#define POT_CHECK_FWD_MIN_DELTA    3
#define POT_CHECK_FWD_MAX_DELTA   15
#define POT_CHECK_STOP_MIN_DELTA  -2
#define POT_CHECK_STOP_MAX_DELTA   2
#define POT_CHECK_BAD_INCREMENT    1
#define POT_CHECK_GOOD_DECREMENT   1
//#define POT_CHECK_BAD_THRESHOLD   30
#define POT_CHECK_BAD_THRESHOLD   60
#define POT_CHECK_BAD_LOOP_MAX  1000

typedef enum
{
  FEEDBACK_DISABLED = 0,
  FEEDBACK_ENABLED,
  FEEDBACK_POT_BROKEN
} FeedbackState;

typedef enum
{
  DRIVE_MODE_UNINIT = 0,
  DRIVE_MODE_TANK,
  DRIVE_MODE_CRAB,
  DRIVE_MODE_MONSTER,
  DRIVE_MODE_CAR
} DriveModeType;

typedef enum
{
  NOT_CROSSED = 0,
  ABOUT_TO_CROSS,
  CROSSED
} CrossStateType;

/****************************** STRUCTURES ************************************/
typedef struct
{
  FeedbackState state;
  UINT16 front_pot_tgt;
  UINT16 back_pot_tgt;
} CrabTgts;

typedef struct
{
  INT8   crab_speed_prev;
  UINT16 pot_prev;
  INT16  bad_loop_counter;
}PotCheckVarsType;

extern UINT8 g_use_differential_theta;
extern UINT8 g_use_forced_theta;
extern INT16 g_forced_theta;
/************************* FUNCTION PROTOTYPES ********************************/
extern void crab_init(void);
extern void drive_control(UINT8 use_deadzone);
extern DriveModeType drive_mode_select(UINT8 *first_loop);

extern void drive_mode_monster(UINT8 steer_back);
extern void drive_mode_crab(UINT8 first_loop);
extern void drive_mode_tank(UINT8 use_deadzone);
extern void drive_speed_calc(UINT8 x_axis, UINT8 y_axis);
extern void theta_correct(INT8 *front_diff, INT8 *back_diff,
                          UINT8 desired_brad);
void theta_correct_differential(INT8 *x_stick_diff);

extern void convert_brads_to_joystick(UINT8 brads, UINT8 *x, UINT8 *y);
extern UINT8 convert_joystick_to_brads(UINT8 x, UINT8 y);
extern UINT16 convert_brad_to_pot(UINT8 desired_pos_in_brad,
                                  CrabType *calibration, INT8 brad_offset);
INT16 apply_acceleration(INT16 speed, INT16 speed_prev);
extern void crab_pos_control(void);
extern void bad_pot_check(INT8 crab_speed, UINT16 pot_curr,
                          PotCheckVarsType *check_vars);
extern INT8 crab_feedback(UINT16 desired_crab_pos_pot,
                          PotDataType *cur_crab_pos,
                          CrabType *calibration,
                          PidValsType *pid_vals);
#endif /* __ws_drive_h_ */

