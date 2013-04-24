/*******************************************************************************
* FILE NAME: ws_lift.h
*
* DESCRIPTION:
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __ws_lift_h_
#define __ws_lift_h_

/******************************* TYPEDEFS *************************************/

/******************************** MACROS **************************************/

/***************************** DEFINITIONS ************************************/

#define ROLLER_REV              (0 + HAT_RANGE)
#define ROLLER_OFF              64
#define ROLLER_NOTHING          127
#define ROLLER_REV_SLOW         192
#define ROLLER_FWD              (255 - HAT_RANGE)

#define SLAPPER_THRESH          200

#define ROLLER_SPEED_OFF        0
#define ROLLER_SPEED_FWD_FAST   127
#define ROLLER_SPEED_FWD_SLOW   20
#define ROLLER_SPEED_REV        -127
#define ROLLER_SPEED_REV_SLOW   -30

#ifdef PROTO
#define ACCUM_UNFOLD_LOOPS  60
#else
#define ACCUM_UNFOLD_LOOPS  25
#endif

typedef enum
{
  ROLLER_BUTTON_OFF = 0,
  ROLLER_BUTTON_FWD,
  ROLLER_BUTTON_REV,
  ROLLER_BUTTON_NOTHING,
  ROLLER_BUTTON_REV_SLOW
} RollerButtonType;

typedef enum
{
  ROLLER_STATE_OFF = 0,
  ROLLER_STATE_FWD_SLOW,
  ROLLER_STATE_FWD_FAST,
  ROLLER_STATE_REV,
  ROLLER_STATE_REV_SLOW
} RollerStateType;

typedef enum
{
  LIFT_STATE_UNKNOWN = 0,
  LIFT_STATE_HOME,
  LIFT_STATE_GATHER,
  LIFT_STATE_READY_HURDLE,
  LIFT_STATE_HURDLE,
  LIFT_STATE_HURDLE_LOWER,
  LIFT_STATE_DRIVE_BY,
  LIFT_STATE_DRIVE_BY_LOWER_1,
  LIFT_STATE_DRIVE_BY_LOWER_2,
  LIFT_STATE_LOWER_TO_GATHER,
  LIFT_STATE_GATHER_TO_HURDLE
} LiftStateType;

typedef enum
{
  MANU_LIFT_STATE_DOWN = 0,
  MANU_LIFT_STATE_MID1,
  MANU_LIFT_STATE_MID2,
  MANU_LIFT_STATE_UP
} ManuLiftStateType;

/****************************** STRUCTURES ************************************/

/************************* FUNCTION PROTOTYPES ********************************/
extern void lift_init(void);
extern void lift_reset(void);
extern void lift_control(void);
extern void lift_control_manual(void);
extern void lift_control_auto(void);
extern void roller_control(void);
extern void slapper_control(void);
extern void unfold_accumulator(UINT8 reinit);

#endif /* __ws_lift_h_ */


