/*******************************************************************************
* FILE NAME: ws_autonomous.h
*
* DESCRIPTION:
*  This is the include file which corresponds to autonomous.c
*
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __ws_autonomous_h_
#define __ws_autonomous_h_

/******************************** MACROS **************************************/

/***************************** DEFINITIONS ************************************/

  /*
     1 - 220
     2 - 199
     3 - 175
     4 - 150
     5 - 131
     6 - 106
     7 - 82
     8 -  62
     9 - 39

  */


/* autonomous program selector knob */
#if PROTO
#define AUTO_PROG_OI_SEL0   0
#define AUTO_PROG_OI_SEL1   13
#define AUTO_PROG_OI_SEL2   36
#define AUTO_PROG_OI_SEL3   59
#define AUTO_PROG_OI_SEL4   83
#define AUTO_PROG_OI_SEL5   106
#define AUTO_PROG_OI_SEL6   129
#define AUTO_PROG_OI_SEL7   152
#define AUTO_PROG_OI_SEL8   175
#define AUTO_PROG_OI_SEL9   200
#define AUTO_PROG_OI_SEL10  223
#define AUTO_PROG_OI_DIFF     5
#else
#define AUTO_PROG_OI_SEL0 220
#define AUTO_PROG_OI_SEL1 199
#define AUTO_PROG_OI_SEL2 178
#define AUTO_PROG_OI_SEL3 153
#define AUTO_PROG_OI_SEL4 131
#define AUTO_PROG_OI_SEL5 106
#define AUTO_PROG_OI_SEL6 82
#define AUTO_PROG_OI_SEL7  62
#define AUTO_PROG_OI_SEL8 39
#define AUTO_PROG_OI_DIFF     7

#endif

/* autonomous position selector */
#define AUTO_POS_OI_LEFT      5
#define AUTO_POS_OI_CENTER  127
#define AUTO_POS_OI_RIGHT   250
#define AUTO_POS_OI_DIFF      5

/* autonomous delay selector */
#define AUTO_DELAY_OI_0       5
#define AUTO_DELAY_OI_1     127
#define AUTO_DELAY_OI_2     250
#define AUTO_DELAY_OI_DIFF    5

/* Constants to determine program state */
#define AUTO_PROGRAM_NOT_DONE  0
#define AUTO_PROGRAM_DONE      1

/* Constant for the size of auto programs */
#define MAX_AUTO_PROGRAMS  1

#define AUTO_LED_BLINK_END 8
#define AUTO_LED_BLINK_DELAY_END (AUTO_LED_BLINK_END + 28)

/****************************** STRUCTURES ************************************/

/*******************************************
2008 Autonomous Constants and Data Types
*******************************************/
typedef enum
{
  BLINK_ON,
  BLINK_OFF,
  BLINK_DELAY
} LEDBlinkStates;

typedef enum
{
  AUTO_OFF = 0,
  AUTO_ON
} AutoStateType;

typedef enum
{
  TASK_STATE_PROCESSING = 0,
  TASK_STATE_DONE
} AutoTaskState;

typedef enum
{
  COLOR_RED = 0,
  COLOR_BLUE
} ColorType;

typedef enum
{
  BALL_POS_NONE = 0,
  BALL_POS_LEFT,
  BALL_POS_CENTER,
  BALL_POS_RIGHT
} BallPosType;

typedef enum
{
  START_POS_LEFT = 0,
  START_POS_CENTER,
  START_POS_RIGHT
} StartPosType;

typedef enum
{
  DELAY_0 = 0,
  DELAY_1,
  DELAY_2
} DelayType;

/************************* FUNCTION PROTOTYPES ********************************/

extern void autonomous_init(void);
extern void auto_lock_in(void);
extern void auto_main(void);
extern void auto_chooser(void);
extern void auto_output_off(void);
extern void display_auto_data(void);
extern UINT8 auto_run_program(UINT8 s_auto_locked_in, UINT8 s_prog_num);

extern ColorType g_auto_color;
extern DelayType g_auto_delay;
extern StartPosType g_start_pos;
extern BallPosType g_ball_pos;

#endif /* __ws_autonomous_h_ */


