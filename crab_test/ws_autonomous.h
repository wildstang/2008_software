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

#define FAKE_AUTON 0
#define PRINT_TASK_INFO 1

/******************************** MACROS **************************************/

/***************************** DEFINITIONS ************************************/


/* autonomous program selector knob */
#ifdef REAL_ROBOT
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
#define _SW_LEFT   0
#define _SW_MID    127
#define _SW_RIGHT  250
#define _SW_UNUSED 200
#define AUTO_PROG_OI_SEL0   _SW_LEFT
#define AUTO_PROG_OI_SEL1   _SW_UNUSED
#define AUTO_PROG_OI_SEL2   _SW_UNUSED
#define AUTO_PROG_OI_SEL3   _SW_UNUSED
#define AUTO_PROG_OI_SEL4   _SW_UNUSED
#define AUTO_PROG_OI_SEL5   _SW_UNUSED
#define AUTO_PROG_OI_SEL6   _SW_MID
#define AUTO_PROG_OI_SEL7   _SW_RIGHT
#define AUTO_PROG_OI_SEL8   _SW_UNUSED
#define AUTO_PROG_OI_SEL9   _SW_UNUSED
#define AUTO_PROG_OI_SEL10  _SW_UNUSED
#define AUTO_PROG_OI_DIFF     4
#endif

/* Constants to determine program state */
#define AUTO_PROGRAM_NOT_DONE  0
#define AUTO_PROGRAM_DONE      1

/* Constant for the size of auto programs */
#define MAX_AUTO_PROGRAMS  1

#define AUTO_LED_BLINK_END 8
#define AUTO_LED_BLINK_DELAY_END (AUTO_LED_BLINK_END + 28)

/****************************** STRUCTURES ************************************/

/*******************************************
2007 Autonomous Constants and Data Types
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

/************************* FUNCTION PROTOTYPES ********************************/

extern void autonomous_init(void);
extern void auto_lock_in(void);
extern void auto_main(void);
extern void auto_chooser(void);
extern void auto_output_off(void);
extern void display_auto_data(void);
extern UINT8 auto_run_program(UINT8, UINT8);
extern UINT8 ap_sleeper(void);
extern UINT8 ap_grabber(void);
extern UINT8 ap_hk_squirrel(void);
extern UINT8 ap_sweeper_left(void);
extern UINT8 ap_sweeper_right(void);
extern UINT8 ap_sonar_cw(void);
extern UINT8 ap_sonar_ccw(void);
extern UINT8 ap_sonar_side_cw(void);
extern UINT8 ap_sonar_side_ccw(void);
extern UINT8 ap_sonar_stm(SonarStmParamType);

#endif /* __ws_autonomous_h_ */

