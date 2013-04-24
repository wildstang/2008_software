#ifndef __ws_autoprog_h_
#define __ws_autoprog_h_

#define TO_NONE                    ((UINT16) -1)
#define GYRO_180_DIFF              1630

typedef enum
{
  AP_SLPR_STATE_WAIT_FOR_BALL_POS,
  AP_SLPR_STATE_INIT_FROM_BALL_POS,
  AP_SLPR_STATE_DELAY,
  AP_SLPR_STATE_POINT_TO_APPROACH,
  AP_SLPR_STATE_DRIVE_TO_APPROACH,
  AP_SLPR_STATE_POINT_TO_ATTACK,
  AP_SLPR_STATE_DRIVE_TO_ATTACK,
  AP_SLPR_STATE_POINT_TO_ATTACK_DONE,
  AP_SLPR_STATE_DRIVE_TO_ATTACK_DONE,
  AP_SLPR_STATE_POINT_TO_CLEAR_WALL,
  AP_SLPR_STATE_DRIVE_TO_CLEAR_WALL,
  AP_SLPR_STATE_POINT_TO_PREP_FOR_LINE2,
  AP_SLPR_STATE_DRIVE_TO_PREP_FOR_LINE2,
  AP_SLPR_STATE_DRIVE_TO_LINE2,
  AP_SLPR_STATE_DRIVE_SHORT_LINE2,
  AP_SLPR_STATE_DRIVE_TO_FINISH_LINE2,
  AP_SLPR_STATE_DRIVE_TO_LINE3,
  AP_SLPR_STATE_DRIVE_TO_FINISH_LINE3,
  AP_SLPR_STATE_DRIVE_TO_LINE4,
  AP_SLPR_STATE_DRIVE_SHORT_LINE4,
  AP_SLPR_STATE_DRIVE_TO_FINISH_LINE4,
  AP_SLPR_STATE_DONE
} ApSlprState;

typedef enum
{
  AP_LPR_STATE_DELAY,
  AP_LPR_STATE_DRIVE_OUT,
  AP_LPR_STATE_START_TURN,
  AP_LPR_STATE_DRIVE_SHORT_SIDE,
  AP_LPR_STATE_FINISH_TURN,
  AP_LPR_STATE_DRIVE_AFTER_TURN,
  AP_LPR_STATE_DONE
} ApLprState;


typedef enum
{
  AUTON_DRIVE_DRIVING = 0,
  AUTON_DRIVE_DONE
} AutonDriveType;


AutonDriveType drive_by_ticks(UINT8 drive_x, UINT8 drive_y, INT16 des_ticks);
AutonDriveType drive_by_gyro(UINT8 drive_x, UINT8 drive_y, INT16 des_gyro_delta);

extern UINT8 ap_sleeper(void);
extern UINT8 ap_prog1(void);
extern UINT8 ap_prog2(void);
extern UINT8 ap_prog3(UINT8 num_lines, UINT8 allow_slap, UINT8 random_slap);
extern UINT8 ap_prog4(UINT8 num_lines, UINT8 allow_slap, UINT8 random_slap);

#endif /* __ws_autonomous_prog_h_ */
