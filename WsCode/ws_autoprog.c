#include <stdio.h>
#include "ifi_frc.h"
#include "ws_includes.h"
#include "ws_io.h"
#include "ws_autonomous.h"
#include "ws_autoprog.h"


UINT8 auto_run_program(UINT8 auto_lock, UINT8 prog)
{
  UINT8 ret = AUTO_PROGRAM_DONE;

  /* If we're not locked in, go to the default case
   * of do nothing */
  if(auto_lock == 0)
  {
    printf("NOT LOCKED IN - DO NOTHING ");
    prog = 0;
  }

  printf("PROG %d ", prog);
  switch(prog)
  {
    case 1:
      ret = ap_sleeper();
      break;
    case 2:
      ret = ap_prog2();
      break;
    case 3:
      ret = ap_prog3(4, 0, 0);
      break;
    case 4:
      ret = ap_prog3(4, 1, 0);
      break;
    case 5:
      ret = ap_prog3(4, 1, 1);
      break;
    case 6:
      ret = ap_sleeper();
      break;
    case 7:
      ret = ap_sleeper();
      break;
    case 8:
      ret = ap_sleeper();
      break;
    default:
      ret = ap_sleeper();
      break;
  }
  return ret;
}

AutonDriveType drive_by_ticks(UINT8 drive_x, UINT8 drive_y, INT16 des_ticks)
{
  static INT16 tick_count = 0;
  AutonDriveType ret = AUTON_DRIVE_DRIVING;

  tick_count += cc_data.data.encoder.left;

  printf("des %4d cur %4d ", des_ticks, tick_count);
  if (((des_ticks >= 0) && (tick_count >= des_ticks)) ||
      ((des_ticks < 0)  && (tick_count <= des_ticks)))
  {
    tick_count = 0;
    Oi_drive_x = 127;
    Oi_drive_y = 127;

    ret = AUTON_DRIVE_DONE;
  }
  else
  {
    Oi_drive_x = drive_x;
    Oi_drive_y = drive_y;
  }
  return ret;
}

AutonDriveType drive_by_gyro(UINT8 drive_x, UINT8 drive_y,
                             INT16 des_gyro_delta)
{
  static INT16 initial_gyro;
  static UINT16 loop_count = 0;
  AutonDriveType ret = AUTON_DRIVE_DRIVING;

  if (loop_count == 0)
  {
    initial_gyro = cc_data.data.encoder.orient;
    printf("INIT GYRO %d ", initial_gyro);
  }

  printf("des %d cur %d", des_gyro_delta,
         cc_data.data.encoder.orient - initial_gyro);
  if (((des_gyro_delta >= 0) &&
       ((cc_data.data.encoder.orient - initial_gyro) >= des_gyro_delta)) ||
      ((des_gyro_delta < 0) &&
       ((cc_data.data.encoder.orient - initial_gyro) < des_gyro_delta)))
  {
    loop_count = 0;
    Oi_drive_x = 127;
    Oi_drive_y = 127;

    ret = AUTON_DRIVE_DONE;
  }
  else
  {
    Oi_drive_x = drive_x;
    Oi_drive_y = drive_y;
    loop_count++;
  }
}

