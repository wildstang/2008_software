/*******************************************************************************
* FILE NAME: ws_io.h
*
* DESCRIPTION:
*  This is the include file which corresponds to io_code.c
*
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __ws_io_h_
#define __ws_io_h_


/******************************* TYPEDEFS *************************************/

typedef enum
{
  DISPLAY_DATA_PSI,
  DISPLAY_DATA_AUTO,
  DISPLAY_DATA_CALIBRATE,
  DISPLAY_DATA_OVERRIDE,
  DISPLAY_DATA_TURRET
} DisplayDataType;

/******************************** MACROS **************************************/
#define SOLENOID_CONTROL_2WAY(mv, relay_rev, relay_fwd, state_off, state_rev, state_fwd) \
  if ((mv) == (state_off)) \
  { \
    relay_rev = 0; \
    relay_fwd = 0; \
  } \
  else if ((mv) == (state_rev)) \
  { \
    relay_rev = 1; \
    relay_fwd = 0; \
  } \
  else if ((mv) == (state_fwd)) \
  { \
    relay_rev = 0; \
    relay_fwd = 1; \
  }


/***************************** DEFINITIONS ************************************/
#define OI_CALIBRATE_ENCODERS    50
#define OI_CALIBRATE_JOYSTICKS   200

#define PRESSURE_BELOW_120        0
#define PRESSURE_ABOVE_120        1

/**************************************************************
 * Inputs
 **************************************************************/

/***** RC Analog Inputs *****/
/* As analog inputs are added, NUM_ADC_CHANNELS in adc.h must be changed */
#define Analog_in_front_crab         1
#define Analog_in_back_crab          2
#define Analog_in_front_wind         3
#define Analog_in_back_wind          4
#define Analog_in_05                 5
#define Analog_in_06                 6
#define Analog_in_07                 7
#define Analog_in_08                 8
#define Analog_in_09                 9
#define Analog_in_10                 10
#define Analog_in_11                 11
#define Analog_in_12                 12
#define Analog_in_13                 13
#define Analog_in_14                 14
#define Analog_in_15                 15
#define Analog_in_16                 16


/* The digital inputs / outputs are the same pins, a pin can either be an
   output or an input, not both */
/***** RC Digital Inputs *****/
#define Dig_in_pressure           rc_dig_in01
#define Dig_in_02                 rc_dig_in02
#define Dig_in_03                 rc_dig_in03
#define Dig_in_04                 rc_dig_in04
#define Dig_in_05                 rc_dig_in05
#define Dig_in_06                 rc_dig_in06
#define Dig_in_07                 rc_dig_in07
#define Dig_in_08                 rc_dig_in08
#define Dig_in_09                 rc_dig_in09
#define Dig_in_10                 rc_dig_in10
#define Dig_in_11                 rc_dig_in11
#define Dig_in_12                 rc_dig_in12
#define Dig_in_13                 rc_dig_in13
#define Dig_in_14                 rc_dig_in14
#define Dig_in_15                 rc_dig_in15
#define Dig_in_16                 rc_dig_in16
#define Dig_in_17                 rc_dig_in17
#define Dig_in_18                 rc_dig_in18

/***** RC Digital Outputs *****/
#define Dig_out_01                rc_dig_out01
#define Dig_out_brake_mode        rc_dig_out02
#define Dig_out_03                rc_dig_out03
#define Dig_out_04                rc_dig_out04
#define Dig_out_05                rc_dig_out05
#define Dig_out_06                rc_dig_out06
#define Dig_out_07                rc_dig_out07
#define Dig_out_08                rc_dig_out08
#define Dig_out_09                rc_dig_out09
#define Dig_out_10                rc_dig_out10
#define Dig_out_11                rc_dig_out11
#define Dig_out_12                rc_dig_out12
#define Dig_out_13                rc_dig_out13
#define Dig_out_14                rc_dig_out14
#define Dig_out_15                rc_dig_out15
#define Dig_out_16                rc_dig_out16
#define Dig_out_17                rc_dig_out17
#define Dig_out_18                rc_dig_out18


/***** PS2 aliases for multiplexed buttons *****/
#define Oi_ps2_x_no_shift        (!p2_sw_trig && !p2_sw_top && \
                                  !p2_sw_aux1 && p2_sw_aux2)
#define Oi_ps2_square_no_shift   (!p2_sw_trig && !p2_sw_top && \
                                  p2_sw_aux1 && !p2_sw_aux2)
#define Oi_ps2_triangle_no_shift (!p2_sw_trig && !p2_sw_top && \
                                  p2_sw_aux1 && p2_sw_aux2)
#define Oi_ps2_circle_no_shift   (!p2_sw_trig && p2_sw_top && \
                                  !p2_sw_aux1 && !p2_sw_aux2)

/* Shift functionality triggered by L1 */
#define Oi_ps2_x_shift_l         (p2_sw_trig && !p2_sw_top && \
                                  !p2_sw_aux1 && !p2_sw_aux2)
#define Oi_ps2_square_shift_l    (p2_sw_trig && !p2_sw_top && \
                                  !p2_sw_aux1 && p2_sw_aux2)
#define Oi_ps2_triangle_shift_l  (p2_sw_trig && !p2_sw_top && \
                                  p2_sw_aux1 && !p2_sw_aux2)
#define Oi_ps2_circle_shift_l    (p2_sw_trig && !p2_sw_top && \
                                  p2_sw_aux1 && p2_sw_aux2)

/* Shift functionality triggered by R1 */
#define Oi_ps2_x_shift_r         (p2_sw_trig && p2_sw_top && \
                                  !p2_sw_aux1 && !p2_sw_aux2)
#define Oi_ps2_square_shift_r    (p2_sw_trig && p2_sw_top && \
                                  !p2_sw_aux1 && p2_sw_aux2)
#define Oi_ps2_triangle_shift_r  (p2_sw_trig && p2_sw_top && \
                                  p2_sw_aux1 && !p2_sw_aux2)
#define Oi_ps2_circle_shift_r    (p2_sw_trig && p2_sw_top && \
                                  p2_sw_aux1 && p2_sw_aux2)

/* Macros to get the shifted state */
#define Oi_ps2_is_shifted        (p2_sw_trig)
#define Oi_ps2_is_shifted_l      (p2_sw_trig && !p2_sw_top)
#define Oi_ps2_is_shifted_r      (p2_sw_trig && p2_sw_top)

#define SET_Oi_ps2_x_no_shift()                 \
  do {p2_sw_trig = 0; p2_sw_top = 0; p2_sw_aux1 = 0; p2_sw_aux2 = 1;} while(0)
#define SET_Oi_ps2_square_no_shift()   \
  do {p2_sw_trig = 0; p2_sw_top = 0; p2_sw_aux1 = 1; p2_sw_aux2 = 0;} while(0)
#define SET_Oi_ps2_triangle_no_shift() \
  do {p2_sw_trig = 0; p2_sw_top = 0; p2_sw_aux1 = 1; p2_sw_aux2 = 1;} while(0)
#define SET_Oi_ps2_circle_no_shift()   \
  do {p2_sw_trig = 0; p2_sw_top = 1; p2_sw_aux1 = 0; p2_sw_aux2 = 0;} while(0)

#define SET_Oi_ps2_x_shift_l()      \
  do {p2_sw_trig = 1; p2_sw_top = 0; p2_sw_aux1 = 0; p2_sw_aux2 = 0;} while(0)
#define SET_Oi_ps2_square_shift_l()      \
  do {p2_sw_trig = 1; p2_sw_top = 0; p2_sw_aux1 = 0; p2_sw_aux2 = 1;} while(0)
#define SET_Oi_ps2_triangle_shift_l()    \
  do {p2_sw_trig = 1; p2_sw_top = 0; p2_sw_aux1 = 1; p2_sw_aux2 = 0;} while(0)
#define SET_Oi_ps2_circle_shift_l()      \
  do {p2_sw_trig = 1; p2_sw_top = 0; p2_sw_aux1 = 1; p2_sw_aux2 = 1;} while(0)

#define SET_Oi_ps2_x_shift_r()      \
  do {p2_sw_trig = 1; p2_sw_top = 1; p2_sw_aux1 = 0; p2_sw_aux2 = 0;} while(0)
#define SET_Oi_ps2_square_shift_r()      \
  do {p2_sw_trig = 1; p2_sw_top = 1; p2_sw_aux1 = 0; p2_sw_aux2 = 1;} while(0)
#define SET_Oi_ps2_triangle_shift_r()    \
  do {p2_sw_trig = 1; p2_sw_top = 1; p2_sw_aux1 = 1; p2_sw_aux2 = 0;} while(0)
#define SET_Oi_ps2_circle_shift_r()      \
  do {p2_sw_trig = 1; p2_sw_top = 1; p2_sw_aux1 = 1; p2_sw_aux2 = 1;} while(0)

/* Set the previous value of the PS2 buttons */
#define Oi_ps2_x_no_shift_prev        (!p2_sw_trig_prev && !p2_sw_top_prev && \
                                       !p2_sw_aux1_prev && p2_sw_aux2_prev)
#define Oi_ps2_square_no_shift_prev   (!p2_sw_trig_prev && !p2_sw_top_prev && \
                                       p2_sw_aux1_prev && !p2_sw_aux2_prev)
#define Oi_ps2_triangle_no_shift_prev (!p2_sw_trig_prev && !p2_sw_top_prev && \
                                       p2_sw_aux1_prev && p2_sw_aux2_prev)
#define Oi_ps2_circle_no_shift_prev   (!p2_sw_trig_prev && p2_sw_top_prev && \
                                       !p2_sw_aux1_prev && !p2_sw_aux2_prev)

/* Set the previous value of the left shifted PS2 buttons */
#define Oi_ps2_x_shift_l_prev         (p2_sw_trig_prev && !p2_sw_top_prev && \
                                       !p2_sw_aux1_prev && !p2_sw_aux2_prev)
#define Oi_ps2_square_shift_l_prev    (p2_sw_trig_prev && !p2_sw_top_prev && \
                                       !p2_sw_aux1_prev && p2_sw_aux2_prev)
#define Oi_ps2_triangle_shift_l_prev  (p2_sw_trig_prev && !p2_sw_top_prev && \
                                       p2_sw_aux1_prev && !p2_sw_aux2_prev)
#define Oi_ps2_circle_shift_l_prev    (p2_sw_trig_prev && !p2_sw_top_prev && \
                                       p2_sw_aux1_prev && p2_sw_aux2_prev)

/* Set the previous value of the right shifted PS2 buttons */
#define Oi_ps2_x_shift_r_prev         (p2_sw_trig_prev && p2_sw_top_prev && \
                                       !p2_sw_aux1_prev && !p2_sw_aux2_prev)
#define Oi_ps2_square_shift_r_prev    (p2_sw_trig_prev && p2_sw_top_prev && \
                                       !p2_sw_aux1_prev && p2_sw_aux2_prev)
#define Oi_ps2_triangle_shift_r_prev  (p2_sw_trig_prev && p2_sw_top_prev && \
                                       p2_sw_aux1_prev && !p2_sw_aux2_prev)
#define Oi_ps2_circle_shift_r_prev    (p2_sw_trig_prev && p2_sw_top_prev && \
                                       p2_sw_aux1_prev && p2_sw_aux2_prev)


/************************************
    Drive PS2 Inputs
    Left Stick X  - X
    Left Stick Y  - Y
    Right Stick X - Wheel
    Right Stick Y - Aux
    L2            - Top
    R2            - Trig
    L1            - Aux2
    R1            - Aux1

 ***********************************/

/*************************
 * Joystick Inputs
 *************************/

/***** Drive Joystick Analog Inputs *****/
#define Oi_drive_x                    p4_wheel
#define Oi_drive_y                    p4_aux

#define Oi_crab_x                     p4_x
#define Oi_crab_y                     p4_y

/***** Drive Joystick Digital Inputs *****/
#define Oi_sw_turbo                   p4_sw_trig
#define Oi_sw_landing_gear            p4_sw_top
#define Oi_sw_monster_mode            p4_sw_aux2
#define Oi_sw_car                     p4_sw_aux1

#define Oi_sw_cal_crab                p4_sw_aux2        /* L1 */
#define Oi_sw_cal_crab_prev           p4_sw_aux2_prev   /* L1 */

/***** Manipulator Joystick Analog Inputs *****/
#define Oi_roller                     p2_y              /* Left Stick */
#define Oi_slapper                    p2_wheel          /* Right Stick Y */

/***** Manipulator Joystick Digital Inputs *****/
/* these buttons will progress the lift towards the end position */
#define Oi_sw_lift_home               Oi_ps2_x_no_shift
#define Oi_sw_lift_home_prev          Oi_ps2_x_no_shift_prev
#define Oi_sw_lift_gather             Oi_ps2_circle_no_shift
#define Oi_sw_lift_gather_prev        Oi_ps2_circle_no_shift_prev
#define Oi_sw_lift_hurdle             Oi_ps2_square_no_shift
#define Oi_sw_lift_hurdle_prev        Oi_ps2_square_no_shift_prev
#define Oi_sw_lift_drive_by           Oi_ps2_triangle_no_shift
#define Oi_sw_lift_drive_by_prev      Oi_ps2_triangle_no_shift_prev

/* Manual lift controls */
#define Oi_sw_lift_manu_mode            Oi_ps2_is_shifted
#define Oi_sw_lift_manu_mode_left       Oi_ps2_is_shifted_l
#define Oi_sw_lift_manu_mode_right      Oi_ps2_is_shifted_r

#define Oi_sw_lift_manu_accum_in        Oi_ps2_square_shift_l
#define Oi_sw_lift_manu_accum_out       Oi_ps2_circle_shift_l
#define Oi_sw_lift_manu_lift_up         Oi_ps2_triangle_shift_l
#define Oi_sw_lift_manu_lift_up_prev    Oi_ps2_triangle_shift_l_prev
#define Oi_sw_lift_manu_lift_down       Oi_ps2_x_shift_l
#define Oi_sw_lift_manu_lift_down_prev  Oi_ps2_x_shift_l_prev

#define Oi_sw_lift_manu_nest_in         Oi_ps2_square_shift_r
#define Oi_sw_lift_manu_nest_out        Oi_ps2_circle_shift_r
#define Oi_sw_lift_manu_level_up        Oi_ps2_triangle_shift_r
#define Oi_sw_lift_manu_level_down      Oi_ps2_x_shift_r

/* These are needed because the switches are boolean operations
   and we can't set them directly */
#define SET_Oi_lift_home            SET_Oi_ps2_x_no_shift
#define SET_Oi_lift_hurdle          SET_Oi_ps2_square_no_shift
#define SET_Oi_lift_drive_by        SET_Oi_ps2_triangle_no_shift
#define SET_Oi_lift_gather          SET_Oi_ps2_circle_no_shift
#define SET_Oi_lift_manu_lift_down  SET_Oi_ps2_x_shift_l
#define SET_Oi_lift_manu_accum_in   SET_Oi_ps2_square_shift_l
#define SET_Oi_lift_manu_lift_up    SET_Oi_ps2_triangle_shift_l
#define SET_Oi_lift_manu_accum_out  SET_Oi_ps2_circle_shift_l
#define SET_Oi_lift_manu_level_down SET_Oi_ps2_x_shift_r
#define SET_Oi_lift_manu_nest_out   SET_Oi_ps2_square_shift_r
#define SET_Oi_lift_manu_level_up   SET_Oi_ps2_triangle_shift_r
#define SET_Oi_lift_manu_nest_in    SET_Oi_ps2_circle_shift_r


/*************************
 * Button Box Inputs
 *************************/

/* Driver */
#define Oi_sw_crab_manu_mode          p3_sw_trig   /* top right */
#define Oi_sw_theta_correct           p3_sw_top    /* bottom right */
#define Oi_auto_prog_select           p3_y
#define Oi_auto_color_select          p3_sw_aux1
#define Oi_auto_pos_select            p3_aux
#define Oi_auto_delay_select          p3_wheel
#define Oi_sw_auto_lockin             p3_sw_aux2
#define Oi_sw_auto_lockin_prev        p3_sw_aux2_prev

/* Manipulator */
#define Oi_sw_accum_unfold            p1_sw_aux1      /* bottom right */
#define Oi_sw_accum_unfold_prev       p1_sw_aux1_prev /* bottom right */

#define Oi_sw_lift_disabled           p1_sw_trig      /* top left */

/* General */
#define Oi_calibrate                  p3_x         /* right analog */


/* debug buttons */
#define Oi_sw_encoder_debug           p1_sw_top
#define Oi_sw_crab_pot_debug          p1_sw_aux1     /* bottom right */
#define Oi_eeprom_debug               p1_sw_aux2
#define Oi_eeprom_debug_prev          p1_sw_aux2_prev

/*
   P1
   AUX2 - TOP RIGHT - UNMAPPED
   TRIG - UP LEFT - LIFT DISABLE
   TOP - BOT LEFT -  UNFOLD
   AUX1 - BOT RIGHT - MOMEMTARY UNMPPED

   P3
   TRIG - MID RIGHT - CRAB DISBLE
   TOP - LOW RIGHT - THETA
   AUX2 - LOW LEFT - AUTO LOCK
   AUX1 - LOW MID - RED/BLUE
   X - UP RIGHT - CALIBRATE (LEFT = 0, MID = 127, RIGHT = 254)
   AUX - MID MID - Start pos (LEFT = 0, MID = 127, RIGHT = 254)
   WHEEL - MID LEFT - DELAY (LEFT = 0, MID = 127, RIGHT = 254)
   Y - AUTO DIAL - 0 position == 230

   */

/**************************************************************
 * Outputs
 **************************************************************/
/***** RC Digital Outputs *****/
#define Rc_relay_nest_tilt           relay1_fwd   /* blue/black   */
#define Rc_relay_accum_tilt          relay1_rev   /* blue/white   */
#define Rc_relay_landing_gear        relay2_fwd   /* green/black  */
#define Rc_relay_slapper             relay2_rev   /* green/white  */
#define Rc_relay_lift_1_up           relay3_fwd   /* orange/white */
#define Rc_relay_lift_1_down         relay3_rev
#define Rc_relay_lift_2_up           relay4_fwd   /* red/white    */
#define Rc_relay_lift_2_down         relay4_rev
#define Rc_relay_nest_level_up       relay5_fwd   /* yellow/white */
#define Rc_relay_nest_level_down     relay5_rev
#define Rc_relay_ladder_up           relay6_fwd   /* grey/white    */
#define Rc_relay_ladder_down         relay6_rev
#define Rc_relay_7_fwd               relay7_fwd   /* purple/white   */
#define Rc_relay_7_rev               relay7_rev
#define Rc_relay_8_fwd               relay8_fwd
#define Rc_relay_pump                relay8_rev   /* red/green  */

/***** RC Analog Outputs *****/
#define Rc_analog_out_drive_rf            pwm01   /* blue         */
#define Rc_analog_out_drive_rb            pwm02   /* green        */
#define Rc_analog_out_drive_lf            pwm03   /* orange       */
#define Rc_analog_out_drive_lb            pwm04   /* red          */
#define Rc_analog_out_pwm05               pwm05   /* ?            */
#define Rc_analog_out_pwm06               pwm06   /* ?            */
#define Rc_analog_out_roller              pwm07   /* purple       */
#define Rc_analog_out_pwm08               pwm08   /* brown        */
#define Rc_analog_out_pwm09               pwm09   /* black        */
#define Rc_analog_out_pwm10               pwm10   /* blue/brown   */
#define Rc_analog_out_pwm11               pwm11   /* green/brown  */
#define Rc_analog_out_pwm12               pwm12   /* orange/brown */
#define Rc_analog_out_front_crab          pwm13   /* yellow       */
#define Rc_analog_out_back_crab           pwm14   /* grey         */
#define Rc_analog_out_pwm15               pwm15   /* ?            */
#define Rc_analog_out_pwm16               pwm16   /* ?            */

/**************************************************************
 * LEDs
 **************************************************************/

/***** Input Scaling *****/
/* drive joystick scaling constants */
#define DRIVE_STICK_X_MIN        0
#define DRIVE_STICK_X_MIDDLE     127
#define DRIVE_STICK_X_MAX        246

#define DRIVE_STICK_Y_MIN        0
#define DRIVE_STICK_Y_MIDDLE     127
#define DRIVE_STICK_Y_MAX        246

#define CRAB_STICK_X_MIN         0
#define CRAB_STICK_X_MIDDLE      127
#define CRAB_STICK_X_MAX         246

#define CRAB_STICK_Y_MIN         0
#define CRAB_STICK_Y_MIDDLE      127
#define CRAB_STICK_Y_MAX         246

#define SC_CALIB_STICK_DEADZONE  5

/****************************** STRUCTURES ************************************/

/************************* FUNCTION PROTOTYPES ********************************/
extern void assign_outputs_slow(void);
extern void assign_outputs_fast(void);
extern UINT8 joystick_scaling(UINT8, UINT8, UINT8, UINT8);
extern void io_print_oi_inputs(void);
extern void io_print_rc_inputs(void);
extern void display_oi_data(UINT8, DisplayDataType);
extern void set_motor_vals_off(void);

#endif /* __ws_io_h_ */

