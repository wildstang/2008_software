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


/* NEED TO MAKE CLEAN WHEN CHANGING THIS */
#define TWO_BUTTON_GRIPPER 0
#define ANALOG_GRIPPER 1


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

/***************************** DEFINITIONS ************************************/
#define OI_CALIBRATE_ENCODERS    50
#define OI_CALIBRATE_JOYSTICKS   200

#define OI_ROTATE_LEFT           200
#define OI_ROTATE_RIGHT          50

#define PRESSURE_BELOW_120        0
#define PRESSURE_ABOVE_120        1

#define RAMP_LOCKED_PWM           230
#define RAMP_UNLOCKED_PWM         128

/**************************************************************
 * Inputs
 **************************************************************/

/***** RC Analog Inputs *****/
#define Analog_in_front_crab             rc_ana_in02
#define Analog_in_back_crab              rc_ana_in01
#define Analog_in_shoulder               rc_ana_in03
#define Analog_in_elbow                  rc_ana_in04
#define Analog_in_rotate                 rc_ana_in05
#define Analog_in_06                     rc_ana_in06
#define Analog_in_07                     rc_ana_in07
#define Analog_in_08                     rc_ana_in08
#define Analog_in_09                     rc_ana_in09
#define Analog_in_10                     rc_ana_in10
#define Analog_in_11                     rc_ana_in11
#define Analog_in_12                     rc_ana_in12
#define Analog_in_13                     rc_ana_in13
#define Analog_in_14                     rc_ana_in14
#define Analog_in_15                     rc_ana_in15
#define Analog_in_16                     rc_ana_in16


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
/* Shift functionality triggered by L1 */
#define Oi_ps2_x                 (!p2_sw_trig && !p2_sw_aux1 && !p2_sw_top && p2_sw_aux2)
#define Oi_ps2_square_no_shift   (!p2_sw_trig && !p2_sw_top && p2_sw_aux1 && !p2_sw_aux2)
#define Oi_ps2_triangle_no_shift (!p2_sw_trig && !p2_sw_top && p2_sw_aux1 && p2_sw_aux2)
#define Oi_ps2_circle_no_shift   (!p2_sw_trig && p2_sw_top && !p2_sw_aux1 && !p2_sw_aux2)
#define Oi_ps2_square_shift_l    (!p2_sw_trig && p2_sw_top && !p2_sw_aux1 && p2_sw_aux2)
#define Oi_ps2_triangle_shift_l  (!p2_sw_trig && p2_sw_top && p2_sw_aux1 && !p2_sw_aux2)
#define Oi_ps2_circle_shift_l    (!p2_sw_trig && p2_sw_top && p2_sw_aux1 && p2_sw_aux2)
#define Oi_ps2_square_shift_r    (p2_sw_trig && !p2_sw_top && !p2_sw_aux1 && !p2_sw_aux2)
#define Oi_ps2_triangle_shift_r  (p2_sw_trig && !p2_sw_top && !p2_sw_aux1 && p2_sw_aux2)
#define Oi_ps2_circle_shift_r    (p2_sw_trig && !p2_sw_top && p2_sw_aux1 && !p2_sw_aux2)


#define SET_Oi_ps2_x()                 \
  do {p2_sw_trig = 0; p2_sw_top = 0; p2_sw_aux1 = 0; p2_sw_aux2 = 1;} while(0)
#define SET_Oi_ps2_square_no_shift()   \
  do {p2_sw_trig = 0; p2_sw_top = 0; p2_sw_aux1 = 1; p2_sw_aux2 = 0;} while(0)
#define SET_Oi_ps2_triangle_no_shift() \
  do {p2_sw_trig = 0; p2_sw_top = 0; p2_sw_aux1 = 1; p2_sw_aux2 = 1;} while(0)
#define SET_Oi_ps2_circle_no_shift()   \
  do {p2_sw_trig = 0; p2_sw_top = 1; p2_sw_aux1 = 0; p2_sw_aux2 = 0;} while(0)
#define SET_Oi_ps2_square_shift_l()      \
  do {p2_sw_trig = 0; p2_sw_top = 1; p2_sw_aux1 = 0; p2_sw_aux2 = 1;} while(0)
#define SET_Oi_ps2_triangle_shift_l()    \
  do {p2_sw_trig = 0; p2_sw_top = 1; p2_sw_aux1 = 1; p2_sw_aux2 = 0;} while(0)
#define SET_Oi_ps2_circle_shift_l()      \
  do {p2_sw_trig = 0; p2_sw_top = 1; p2_sw_aux1 = 1; p2_sw_aux2 = 1;} while(0)
#define SET_Oi_ps2_square_shift_r()      \
  do {p2_sw_trig = 1; p2_sw_top = 0; p2_sw_aux1 = 0; p2_sw_aux2 = 0;} while(0)
#define SET_Oi_ps2_triangle_shift_r()    \
  do {p2_sw_trig = 1; p2_sw_top = 0; p2_sw_aux1 = 0; p2_sw_aux2 = 1;} while(0)
#define SET_Oi_ps2_circle_shift_r()      \
  do {p2_sw_trig = 1; p2_sw_top = 0; p2_sw_aux1 = 1; p2_sw_aux2 = 0;} while(0)


#define Oi_ps2_x_prev \
        (!p2_sw_aux1_prev && !p2_sw_top_prev && p2_sw_aux2_prev)
#define Oi_ps2_square_no_shift_prev \
        (!p2_sw_trig_prev && !p2_sw_top_prev && \
         p2_sw_aux1_prev && !p2_sw_aux2_prev)
#define Oi_ps2_triangle_no_shift_prev \
        (!p2_sw_trig_prev && !p2_sw_top_prev && \
         p2_sw_aux1_prev && p2_sw_aux2_prev)
#define Oi_ps2_circle_no_shift_prev \
        (!p2_sw_trig_prev && p2_sw_top_prev && \
         !p2_sw_aux1_prev && !p2_sw_aux2_prev)
#define Oi_ps2_square_shift_l_prev \
        (!p2_sw_trig_prev && p2_sw_top_prev && \
         !p2_sw_aux1_prev && p2_sw_aux2_prev)
#define Oi_ps2_triangle_shift_l_prev \
        (!p2_sw_trig_prev && p2_sw_top_prev && \
         p2_sw_aux1_prev && !p2_sw_aux2_prev)
#define Oi_ps2_circle_shift_l_prev \
        (!p2_sw_trig_prev && p2_sw_top_prev && \
         p2_sw_aux1_prev && p2_sw_aux2_prev)
#define Oi_ps2_square_shift_r_prev \
        (p2_sw_trig_prev && !p2_sw_top_prev && \
         !p2_sw_aux1_prev && !p2_sw_aux2_prev)
#define Oi_ps2_triangle_shift_r_prev \
        (p2_sw_trig_prev && !p2_sw_top_prev && \
         !p2_sw_aux1_prev && p2_sw_aux2_prev)
#define Oi_ps2_circle_shift_r_prev \
        (p2_sw_trig_prev && !p2_sw_top_prev && \
         p2_sw_aux1_prev && !p2_sw_aux2_prev)

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


/***** Drive Joystick Analog Inputs *****/

#if 1
#define Oi_drive_x                    p4_wheel
#define Oi_drive_y                    p4_aux
#define Oi_crab_x                     p4_x
#define Oi_crab_y                     p4_y
#else
#define Oi_drive_x                    p4_x
#define Oi_drive_y                    p4_y
#define Oi_crab_x                     p2_x
#define Oi_crab_y                     p2_y
#endif

/***** Drive Joystick Digital Inputs *****/
#define Oi_sw_turbo                   p4_sw_trig        /* R2 */
#define Oi_sw_landing_gear            p4_sw_top         /* L2 */
#define Oi_sw_crab_arc_cw             p4_sw_aux1        /* R1 */
#define Oi_sw_crab_arc_ccw            g_crab_arc_ccw

#define Oi_sw_cal_crab                p4_sw_aux2        /* L1 */
#define Oi_sw_cal_crab_prev           p4_sw_aux2_prev   /* L1 */


/***** Manipulator Joystick Analog Inputs *****/
#if 0
#define Oi_arm_shoulder               p2_aux            /* Right Y */
#define Oi_arm_elbow                  p2_y              /* Left Y */
#define Oi_arm_rotate                 p2_x              /* D pad */
#endif

/***** Manipulator Joystick Digital Inputs *****/
#if 0
#define Oi_sw_arm_telescope           Oi_ps2_x
#define Oi_sw_arm_telescope_prev      Oi_ps2_x_prev

#if TWO_BUTTON_GRIPPER
#define Oi_sw_gripper_open            p3_sw_aux1      /* L2 */
#define Oi_sw_gripper_open_prev       p3_sw_aux1_prev /* L2 */

#define Oi_sw_gripper_close           p2_sw_trig      /* R2 */
#define Oi_sw_gripper_close_prev      p2_sw_trig_prev /* R2 */
#elif ANALOG_GRIPPER
#define Oi_sw_gripper                 p2_wheel
#define Oi_sw_gripper_prev            p2_wheel_prev
#else
#define Oi_sw_gripper                 p3_sw_aux1     /* L2 */
#define Oi_sw_gripper_prev            p3_sw_aux1_prev/* L2 */
#endif
#endif

#if 0
#define Oi_sw_cal_shoulder            Oi_ps2_circle_no_shift
#define Oi_sw_cal_shoulder_prev       Oi_ps2_circle_no_shift_prev
#define Oi_sw_cal_elbow               Oi_ps2_triangle_no_shift
#define Oi_sw_cal_elbow_prev          Oi_ps2_triangle_no_shift_prev
#define Oi_sw_cal_rotate              Oi_ps2_square_no_shift
#define Oi_sw_cal_rotate_prev         Oi_ps2_square_no_shift_prev
#if TWO_BUTTON_GRIPPER
#define Oi_sw_cal_middles             (p3_sw_aux1 && p2_sw_trig && Oi_ps2_x)
                                      /* L2 && R2 && X */
#define Oi_sw_cal_middles_prev        (p3_sw_aux1_prev && p2_sw_trig_prev && \
                                       Oi_ps2_x_prev)
#else
#define Oi_sw_cal_middles             (p2_wheel)
                                      /* L2 && R2 && X */
#define Oi_sw_cal_middles_prev        ( p2_wheel_prev)
#endif

#define Oi_sw_arm_preset_0  Oi_ps2_square_no_shift
#define Oi_sw_arm_preset_1  Oi_ps2_triangle_no_shift
#define Oi_sw_arm_preset_2  Oi_ps2_circle_no_shift
#define Oi_sw_arm_preset_3  Oi_ps2_square_shift_l
#define Oi_sw_arm_preset_4  Oi_ps2_triangle_shift_l
#define Oi_sw_arm_preset_5  Oi_ps2_circle_shift_l
#define Oi_sw_arm_preset_6  Oi_ps2_square_shift_r
#define Oi_sw_arm_preset_7  Oi_ps2_triangle_shift_r
#define Oi_sw_arm_preset_8  Oi_ps2_circle_shift_r

#define Oi_sw_arm_preset_grab_floor    Oi_sw_arm_preset_0
#define Oi_sw_arm_preset_storage       Oi_sw_arm_preset_1
#define Oi_sw_arm_preset_grab_hp       Oi_sw_arm_preset_2
#define Oi_sw_arm_preset_score_low     Oi_sw_arm_preset_3
#define Oi_sw_arm_preset_score_mid     Oi_sw_arm_preset_4
#define Oi_sw_arm_preset_score_high    Oi_sw_arm_preset_5
#define Oi_sw_arm_preset_driveby_left  Oi_sw_arm_preset_6
#define Oi_sw_arm_preset_rotate_center Oi_sw_arm_preset_7
#define Oi_sw_arm_preset_driveby_right Oi_sw_arm_preset_8

#define SET_Oi_sw_arm_preset_0()  SET_Oi_ps2_square_no_shift()
#define SET_Oi_sw_arm_preset_1()  SET_Oi_ps2_triangle_no_shift()
#define SET_Oi_sw_arm_preset_2()  SET_Oi_ps2_circle_no_shift()
#define SET_Oi_sw_arm_preset_3()  SET_Oi_ps2_square_shift_l()
#define SET_Oi_sw_arm_preset_4()  SET_Oi_ps2_triangle_shift_l()
#define SET_Oi_sw_arm_preset_5()  SET_Oi_ps2_circle_shift_l()
#define SET_Oi_sw_arm_preset_6()  SET_Oi_ps2_square_shift_r()
#define SET_Oi_sw_arm_preset_7()  SET_Oi_ps2_triangle_shift_r()
#define SET_Oi_sw_arm_preset_8()  SET_Oi_ps2_circle_shift_r()

#define SET_Oi_sw_arm_preset_grab_floor()      SET_Oi_sw_arm_preset_0()
#define SET_Oi_sw_arm_preset_storage()         SET_Oi_sw_arm_preset_1()
#define SET_Oi_sw_arm_preset_grab_hp()         SET_Oi_sw_arm_preset_2()
#define SET_Oi_sw_arm_preset_score_low()       SET_Oi_sw_arm_preset_3()
#define SET_Oi_sw_arm_preset_score_mid()       SET_Oi_sw_arm_preset_4()
#define SET_Oi_sw_arm_preset_score_high()      SET_Oi_sw_arm_preset_5()
#define SET_Oi_sw_arm_preset_driveby_left()    SET_Oi_sw_arm_preset_6()
#define SET_Oi_sw_arm_preset_rotate_center()   SET_Oi_sw_arm_preset_7()
#define SET_Oi_sw_arm_preset_driveby_right()   SET_Oi_sw_arm_preset_8()
#endif

/***** Button Box Inputs *****/
/* Driver */
#define Oi_sw_crab_manu_mode          p3_sw_trig   /* top right */
#define Oi_sw_arm_manu_override       p1_sw_trig   /* middle left */
#define Oi_sw_theta_correct           p3_sw_top    /* bottom right */

#define Oi_sw_ramp_release            p1_sw_top    /* top left */
#define Oi_sw_tower_release           p1_sw_aux1   /* bottom left */
/* p1_sw_trig   middle left */
/* p1_sw_top    top left */
/* p1_x left    analog */

/* Manipulator */

/* General */
#define Oi_calibrate                  p3_x         /* right analog */


/* R2 on both controllers */
#define PROTO_LOCKIN  0
#if PROTO_LOCKIN
#define Oi_sw_auto_lockin             (p4_sw_trig && p4_sw_top)
#define Oi_sw_auto_lockin_prev        (p4_sw_trig_prev && p4_sw_top_prev)
#define Oi_auto_prog_select           p1_x
#else
#define Oi_auto_prog_select           p3_y
#define Oi_sw_auto_lockin             p3_sw_aux2
#define Oi_sw_auto_lockin_prev        p3_sw_aux2_prev
#endif

/*
#define Oi_auto_prog_select           p3_y
#define Oi_auto_pos_select            p3_x
#define Oi_auto_delay_select          p1_y
*/

/* debug buttons */
#define Oi_sw_encoder_debug           p1_sw_top
/*
#define Oi_sw_arm_debug               p1_sw_trig
*/


/**************************************************************
 * Outputs
 **************************************************************/
/***** RC Digital Outputs *****/
#define Rc_relay_gripper_bottom      relay1_fwd   /* blue/black   */
#define Rc_relay_gripper_top         relay1_rev
#define Rc_relay_telescope           relay2_fwd   /* green/black  */
#define Rc_relay_2_rev               relay2_rev   /* green/white  */
#define Rc_relay_landing_gear        relay3_fwd   /* orange/black */
#define Rc_relay_3_rev               relay3_rev
#define Rc_relay_ramp_release        relay4_fwd   /* red/black    */
#define Rc_relay_tower_release       relay4_rev
#define Rc_relay_5_fwd               relay5_fwd   /* yellow/black */
#define Rc_relay_5_rev               relay5_rev
#define Rc_relay_pump_on             relay6_fwd   /* red/green    */
#define Rc_relay_6_rev               relay6_rev
#define Rc_relay_7_fwd               relay7_fwd   /* grey/black   */
#define Rc_relay_7_rev               relay7_rev
#define Rc_relay_8_fwd               relay8_fwd   /* black/black  */
#define Rc_relay_8_rev               relay8_rev

/***** RC Analog Outputs *****/
#define Rc_analog_out_drive_rf            pwm01   /* blue         */
#define Rc_analog_out_drive_rb            pwm02   /* green        */
#define Rc_analog_out_drive_lf            pwm03   /* orange       */
#define Rc_analog_out_drive_lb            pwm04   /* red          */
#define Rc_analog_out_front_crab          pwm05   /* yellow       */
#define Rc_analog_out_back_crab           pwm06   /* grey         */
#define Rc_analog_out_shoulder            pwm07   /* purple       */
#define Rc_analog_out_elbow               pwm08   /* brown        */
#define Rc_analog_out_rotate              pwm09   /* black        */
#define Rc_analog_out_ramp_lock           pwm10   /* blue/brown   */
#define Rc_analog_out_pwm11               pwm11   /* green/brown  */
#define Rc_analog_out_pwm12               pwm12   /* orange/brown */
#define Rc_analog_out_pwm13               pwm13   /* ?            */
#define Rc_analog_out_pwm14               pwm14   /* ?            */
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

#define SHOULDER_STICK_MIN        0
#define SHOULDER_STICK_MIDDLE     127
#define SHOULDER_STICK_MAX        246

#define ELBOW_STICK_MIN         0
#define ELBOW_STICK_MIDDLE      127
#define ELBOW_STICK_MAX         246

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
extern void set_arm_motor_vals_off(void);

#endif /* __ws_io_h_ */

