/*******************************************************************************
* FILE NAME: ws_arm.h
*
* DESCRIPTION:
*  This is the include file which corresponds to ws_arm.c
*
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/
#ifndef __ws_arm_h_
#define __ws_arm_h_

#define ANALOG_GRIPPER_OPEN           200
#define ANALOG_GRIPPER_CLOSED         30

#define SHOULDER_DEADZONE    2
#define ELBOW_DEADZONE       2
#define ROTATE_DEADZONE    2

#define SHOULDER_MANU_DEADZONE      42
#define ELBOW_MANU_DEADZONE         42
#define ROTATE_MANU_DEADZONE        42

#define ROTATE_ACCEL_RATE     18

#define ARM_MASK_SHOULDER_SHIFT        (7)
#define ARM_MASK_ELBOW_SHIFT           (6)
#define ARM_MASK_ROTATE_SHIFT          (5)
#define ARM_MASK_GRIPPER_TOP_SHIFT     (4)
#define ARM_MASK_GRIPPER_BOTTOM_SHIFT  (3)
#define ARM_MASK_TELESCOPE_SHIFT       (2)
#define ARM_MASK_NONE_SHIFT            (1)
#define ARM_MASK_ALL_SHIFT             (0)

#define ARM_MASK_NONE             (1 << ARM_MASK_NONE_SHIFT)
#define ARM_MASK_ALL              (1 << ARM_MASK_ALL_SHIFT)
#define ARM_MASK_SHOULDER         (1 << ARM_MASK_SHOULDER_SHIFT)
#define ARM_MASK_ELBOW            (1 << ARM_MASK_ELBOW_SHIFT)
#define ARM_MASK_ROTATE           (1 << ARM_MASK_ROTATE_SHIFT)
#define ARM_MASK_GRIPPER_TOP      (1 << ARM_MASK_GRIPPER_TOP_SHIFT)
#define ARM_MASK_GRIPPER_BOTTOM   (1 << ARM_MASK_GRIPPER_BOTTOM_SHIFT)
#define ARM_MASK_TELESCOPE        (1 << ARM_MASK_TELESCOPE_SHIFT)

#define APPLY_CONSTRAINT_NONE(mode, c, mv, pid)    \
        if((c) == CONSTRAINT_ALLOW_NONE) \
        { \
          (mv) = 0; \
          if((mode) == ARM_CTRL_STATE_FEEDBACK) \
          { \
            clear_pid_vals_history(&pid); \
          } \
        }

#define APPLY_CONSTRAINT_FWD(mode, c, mv, pid)    \
        if(((c) == CONSTRAINT_ALLOW_FWD) && \
           ((mv) > 0)) \
        { \
          (mv) = 0; \
          if((mode) == ARM_CTRL_STATE_FEEDBACK) \
          { \
            clear_pid_vals_history(&pid); \
          } \
        }

#define APPLY_CONSTRAINT_REV(mode, c, mv, pid)    \
        if(((c) == CONSTRAINT_ALLOW_REV) && \
           ((mv) < 0)) \
        { \
          (mv) = 0; \
          if((mode) == ARM_CTRL_STATE_FEEDBACK) \
          { \
            clear_pid_vals_history(&pid); \
          } \
        }

#define APPLY_CONSTRAINTS(mode, c, mv, pid) \
    APPLY_CONSTRAINT_NONE((mode), (c), (mv), (pid)) \
    APPLY_CONSTRAINT_REV((mode), (c), (mv), (pid)) \
    APPLY_CONSTRAINT_FWD((mode), (c), (mv), (pid))

/* 10 bit pot values */
#define DEFAULT_SHOULDER_FRONT  400
#define DEFAULT_SHOULDER_BACK   600
#define DEFAULT_ELBOW_TOP       600
#define DEFAULT_ELBOW_BOTTOM    400
#define DEFAULT_ROTATE_LEFT     400
#define DEFAULT_ROTATE_RIGHT    600

#define DEFAULT_SHOULDER_HORIZ  500
#define DEFAULT_ELBOW_HORIZ     500
#define DEFAULT_ROTATE_HORIZ    500

typedef enum
{
  ARM_CTRL_STATE_MANUAL = 0,
  ARM_CTRL_STATE_MANUAL_OVERRIDE,
  ARM_CTRL_STATE_FEEDBACK
} ArmStateType;


void arm_init(void);
void arm_control(UINT8 force_feedback);
void arm_control_manual(UINT8 use_constraints);
void arm_control_feedback(void);

ArmPosType get_cur_arm_position(void);
void set_tgt_arm_position(void);

ArmConstraintType calc_constraints(ArmPosType pos_cur, ArmStateType mode);
void apply_constraints(ArmConstraintType constraints, ArmStateType mode);
extern ArmPosType feedback_tgt_pos;
extern UINT8 telescope_prev;
#if ANALOG_GRIPPER
extern UINT8 gripper_prev;
#endif




#endif /* __ws_arm_h_ */
