/*******************************************************************************
* FILE NAME: ws_cc.h
*
* DESCRIPTION:
*  This is the include file which corresponds to ws_cc.c
*
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __ws_cc_h_
#define __ws_cc_h_

/******************************* TYPEDEFS *************************************/

typedef enum
{
   CC_SUCCESS = 1,
   CC_TIMEOUT,
   CC_FAIL
} CcReturnType;

/******************************** MACROS **************************************/

/***************************** DEFINITIONS ************************************/

/* Command bytes sent to CC to request data */
/* Left Encoder(8bit), Right Encoder(8bit), Orient(8bit) */
#define  CC_CMD_REQ_ENCODER                2

/* Command bytes sent to CC to set data or state */

/* Ack bytes */

/* Command bytes sent to CC for debugging purposes */


/* Num of loops before giving up on a resp from the CC */
#define  CC_LOOP_CNT_TIMEOUT  10000


/* Size of the response to various commands */
#define CC_RESP_ENCODER_VAL_SIZE  8  /* 4 encoder bytes, 
                                        2 orient bytes,
                                        2 sonar bytes
                                      */


/* Valid Data states */
#define CC_INVALID_DATA      0
#define CC_VALID_DATA        1

/****************************** STRUCTURES ************************************/

/************************* FUNCTION PROTOTYPES ********************************/

extern UINT8 cc_get_encoder_vals(EncoderValsType *);
extern UINT8 read_cc(unsigned char *, UINT8, UINT8, UINT8, UINT16);

#endif /* __ws_cc_h_ */

