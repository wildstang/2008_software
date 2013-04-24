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

/* return values for requesting data */
typedef enum
{
  CC_REQ_FAILURE = 0,
  CC_REQ_SUCCESS
} CcReqStatusType;

/******************************** MACROS **************************************/

/***************************** DEFINITIONS ************************************/

/* Command bytes sent to CC to request data */
/* Left Encoder(8bit), Right Encoder(8bit), Orient(8bit) */
#define  CC_CMD_REQ_ENCODER                0x02
#define  CC_CMD_REQ_EEPROM_CALIB           0x11

/* Command bytes sent to CC to set data or state */
#define  CC_CMD_SET_EEPROM_CALIB           0x12

/* Ack bytes */

/* Command bytes sent to CC for debugging purposes */


/* Size of the response to various commands */
#define CC_RESP_ENCODER_VAL_SIZE  6  /* 2 encoder bytes,
                                        2 orient bytes,
                                        2 ball bytes
                                      */
#define CC_RESP_EEPROM_SIZE       NUM_EEPROM_BYTES

#define CC_MAX_DATA_SIZE CC_RESP_EEPROM_SIZE


/****************************** STRUCTURES ************************************/

/************************* FUNCTION PROTOTYPES ********************************/
extern void cc_get_new_data(void);
extern CcReqStatusType cc_request_eeprom_calibration(UINT8 num_bytes);
extern CcReqStatusType cc_request_encoder_vals(void);
extern void cc_send_eeprom_calibration(UINT8 num_bytes, UINT8 *data);
extern void cc_read_data(void);
extern UINT8 read_cc(UINT8* p_data, UINT8 resp_size);

#endif /* __ws_cc_h_ */

