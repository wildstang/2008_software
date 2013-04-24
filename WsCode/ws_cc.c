/*******************************************************************************
* FILE NAME: ws_cc.c
*
* DESCRIPTION:
*
* USAGE:
*
*******************************************************************************/

#include <stdio.h>
#include "ifi_frc.h"
#include "ws_includes.h"
#include "ws_io.h"
#include "serial_ports.h"
#include "ws_cc.h"

#define ENCODER_FWD_POSITIVE 0

CcReqDataType s_cc_req_data = CC_REQ_UNINIT;


/*******************************************************************************
* FUNCTION NAME: cc_get_new_data
* PURPOSE:
*******************************************************************************/
void cc_get_new_data(void)
{
  static int eeprom_rqst_count = 0;

  /* get new data from the CC */
  cc_read_data();

  if (calibration_vals.state == CALIBRATION_BAD_RC_DATA)
  {
    if (eeprom_rqst_count < 80)
    {
      /* calibration data in RC eeprom is bad, try to get it from the CC, but
         only try for a limited time, give up after a while */
      cc_request_eeprom_calibration(NUM_EEPROM_BYTES);
      eeprom_rqst_count++;
    }
    else
    {
      calibration_vals.state = CALIBRATION_BAD_CC_DATA;
    }
  }
  else
  {
    cc_request_encoder_vals();
  }
}


/*******************************************************************************
* FUNCTION NAME: cc_request_eeprom_calibration
* PURPOSE:
*******************************************************************************/
CcReqStatusType cc_request_eeprom_calibration(UINT8 num_bytes)
{
  if (s_cc_req_data == CC_REQ_UNINIT)
  {
    s_cc_req_data = CC_REQ_EEPROM;
    Write_Serial_Port_Two(CC_CMD_REQ_EEPROM_CALIB);
    Write_Serial_Port_Two(num_bytes);
    return CC_REQ_SUCCESS;
  }
  else
  {
    printf("!!! invalid CC request: eeprom !!!");
    return CC_REQ_FAILURE;
  }
}


/*******************************************************************************
* FUNCTION NAME: cc_request_encoder_vals
* PURPOSE:
*******************************************************************************/
CcReqStatusType cc_request_encoder_vals(void)
{
  if (s_cc_req_data == CC_REQ_UNINIT)
  {
    s_cc_req_data = CC_REQ_ENCODER;
    Write_Serial_Port_Two(CC_CMD_REQ_ENCODER);
    return CC_REQ_SUCCESS;
  }
  else
  {
    printf("!!! invalid CC request: encoder !!!");
    return CC_REQ_FAILURE;
  }
}


/*******************************************************************************
* FUNCTION NAME: cc_send_eeprom_calibration
* PURPOSE:
*******************************************************************************/
void cc_send_eeprom_calibration(UINT8 num_bytes, UINT8 *data)
{
  int i;

  /* send the command */
  Write_Serial_Port_Two(CC_CMD_SET_EEPROM_CALIB);

  /* send the number of bytes */
  Write_Serial_Port_Two(num_bytes);

#if PRINT_CC_DATA
  printf("Send eeprom %dB ", num_bytes);
#endif

  /* send the data */
  for (i = 0; i < num_bytes; i++)
  {
#if PRINT_CC_DATA
    printf("%02X ", data[i]);
#endif
    Write_Serial_Port_Two(data[i]);
  }
}


/*******************************************************************************
* FUNCTION NAME: cc_read_data
* PURPOSE:
*******************************************************************************/
void cc_read_data(void)
{
  UINT8 data[CC_MAX_DATA_SIZE];
  int i;

  switch (s_cc_req_data)
  {
    case CC_REQ_ENCODER:
      if (read_cc(data, CC_RESP_ENCODER_VAL_SIZE) == CC_RESP_ENCODER_VAL_SIZE)
      {
        cc_data.type = CC_REQ_ENCODER;

        /* data is |Left|Right|OrientHi|OrientLo|BallClose|BallFar|
           break out the data */
#if PRINT_CC_DATA
        printf("data=%02X%02X%02X%02X%02X%02X%02X%02X ",
               (int)data[0], (int)data[1], (int)data[2], (int)data[3],
               (int)data[4], (int)data[5]);
#endif

#if ENCODER_FWD_POSITIVE
        /* Left encoder is byte 0 */
        cc_data.data.encoder.left = data[0];

        /* Right encoder is byte 1 */
        cc_data.data.encoder.right = data[1];
#else
        /* Left encoder is byte 0 */
        cc_data.data.encoder.right = 0 - data[0];

        /* Right encoder is byte 1 */
        cc_data.data.encoder.left = 0 - data[1];
#endif

        /* Orient is bytes 2 & 3 */
        cc_data.data.encoder.orient = ((INT16)data[2] << 8) |
                                         data[3];

        /* Near ball is byte 4 */
        cc_data.data.encoder.ball_near_pos = data[4];

        /* Far ball is byte 5 */
        cc_data.data.encoder.ball_far_pos = data[5];

#if PRINT_CC_DATA
        printf("L=%2d R=%2d O=%3d NB=%d FB=%d ", cc_data.data.encoder.left,
               cc_data.data.encoder.right, cc_data.data.encoder.orient,
               cc_data.data.encoder.ball_near_pos,
               cc_data.data.encoder.ball_far_pos);
#endif
      }
      else
      {
        cc_data.type = CC_REQ_UNINIT;
#if PRINT_CC_ERRORS
        printf("CC TO ENCODER ");
#endif
      }

      break;

    case CC_REQ_EEPROM:
      if (read_cc(data, CC_RESP_EEPROM_SIZE) == CC_RESP_EEPROM_SIZE)
      {
        cc_data.type = CC_REQ_EEPROM;

        /* copy the data */
#if PRINT_CC_DATA
        printf("data=");
#endif
        for (i = 0; i < CC_RESP_EEPROM_SIZE; i++)
        {
          cc_data.data.eeprom[i] = data[i];
#if PRINT_CC_DATA
          printf("%02X", (int)data[i]);
#endif
        }
#if PRINT_CC_DATA
        printf(" ");
#endif
      }
      else
      {
        cc_data.type = CC_REQ_UNINIT;
#if PRINT_CC_ERRORS
        printf("CC TO EEPROM ");
#endif
      }

      break;

    case CC_REQ_UNINIT:
    default:
      cc_data.type = CC_REQ_UNINIT;
      printf("!!! reading data when none was requested !!!");
      break;
  }

  s_cc_req_data = CC_REQ_UNINIT;
}


/*******************************************************************************
* FUNCTION NAME: read_cc
* PURPOSE:
*******************************************************************************/
UINT8 read_cc(UINT8* p_data, UINT8 resp_size)
{
  UINT8  bytes_rcvd = 0; /* total bytes received */
  UINT8  i;

  if (Serial_Port_Two_Byte_Count() == resp_size)
  {
    /* correct number of bytes are queued up */
    for (i = 0; i < resp_size; i++)
    {
      p_data[i] = Read_Serial_Port_Two();
      bytes_rcvd++;
    }
  }

  /* clear the queue regardless, this handles the case where we have less than
     the correct number of bytes or if some extra garbage snuck in */
  clear_serial_port_two_rx();

  return(bytes_rcvd);
}

