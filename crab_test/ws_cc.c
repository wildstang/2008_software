/*******************************************************************************
* FILE NAME: ws_cc.c
*
* DESCRIPTION:
*
* USAGE:
*
*******************************************************************************/

#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "ws_includes.h"
#include "user_routines.h"
#include "delays.h"       /*defined locally*/
#include "ws_io.h"
#include "serial_ports.h"
#include "ws_cc.h"


/* Use these defines to dump debug info from the CC in input_data */
/*
#define DEBUG_DIST_HDG
*/
/*
#define DEBUG_ENCODER_VALS
*/
#define PRINT_CC_ERRORS
/*
 */


/*******************************************************************************
* FUNCTION NAME: cc_get_encoder_vals
* PURPOSE:       Read the encoder values from the CC
* CALLED FROM:
* ARGUMENTS:
* RETURNS:
*******************************************************************************/
UINT8 cc_get_encoder_vals(EncoderValsType *p_encoder_vals)
{
  CcReturnType cc_ret_val = CC_SUCCESS;
  UINT8        encoder_vals_data[CC_RESP_ENCODER_VAL_SIZE];

  /* request encoder values */
  if (read_cc(&(encoder_vals_data[0]), CC_CMD_REQ_ENCODER, 0,
              CC_RESP_ENCODER_VAL_SIZE, CC_LOOP_CNT_TIMEOUT) <
      CC_RESP_ENCODER_VAL_SIZE)
  {
    /* No response from the CC...tell the auton code not to run */
    cc_ret_val = CC_TIMEOUT;
#ifdef PRINT_CC_ERRORS
    printf("CC TO-Encoder ");
#endif
  }
  else
  {
    /* encoder_vals_data is |LeftBack|RightBack|LeftFront|RightFront|
       OrientHi|OrientLo|SonarHi|SonarLow|
       break out the data */
#ifdef DEBUG_ENCODER_VALS
    printf("encoder_vals_data=%02X%02X%02X%02X%02X%02X%02X%02X ",
           (int)encoder_vals_data[0], (int)encoder_vals_data[1],
           (int)encoder_vals_data[2], (int)encoder_vals_data[3],
           (int)encoder_vals_data[4], (int)encoder_vals_data[5],
           (int)encoder_vals_data[6], (int)encoder_vals_data[7]);
#endif

    /* Left front encoder is byte 0 */
    p_encoder_vals->left_back = encoder_vals_data[0];

    /* Left back encoder is byte 1 */
    p_encoder_vals->right_back = encoder_vals_data[1];

    /* Right front encoder is byte 2 */
    p_encoder_vals->left_front = encoder_vals_data[2];

    /* Right back encoder is byte 3 */
    p_encoder_vals->right_front = encoder_vals_data[3];

    /* Orient is bytes 4 & 5 */
    p_encoder_vals->orient = ((INT16)encoder_vals_data[4] << 8) |
                                     encoder_vals_data[5];
    /* Sonar is bytes 6 & 7 */
    p_encoder_vals->sonar = ((UINT16)encoder_vals_data[6] << 8) |
                                    encoder_vals_data[7];
  }

#ifdef DEBUG_ENCODER_VALS
  printf("LF=%d LB=%d RF=%d RB=%d O=%d S %d\r", p_encoder_vals->left_front,
         p_encoder_vals->left_back, p_encoder_vals->right_front,
         p_encoder_vals->right_back, p_encoder_vals->orient,
         p_encoder_vals->sonar);
         
#endif

  return(cc_ret_val);
}


/*******************************************************************************
* FUNCTION NAME: read_cc
* PURPOSE:       Read a register on the CC.
* CALLED FROM:
* ARGUMENTS:
*            p_data    - Pointer to the buffer where the contents of the return
*                        data from the CC will be put
*            cmd       - Command byte sent to CC, either setting or requesting
*                        data
*            resp_size - Size of the response message from CC
*            timeout   - Timeout to wait for each byte of the message being
*                        returned
* RETURNS:
*            Number of bytes received from the CC
*******************************************************************************/
UINT8 read_cc(unsigned char* p_data, UINT8 cmd, UINT8 data, UINT8 resp_size,
              UINT16 timeout_per_char)
{
  UINT8  byte_recvd;      /* Was a byte recvd in this loop */
  UINT8  bytes_recvd = 0; /* How many bytes were recvd total */
  UINT8  i;
  UINT16 loop_count;

  /* Clear any extra chars out from the input buffer */
  clear_serial_port_two_rx();

  /* Send the command */
  Write_Serial_Port_Two(cmd);

  /* send out the data if there is any */
  if (data != 0)
  {
    Delay10TCY();
    Write_Serial_Port_Two(data);
  }

  /* Now recv all the expected response data, break out after all the
   * expected bytes are recvd or until we don't recv a byte.
   */
  byte_recvd = 1;   /* Init to 1 so the first loop works */
  for (i = 0; (i < resp_size) && (byte_recvd == 1); i++)
  {
    loop_count = timeout_per_char;
    byte_recvd = 0;

    do
    {
      /* only read from buffer if there is data waiting */
      if (Serial_Port_Two_Byte_Count() > 0)
      {
        /* put data recieved into byte_recvd */
        p_data[i] = Read_Serial_Port_Two();
        byte_recvd = 1;
      }

      loop_count--;
    } while ((byte_recvd == 0) && (loop_count > 0));

    /* Add the byte we recvd to the total count */
    bytes_recvd += byte_recvd;
  }

  return(bytes_recvd);
}

