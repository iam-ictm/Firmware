/****************************************************************************
 * examples/can/can_main.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/can.h>

#include "can.h"

#include <ctype.h>
#include <sys/time.h>
#include <sys/select.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#  define NMSG 10
#  define CAN_OFLAGS O_RDWR

#  define HBC_NEW_NODE_ID 3
#  define HBC_ORIG_NODE_ID 2
#  define HBC_NODE_DIFF (HBC_ORIG_NODE_ID - HBC_NEW_NODE_ID)
#  define HBC_TX_PDO_1 (0x180 + HBC_NEW_NODE_ID)
#  define HBC_TX_PDO_2 (0x280 + HBC_NEW_NODE_ID)
#  define HBC_TX_PDO_3 (0x380 + HBC_NEW_NODE_ID)
#  define HBC_TX_PDO_4 (0x480 + HBC_NEW_NODE_ID)


#ifdef CONFIG_CAN_EXTID
#  define MAX_ID (1 << 29)
#else
#  define MAX_ID (1 << 11)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: px4_hymog_can
 ****************************************************************************/

__EXPORT int px4_hymog_can_main(int argc, char *argv[]);
int px4_hymog_can_main(int argc, char *argv[])
{

#ifdef CONFIG_CAN_EXTID
  uint32_t msgid;
#else
  uint16_t msgid;
#endif

  struct can_msg_s txmsg;
  struct can_msg_s rxmsg;

  int msgdlc;
  //uint8_t msgdata;
  size_t msgsize;
  ssize_t nbytes;

  int fd;
  int errval = 0;
  int ret;
//  int nmsgs = NMSG;

  /* Initialization of the CAN hardware is performed by logic external to
   * this test.
   */

  message("can_main: Initializing external CAN device\n");
  ret = can_devinit();
  if (ret != OK)
    {
      message("can_main: can_devinit failed: %d\n", ret);
      errval = 1;
      goto errout;
    }

  /* Open the CAN device for reading */

  message("can_main: Hardware initialized. Opening the CAN device\n");
  fd = open(CONFIG_EXAMPLES_CAN_DEVPATH, CAN_OFLAGS);
  if (fd < 0)
    {
      message("can_main: open %s failed: %d\n",
              CONFIG_EXAMPLES_CAN_DEVPATH, errno);
      errval = 2;
      goto errout_with_dev;
    }

//    for (; nmsgs > 0; nmsgs--)
    for (;;)
  {
      /* Read the RX message */
      msgsize = sizeof(struct can_msg_s);
      nbytes = read(fd, &rxmsg, msgsize);
      if (nbytes < CAN_MSGLEN(0) || nbytes > msgsize)
        {
          message("ERROR: read(%d) returned %d\n", msgsize, nbytes);
          errval = 4;
          goto errout_with_dev;
        }

      /*Display RX message*/
//        message("Received ID: %3o DLC: %d DATA: %o\n", rxmsg.cm_hdr.ch_id, rxmsg.cm_hdr.ch_dlc, rxmsg.cm_data);

      /*Check whether message from Radio HBC Radiomatic*/
      if (rxmsg.cm_hdr.ch_id == HBC_TX_PDO_1 || rxmsg.cm_hdr.ch_id == HBC_TX_PDO_2
          ||rxmsg.cm_hdr.ch_id == HBC_TX_PDO_3 || rxmsg.cm_hdr.ch_id == HBC_TX_PDO_4)
        {
          txmsg = rxmsg;

          /*Change ID if message from HCB Radiomatic*/
          msgid = rxmsg.cm_hdr.ch_id + HBC_NODE_DIFF;
          msgdlc = rxmsg.cm_hdr.ch_dlc;
          msgsize = CAN_MSGLEN(msgdlc);


          msgflush();

          /* Send the TX message */
          txmsg.cm_hdr.ch_id = msgid;
          nbytes = write(fd, &txmsg, msgsize);
          if (nbytes != msgsize)
            {
              message("ERROR: write(%d) returned %d\n", msgsize, nbytes);
              errval = 3;
              goto errout_with_dev;
            }

          /*Display TX message*/
            message("Send ID: %.3x DLC: %d DATA: %.2x:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n\n",
            txmsg.cm_hdr.ch_id, txmsg.cm_hdr.ch_dlc, txmsg.cm_data[0], txmsg.cm_data[1], txmsg.cm_data[2]
                    , txmsg.cm_data[3], txmsg.cm_data[4], txmsg.cm_data[5], txmsg.cm_data[6], txmsg.cm_data[7]);
        }

  }

errout_with_dev:
  close(fd);

errout:
  message("Terminating!\n");
  msgflush();
  return errval;
}
