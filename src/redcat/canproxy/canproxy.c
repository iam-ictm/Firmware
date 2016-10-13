/****************************************************************************
 *
 *   Copyright (C) 2016 BFH - Bern University of Applied Sciences
 *                      http://bfh.ch
 *   Author: Pascal Mainini <pascal.mainini@bfh.ch>
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

#include <stdio.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/can.h>
#include "canproxy.h"

/****************************************************************************
 * Name: canproxy
 ****************************************************************************/

__EXPORT int canproxy_main(int argc, char *argv[]);
int canproxy_main(int argc, char *argv[])
{
	int fd_remote, fd_control;
	int retval = 0;

	struct can_msg_s canmsg;
	ssize_t nbytes;

	message("canproxy: Initializing CAN hardware...\n");
	retval = can_devinit();
	if (retval != OK)
	{
		message("canproxy: Failure initializing hardware: %d!\n",
			retval);
		retval = 1;
		goto errout;
	}

	message("canproxy: Hardware initialized, opening devices...\n");
	fd_remote = open(CAN_REMOTE, O_RDWR);
	fd_control = open(CAN_CONTROL, O_RDWR);

	if (fd_remote < 0)
	{
		message("canproxy: Opening %s failed: %d!\n",
			CAN_REMOTE, errno);
		retval = 2;
		goto errout_close_fd;
	}

	if (fd_control < 0)
	{
		message("canproxy: Opening %s failed: %d!\n",
			CAN_CONTROL, errno);
		retval = 2;
		goto errout_close_fd;
	}

	message("canproxy: Devices opened, got filedescriptors %d and %d.\n",
		fd_remote, fd_control);

	for (int i=0; i < 10; i++)
	{
		nbytes = read(fd_remote, &canmsg,  CAN_MSGSIZE);
		if (nbytes < CAN_MSGLEN(0) || nbytes > CAN_MSGSIZE)
		{
			message("canproxy: Error: Read %d bytes "
				"(%d < n < %d)!\n",
				nbytes, CAN_MSGLEN(0), CAN_MSGSIZE);
			retval = 3;
			goto errout_close_fd;
		}

		message("canproxy: Received Bytes: %d, ID: %04x, DLC: %d"
			" DATA: %x\n",
			nbytes, canmsg.cm_hdr.ch_id, canmsg.cm_hdr.ch_dlc, 
			canmsg.cm_data);

		nbytes = write(fd_control, &canmsg, CAN_MSGSIZE);
		if (nbytes < 0)
		{
			message("canproxy: Error: write() returned %d!\n",
				errno);
			retval = 4;
			goto errout_close_fd;
		}

		message("canproxy: Retransmitted  %d bytes!\n", nbytes);
	}

	errout_close_fd:
		close(fd_remote);
		close(fd_control);

	errout:
		message("canproxy: Terminating!\n");
		msgflush();
		return retval;
}
