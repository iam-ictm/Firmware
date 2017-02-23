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

#ifndef __APPS_UARTPILOT_H
#define __APPS_UARTPILOT_H

/****************************************************************************
 * Definitions
 ****************************************************************************/

//#define UARTPILOT_DEBUG 1
//#define UARTPILOT_TEST 1
#define UARTPILOT_EXTENDED_MSGS 1

#ifndef UARTPILOT_DEVICE
#  define UARTPILOT_DEVICE "/dev/ttyS6"
#endif

#ifndef UARTPILOT_BAUDRATE
#  define UARTPILOT_BAUDRATE 115200
#endif

#ifndef UARTPILOT_POLL_MILIS
#  define UARTPILOT_POLL_MILIS 0	// Default 10
#endif

#ifndef UARTPILOT_MAX_MSG_LEN
#  define UARTPILOT_MAX_MSG_LEN 80
#endif


/****************************************************************************
 * Exports
 ****************************************************************************/

int uartpilot_main(int argc, char *argv[]);
int uartpilot_thread_main(int argc, char *argv[]);

#endif /* __APPS_UARTPILOT_H */
