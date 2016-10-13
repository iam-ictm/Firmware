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

#ifndef __APPS_CANPROXY_H
#define __APPS_CANPROXY_H

/****************************************************************************
 * Definitions
 ****************************************************************************/

#ifndef CONFIG_CAN
#  error "CAN device support is not enabled (CONFIG_CAN)"
#endif

#ifndef CONFIG_STM32_CAN1
#  error "CAN device 1 is not enabled (CONFIG_STM32_CAN1)"
#endif

#ifndef CONFIG_STM32_CAN2
#  error "CAN device 2 is not enabled (CONFIG_STM32_CAN2)"
#endif

#ifndef CAN_REMOTE
#  define CAN_REMOTE "/dev/can0"
#endif

#ifndef CAN_CONTROL
#  define CAN_CONTROL "/dev/can1"
#endif

#ifndef CAN_MSGSIZE
#  define CAN_MSGSIZE (sizeof(struct can_msg_s))
#endif


/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) syslog(__VA_ARGS__)
#    define msgflush()
#  else
#    define message(...) printf(__VA_ARGS__)
#    define msgflush() fflush(stdout)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message syslog
#    define msgflush()
#  else
#    define message printf
#    define msgflush() fflush(stdout)
#  endif
#endif

/****************************************************************************
 * Exports
 ****************************************************************************/

int can_devinit(void);

#endif /* __APPS_CANPROXY_H */
