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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

#include <px4_posix.h>
#include <termios.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>

#include "uartpilot.h"

static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task_handle;

static void usage(const char *reason)
{
	if (reason) warnx("%s\n", reason);
	warnx("Usage: uartpilot {start|stop|status} [ <serial port> [baudrate] ]\n");
	warnx("       (default: %s / %d)\n\n", UARTPILOT_DEVICE, UARTPILOT_BAUDRATE);
	exit(1);
}

static void uartprintf(int fd, const char *format, ...) {
	char msg[UARTPILOT_MAX_MSG_LEN];

	va_list argp;
	va_start(argp, format);
	vsnprintf(msg, UARTPILOT_MAX_MSG_LEN, format, argp);
	va_end(argp);

	uint8_t checksum = 0;
	char *c = msg;
	while(*c) checksum ^= *c++;

	dprintf(fd, "$%s*%02x\r\n", msg, checksum);
}

static inline uint8_t convert(double val) {
	return (uint8_t) (val * 127.5 - 0.5);
}

__EXPORT int uartpilot_main(int argc, char *argv[]);

int uartpilot_main(int argc, char *argv[])
{
	if (argc < 2) usage("Missing command!");

	if (strcmp(argv[1], "start") == 0) {
		if (thread_running) {
			warnx("uartpilot is already running!\n");
			exit(0);
		}

		thread_should_exit = false;
		daemon_task_handle = px4_task_spawn_cmd("uartpilot",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 5,
						 2000,
						 uartpilot_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		exit(0);
	}

	if (strcmp(argv[1], "stop") == 0) {
		thread_should_exit = true;
		exit(0);
	}

	if (strcmp(argv[1], "status") == 0) {
		if (thread_running) warnx("uartpilot is running.");
		else warnx("uartpilot is stopped.");
		exit(0);
	}

	usage("Unrecognized command!");
	exit(1);
}

int uartpilot_thread_main(int argc, char *argv[])
{
/****************************************************************************
 * Setup UART
 ****************************************************************************/

	char *uart_name = UARTPILOT_DEVICE;
	unsigned uart_speed = UARTPILOT_BAUDRATE;

	if (argc == 2) {
		uart_name = argv[1];
	} else if (argc == 3) {
		uart_name = argv[1];

		errno = 0;
		long conv = strtoul(argv[2], NULL, 10);
		if (errno != 0 || conv > INT_MAX) err(1, "Invalid baud rate specified!");
	}

	warnx("Opening port %s (baudrate %d)", uart_name, uart_speed);
	int uart_fd = open(uart_name, O_RDWR | O_NOCTTY);

	if (uart_fd < 0) err(1, "Error opening port: %s", uart_name);

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;

	/* Back up the original uart configuration to restore it after exit */
	if ((termios_state = tcgetattr(uart_fd, &uart_config)) < 0) {
		close(uart_fd);
		err(1, "Error backing up uart config on %s: %d\n", uart_name, termios_state);
	}

	/* USB serial is indicated by /dev/ttyACM0*/
	if (strcmp(uart_name, "/dev/ttyACM0") != OK && strcmp(uart_name, "/dev/ttyACM1") != OK) {
		/* Set baud rate */
		if (cfsetispeed(&uart_config, uart_speed) < 0 || cfsetospeed(&uart_config, uart_speed) < 0) {
			close(uart_fd);
			err(1, "Error setting baudrate on %s: %d\n", uart_name, termios_state);
		}
	}

	if ((termios_state = tcsetattr(uart_fd, TCSANOW, &uart_config)) < 0) {
		close(uart_fd);
		err(1, "Error setting config on %s\n", uart_name);
	}

/****************************************************************************
 * Setup ORB subscriptions
 ****************************************************************************/

	/* Subscribe armed actuator and prepare structure for storing data */
	int sub_armed = orb_subscribe(ORB_ID(actuator_armed));

	struct actuator_armed_s s_armed;
	s_armed.armed = false;
	s_armed.prearmed = false;

	/* Subscribe to control groups and prepare structure for storing data */
	int sub_controls[1];
	sub_controls[0] = orb_subscribe(ORB_ID(actuator_controls_0));
	//orb_set_interval(_sub_controls, 10);    TODO Don't limit poll intervall for now, 250 Hz should be fine.

	struct actuator_controls_s s_controls;

	px4_pollfd_struct_t fds_poll[1];
	fds_poll[0].fd     = sub_controls[0];
	fds_poll[0].events = POLLIN;

/****************************************************************************
 * Main loop, poll ORB and send commands to UART
 ****************************************************************************/

	thread_running = true;

#ifdef UARTPILOT_EXTENDED_MSGS
	uartprintf(uart_fd, "AGSTA,%d",thread_running);
#endif

	while (!thread_should_exit) {
		int pret = px4_poll(fds_poll, (sizeof(fds_poll) / sizeof(fds_poll[0])), UARTPILOT_POLL_MILIS);

#ifdef UARTPILOT_DEBUG
		orb_copy(ORB_ID(actuator_armed), sub_armed, &s_armed);
		uartprintf(uart_fd, "AGDBG,Poll returned %d", pret);
		uartprintf(uart_fd, "AGARM,%d,%d,%d,%d,%d,%d", s_armed.armed, s_armed.prearmed, s_armed.ready_to_arm, s_armed.lockdown, s_armed.force_failsafe, s_armed.in_esc_calibration_mode);
#endif

#ifdef UARTPILOT_TEST
		uartprintf(uart_fd, "AGCOD,%d,%d", convert(-0.783), convert(0.561));
		continue;
#endif

		/* Timed out, do a periodic check for thread_should_exit. */
		if (pret == 0) continue;

		/* This is undesirable but not much we can do. */
		if (pret < 0) {
			warnx("Poll error %d, %d!", pret, errno);
			usleep(100000);
			continue;
		}

		if (fds_poll[0].revents & POLLIN) {
			orb_copy(ORB_ID(actuator_controls_0), sub_controls[0], &s_controls);
			uartprintf(uart_fd, "AGCOD,%d,%d", convert(s_controls.control[3]), convert(s_controls.control[2]));
		}

#if defined(UARTPILOT_EXTENDED_MSGS) && !defined(UARTPILOT_DEBUG) && !defined(UARTPILOT_TEST)
		bool updated;
		orb_check(sub_armed, &updated);
		if (updated) {
			orb_copy(ORB_ID(actuator_armed), sub_armed, &s_armed);
			uartprintf(uart_fd, "AGARM,%d,%d,%d,%d,%d,%d", s_armed.armed, s_armed.prearmed, s_armed.ready_to_arm, s_armed.lockdown, s_armed.force_failsafe, s_armed.in_esc_calibration_mode);
		}
#endif
	}

/****************************************************************************
 * Cleanup and exit
 ****************************************************************************/
	thread_running = false;
	orb_unsubscribe(sub_controls[0]);
	orb_unsubscribe(sub_armed);

#ifdef UARTPILOT_EXTENDED_MSGS
	uartprintf(uart_fd, "AGSTA,%d",thread_running);
#endif

	warnx("Exiting...");
	return 0;
}
