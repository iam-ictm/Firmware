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

#include <px4_config.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <poll.h>

#include "uartpilot.h"

static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task_handle;


static void usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "Usage: uartpilot {start|stop|status} [ <serial port> [baudrate] ]\n");
	fprintf(stderr, "       (default: %s / %d)\n\n", UARTPILOT_DEVICE, UARTPILOT_BAUDRATE);
	exit(1);
}

__EXPORT int uartpilot_main(int argc, char *argv[]);
int uartpilot_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("uartpilot: Missing command");
	}

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("uartpilot: Already running\n");
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

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("uartpilot is running.");

		} else {
			warnx("uartpilot is stopped.");
		}
		exit(0);
	}

	usage("uartpilot: Unrecognized command");
	exit(1);
}

int uartpilot_thread_main(int argc, char *argv[])
{
	char *uart_name = UARTPILOT_DEVICE;
	unsigned uart_speed = UARTPILOT_BAUDRATE;

	if (argc == 2) {
		uart_name = argv[1];
	} else if (argc == 3) {
		uart_name = argv[1];

		errno = 0;
		long conv = strtoul(argv[2], NULL, 10);
		if (errno != 0 || conv > INT_MAX) {
			warnx("uartpilot: Invalid baud rate specified, using %d!\n", uart_speed);
		} else {
			uart_speed = conv;
		}
	}

	warnx("uartpilot: Opening port %s (baudrate %d)", uart_name, uart_speed);
	int uart_fd = open(uart_name, O_RDWR | O_NOCTTY);

	if (uart_fd < 0) {
		err(1, "uartpilot: Error opening port: %s", uart_name);
	}

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;

	/* Back up the original uart configuration to restore it after exit */
	if ((termios_state = tcgetattr(uart_fd, &uart_config)) < 0) {
		warnx("uartpilot: Error backing up uart config on %s: %d\n", uart_name, termios_state);
		close(uart_fd);
		return -1;
	}

	/* USB serial is indicated by /dev/ttyACM0*/
	if (strcmp(uart_name, "/dev/ttyACM0") != OK && strcmp(uart_name, "/dev/ttyACM1") != OK) {

		/* Set baud rate */
		if (cfsetispeed(&uart_config, uart_speed) < 0 || cfsetospeed(&uart_config, uart_speed) < 0) {
			warnx("uartpilot: Error setting baudrate on %s: %d\n", uart_name, termios_state);
			close(uart_fd);
			return -1;
		}

	}

	if ((termios_state = tcsetattr(uart_fd, TCSANOW, &uart_config)) < 0) {
		warnx("uartpilot: Error setting config on %s\n", uart_name);
		close(uart_fd);
		return -1;
	}

// struct actuator_outputs_s act_outputs;

	int sub_controls = orb_subscribe(ORB_ID(actuator_controls_3));
	int sub_armed = orb_subscribe(ORB_ID(actuator_armed));

	px4_pollfd_struct_t fds[1];
	fds[0].fd     = sub_controls;
	fds[0].events = POLLIN;
	/* Don't limit poll intervall for now, 250 Hz should be fine. */
	//orb_set_interval(_sub_controls, 10);

	struct actuator_controls_s controls_s;
	struct actuator_armed_s	armed_s;
	
	// Start disarmed
	armed_s.armed = false;
	armed_s.prearmed = false;

	thread_running = true;

	dprintf(uart_fd, "$AGSTAT%d\r\n",thread_running);
	while (!thread_should_exit) {
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), UARTPILOT_POLL_MILIS);

#ifdef UARTPILOT_DEBUG
		orb_copy(ORB_ID(actuator_armed), sub_armed, &armed_s);
		dprintf(uart_fd, "$DEBUG Poll returned %d\r\n", pret);
		dprintf(uart_fd, "$DEBUG Armed: %d Prearmed: %d Ready to arm: %d Lockdown: %d Force Failsafe: %d ESC Calibration: %d\r\n", armed_s.armed, armed_s.prearmed, armed_s.ready_to_arm, armed_s.lockdown, armed_s.force_failsafe, armed_s.in_esc_calibration_mode);
#endif

		/* Timed out, do a periodic check for _task_should_exit. */
		if (pret == 0) {
			continue;
		}

		/* This is undesirable but not much we can do. */
		if (pret < 0) {
			PX4_WARN("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(actuator_controls_1), sub_controls, &controls_s);
			orb_copy(ORB_ID(actuator_armed), sub_armed, &armed_s);

			dprintf(uart_fd, "$AGCOD %f %f %f %f %f %f %f %f %llu\r\n", (double) controls_s.control[0], (double) controls_s.control[1], (double) controls_s.control[2], (double) controls_s.control[3], (double) controls_s.control[4], (double) controls_s.control[5], (double) controls_s.control[6], (double) controls_s.control[7], controls_s.timestamp);
			dprintf(uart_fd, "$AGARM %d %d %d %d %d %d\r\n", armed_s.armed, armed_s.prearmed, armed_s.ready_to_arm, armed_s.lockdown, armed_s.force_failsafe, armed_s.in_esc_calibration_mode);
		}
/*
		bool updated;
		orb_check(sub_armed, &updated);
		if (updated) {
			orb_copy(ORB_ID(actuator_armed), sub_armed, &armed_s);
		}
*/	}

	thread_running = false;
	orb_unsubscribe(sub_controls);
	orb_unsubscribe(sub_armed);

	dprintf(uart_fd, "$AGSTAT%d\r\n",thread_running);

	warnx("uartpilot: Exiting");
	fflush(stdout);
	return 0;
}
