/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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

/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_global_position.h>

__EXPORT int simple_control_main(int argc, char *argv[]);

int simple_control_main(int argc, char *argv[])
{
	PX4_INFO("\nSimple control\n");

	struct actuator_outputs_s actuators;

	struct vehicle_global_position_s global_pos;
	memset(&global_pos, 0, sizeof(global_pos));

	int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));

	struct pollfd fds[1] = {};
	fds[0].fd = global_pos_sub;
	fds[0].events = POLLIN;

	actuators.output[0] = 2000;
	actuators.output[1] = 2000;
	actuators.output[2] = 2000;
	actuators.output[3] = 2000;

	orb_advert_t actuators_pub = orb_advertise(ORB_ID(actuator_outputs), &actuators);

	while(1) {

		int ret = poll(fds, 1, 500);

		if (ret < 0) {
			PX4_INFO("poll error");
		} else if (ret == 0) {
			PX4_INFO("no return value");
		} else {
			bool pos_updated;
			orb_check(global_pos_sub, &pos_updated);
			if(fds[0].revents & POLLIN) {
				/* publish to actuators topic */
				orb_publish(ORB_ID(actuator_outputs), actuators_pub, &actuators);
				PX4_INFO("\nactuators published\n");
			}
		}
	}

	return 0;
}
