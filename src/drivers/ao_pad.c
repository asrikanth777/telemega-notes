/*
 * Copyright © 2012 Keith Packard <keithp@keithp.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

#include <ao.h>
#include <ao_pad.h>
#include <ao_74hc497.h>

__xdata uint8_t ao_pad_ignite;

#define ao_pad_igniter_status(c)	AO_PAD_IGNITER_STATUS_UNKNOWN
#define ao_pad_arm_status()		AO_PAD_ARM_STATUS_UNKNOWN

#if 0
#define PRINTD(...) printf(__VA_ARGS__)
#else
#define PRINTD(...) 
#endif

static void
ao_pad_run(void)
{
	for (;;) {
		while (!ao_pad_ignite)
			ao_sleep(&ao_pad_ignite);
		/*
		 * Actually set the pad bits
		 */
		AO_PAD_PORT = (AO_PAD_PORT & (~AO_PAD_ALL_PINS)) | ao_pad_ignite;
		while (ao_pad_ignite) {
			ao_pad_ignite = 0;
			ao_delay(AO_PAD_FIRE_TIME);
		}
		AO_PAD_PORT &= ~(AO_PAD_ALL_PINS);
	}
}

static void
ao_pad_status(void)
{
	for (;;) {
		ao_delay(AO_SEC_TO_TICKS(1));
#if 0
		if (ao_igniter_status(ao_igniter_drogue) == ao_igniter_ready) {
			if (ao_igniter_status(ao_igniter_main) == ao_igniter_ready) {
				for (i = 0; i < 5; i++) {
					ao_beep_for(AO_BEEP_MID, AO_MS_TO_TICKS(50));
					ao_delay(AO_MS_TO_TICKS(100));
				}
			} else {
				ao_beep_for(AO_BEEP_MID, AO_MS_TO_TICKS(200));
			}
		}
#endif
	}
}

static __pdata uint8_t	ao_pad_armed;
static __pdata uint16_t	ao_pad_arm_time;
static __pdata uint8_t	ao_pad_box;

static void
ao_pad(void)
{
	static __xdata struct ao_pad_command	command;
	static __xdata struct ao_pad_query	query;
	int16_t	time_difference;
	uint8_t	c;

	ao_led_off(AO_LED_RED);
	ao_beep_for(AO_BEEP_MID, AO_MS_TO_TICKS(200));
	ao_pad_box = ao_74hc497_read();
	for (;;) {
		flush();
		if (ao_radio_cmac_recv(&command, sizeof (command), 0) != AO_RADIO_CMAC_OK)
			continue;
		
		PRINTD ("tick %d serial %d cmd %d channel %d\n",
			command.tick, command.serial, command.cmd, command.channel);

		switch (command.cmd) {
		case AO_LAUNCH_ARM:
			if (command.box != ao_pad_box) {
				PRINTD ("box number mismatch\n");
				break;
			}

			if (command.channels & ~(AO_PAD_ALL_PINS))
				break;

			time_difference = command.tick - ao_time();
			PRINTD ("arm tick %d local tick %d\n", command.tick, ao_time());
			if (time_difference < 0)
				time_difference = -time_difference;
			if (time_difference > 10) {
				PRINTD ("time difference too large %d\n", time_difference);
				break;
			}
			PRINTD ("armed\n");
			ao_pad_armed = command.channels;
			ao_pad_arm_time = ao_time();

			/* fall through ... */

		case AO_LAUNCH_QUERY:
			if (command.box != ao_pad_box) {
				PRINTD ("box number mismatch\n");
				break;
			}

			query.tick = ao_time();
			query.box = ao_pad_box;
			query.channels = AO_PAD_ALL_PINS;
			query.armed = ao_pad_armed;
			query.arm_status = ao_pad_arm_status();
			for (c = 0; c < AO_PAD_NUM; c++)
				query.igniter_status[c] = ao_pad_igniter_status(c);
			PRINTD ("query tick %d serial %d channel %d valid %d arm %d igniter %d\n",
				query.tick, query.serial, query.channel, query.valid, query.arm_status,
				query.igniter_status);
			ao_radio_cmac_send(&query, sizeof (query));
			break;
		case AO_LAUNCH_FIRE:
			if (!ao_pad_armed) {
				PRINTD ("not armed\n");
				break;
			}
			if ((uint16_t) (ao_time() - ao_pad_arm_time) > AO_SEC_TO_TICKS(20)) {
				PRINTD ("late pad arm_time %d time %d\n",
					ao_pad_arm_time, ao_time());
				break;
			}
			time_difference = command.tick - ao_time();
			if (time_difference < 0)
				time_difference = -time_difference;
			if (time_difference > 10) {
				PRINTD ("time different too large %d\n", time_difference);
				break;
			}
			PRINTD ("ignite\n");
			ao_pad_ignite = ao_pad_armed;
			ao_wakeup(&ao_pad_ignite);
			break;
		}
	}
}

void
ao_pad_test(void)
{
#if 0
	switch (ao_igniter_status(ao_igniter_drogue)) {
	case ao_igniter_ready:
	case ao_igniter_active:
		printf ("Armed: ");
		switch (ao_igniter_status(ao_igniter_main)) {
		default:
			printf("unknown status\n");
			break;
		case ao_igniter_ready:
			printf("igniter good\n");
			break;
		case ao_igniter_open:
			printf("igniter bad\n");
			break;
		}
		break;
	default:
		printf("Disarmed\n");
	}
#endif
}

void
ao_pad_manual(void)
{
	ao_cmd_white();
	if (!ao_match_word("DoIt"))
		return;
	ao_cmd_white();
	ao_pad_ignite = 1;
	ao_wakeup(&ao_pad_ignite);
}

static __xdata struct ao_task ao_pad_task;
static __xdata struct ao_task ao_pad_ignite_task;
static __xdata struct ao_task ao_pad_status_task;

__code struct ao_cmds ao_pad_cmds[] = {
	{ ao_pad_test,	"t\0Test pad continuity" },
	{ ao_pad_manual,	"i <key>\0Fire igniter. <key> is doit with D&I" },
	{ 0, NULL }
};

void
ao_pad_init(void)
{
#if AO_PAD_NUM > 0
	ao_enable_output(AO_PAD_PORT, AO_PAD_PIN_0, AO_PAD_0, 0);
#endif
#if AO_PAD_NUM > 1
	ao_enable_output(AO_PAD_PORT, AO_PAD_PIN_1, AO_PAD_1, 0);
#endif
#if AO_PAD_NUM > 2
	ao_enable_output(AO_PAD_PORT, AO_PAD_PIN_2, AO_PAD_2, 0);
#endif
#if AO_PAD_NUM > 3
	ao_enable_output(AO_PAD_PORT, AO_PAD_PIN_3, AO_PAD_3, 0);
#endif
	ao_cmd_register(&ao_pad_cmds[0]);
	ao_add_task(&ao_pad_task, ao_pad, "pad listener");
	ao_add_task(&ao_pad_ignite_task, ao_pad_run, "pad igniter");
	ao_add_task(&ao_pad_status_task, ao_pad_status, "pad status");
}
