/*
 * Copyright © 2009 Keith Packard <keithp@keithp.com>
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

#include "ao.h"

static char
ao_packet_getchar(void)
{
	char c;
	while ((c = ao_packet_pollchar()) == AO_READ_AGAIN)
	{
		if (!ao_packet_enable)
			break;
		if (ao_packet_master_sleeping)
			ao_wake_task(&ao_packet_task);
		ao_usb_flush();
		ao_sleep(&ao_stdin_ready);
	}
	return c;
}

static void
ao_packet_echo(void) __reentrant
{
	uint8_t	c;
	while (ao_packet_enable) {
		c = ao_packet_getchar();
		if (ao_packet_enable)
			ao_usb_putchar(c);
	}
	ao_exit();
}

static __xdata struct ao_task	ao_packet_echo_task;
static __xdata uint16_t		ao_packet_master_delay;
static __xdata uint16_t		ao_packet_master_time;

#define AO_PACKET_MASTER_DELAY_SHORT	AO_MS_TO_TICKS(100)
#define AO_PACKET_MASTER_DELAY_LONG	AO_MS_TO_TICKS(1000)
#define AO_PACKET_MASTER_DELAY_TIMEOUT	AO_MS_TO_TICKS(2000)

static void
ao_packet_master_busy(void)
{
	ao_packet_master_delay = AO_PACKET_MASTER_DELAY_SHORT;
	ao_packet_master_time = ao_time();
}

static void
ao_packet_master_check_busy(void)
{
	int16_t	idle;
	if (ao_packet_master_delay != AO_PACKET_MASTER_DELAY_SHORT)
		return;
	idle = (int16_t) (ao_time() - ao_packet_master_time);

	if (idle > AO_PACKET_MASTER_DELAY_TIMEOUT)
		ao_packet_master_delay = AO_PACKET_MASTER_DELAY_LONG;
}

void
ao_packet_master(void)
{
	uint8_t	status;

	ao_radio_set_packet();
	ao_tx_packet.addr = ao_serial_number;
	ao_tx_packet.len = AO_PACKET_SYN;
	ao_packet_master_time = ao_time();
	ao_packet_master_delay = AO_PACKET_MASTER_DELAY_SHORT;
	while (ao_packet_enable) {
		ao_packet_send();
		if (ao_tx_packet.len)
			ao_packet_master_busy();
		ao_packet_master_check_busy();
		ao_alarm(ao_packet_master_delay);
		status = ao_packet_recv();
		if (status & AO_DMA_DONE) {
			/* if we can transmit data, do so */
			if (ao_packet_tx_used && ao_tx_packet.len == 0)
				continue;
			if (ao_rx_packet.packet.len)
				ao_packet_master_busy();
			ao_packet_master_sleeping = 1;
			ao_delay(ao_packet_master_delay);
			ao_packet_master_sleeping = 0;
		}
	}
	ao_radio_set_telemetry();
	ao_exit();
}

static void
ao_packet_forward(void) __reentrant
{
	char c;
	ao_packet_enable = 1;
	ao_cmd_white();

	flush();
	ao_set_monitor(0);
	ao_add_task(&ao_packet_task, ao_packet_master, "master");
	ao_add_task(&ao_packet_echo_task, ao_packet_echo, "echo");
	while ((c = ao_usb_getchar()) != '~') {
		if (c == '\r') c = '\n';
		ao_packet_putchar(c);
	}
	ao_packet_enable = 0;
	ao_radio_abort();
	while (ao_packet_echo_task.wchan || ao_packet_task.wchan) {
		ao_wake_task(&ao_packet_echo_task);
		ao_wake_task(&ao_packet_task);
		ao_yield();
	}
}



__code struct ao_cmds ao_packet_master_cmds[] = {
	{ 'p',	ao_packet_forward,	"p                                  Remote packet link." },
	{ 0,	ao_packet_forward,	NULL },
};

void
ao_packet_master_init(void)
{
	ao_cmd_register(&ao_packet_master_cmds[0]);
}
