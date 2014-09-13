/*
 * Copyright © 2014 Keith Packard <keithp@keithp.com>
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

#include <err.h>
#include <fcntl.h>
#include <gelf.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <sysexits.h>
#include <unistd.h>
#include <getopt.h>
#include <string.h>
#include <stdbool.h>
#include "ao-elf.h"
#include "ccdbg.h"
#include "cc-usb.h"
#include "cc.h"
#include "ao-verbose.h"

static const struct option options[] = {
	{ .name = "tty", .has_arg = 1, .val = 'T' },
	{ .name = "device", .has_arg = 1, .val = 'D' },
	{ .name = "raw", .has_arg = 0, .val = 'r' },
	{ .name = "verbose", .has_arg = 1, .val = 'v' },
	{ 0, 0, 0, 0},
};

static void usage(char *program)
{
	fprintf(stderr, "usage: %s [--verbose=<verbose>] [--device=<device>] [-tty=<tty>] main|drogue\n", program);
	exit(1);
}

void
done(struct cc_usb *cc, int code)
{
/*	cc_usb_printf(cc, "a\n"); */
	cc_usb_close(cc);
	exit (code);
}

static int
ends_with(char *whole, char *suffix)
{
	int whole_len = strlen(whole);
	int suffix_len = strlen(suffix);

	if (suffix_len > whole_len)
		return 0;
	return strcmp(whole + whole_len - suffix_len, suffix) == 0;
}

static int
starts_with(char *whole, char *prefix)
{
	int whole_len = strlen(whole);
	int prefix_len = strlen(prefix);

	if (prefix_len > whole_len)
		return 0;
	return strncmp(whole, prefix, prefix_len) == 0;
}

static char **
tok(char *line) {
	char	**strs = malloc (sizeof (char *)), *str;
	int	n = 0;

	while ((str = strtok(line, " \t"))) {
		line = NULL;
		strs = realloc(strs, (n + 2) * sizeof (char *));
		strs[n] = strdup(str);
		n++;
	}
	strs[n] = '\0';
	return strs;
}

static void
free_strs(char **strs) {
	char	*str;
	int	i;

	for (i = 0; (str = strs[i]) != NULL; i++)
		free(str);
	free(strs);
}

struct gps {
	struct gps	*next;
	char		**strs;
};

static struct gps *
gps(struct cc_usb *usb)
{
	struct gps	*head = NULL, **tail = &head;
	cc_usb_printf(usb, "g\nv\n");
	for (;;) {
		char	line[512];
		struct gps	*b;

		cc_usb_getline(usb, line, sizeof (line));
		b = malloc (sizeof (struct gps));
		b->strs = tok(line);
		b->next = NULL;
		*tail = b;
		tail = &b->next;
		if (strstr(line, "software-version"))
			break;
	}
	return head;
}

static void
free_gps(struct gps *b) {
	struct gps *n;

	while (b) {
		n = b->next;
		free_strs(b->strs);
		free(b);
		b = n;
	}
}

char **
find_gps(struct gps *b, char *word0) {
	int i;
	for (;b; b = b->next)
		if (b->strs[0] && !strcmp(b->strs[0], word0))
			return b->strs;
	return NULL;
}

int
do_gps(struct cc_usb *usb) {
	int	count = 0;

	for (;;) {
		struct gps *b = gps(usb);
		char **flags = find_gps(b, "Flags:");
		char **sats = find_gps(b, "Sats:");
		int actual_flags = strtol(flags[1], NULL, 0);
		int actual_sats = strtol(sats[1], NULL, 0);

		if (actual_flags & (1 << 4)) {
			printf("Flags: %s (0x%x)\n", flags[1], actual_flags);
			printf("Sats: %s (%d)\n", sats[1], actual_sats);
			break;
		}

		free_gps(b);
		printf("%d", actual_sats);
		++count;
		if (count >= 50) {
			putchar('\n');
			count = 0;
		}
		else if (count % 10 == 0)
			putchar(' ');
		fflush(stdout);
		sleep(1);
	}
	return 1;
}

int
main (int argc, char **argv)
{
	char			*device = NULL;
	char			*filename;
	Elf			*e;
	unsigned int		s;
	int			i;
	int			c;
	int			tries;
	struct cc_usb		*cc = NULL;
	char			*tty = NULL;
	int			success;
	int			verbose = 0;
	int			ret = 0;

	while ((c = getopt_long(argc, argv, "rT:D:c:s:v:", options, NULL)) != -1) {
		switch (c) {
		case 'T':
			tty = optarg;
			break;
		case 'D':
			device = optarg;
			break;
		case 'v':
			verbose++;
			break;
		default:
			usage(argv[0]);
			break;
		}
	}

	ao_verbose = verbose;

	if (verbose > 1)
		ccdbg_add_debug(CC_DEBUG_BITBANG);

	if (!tty)
		tty = cc_usbdevs_find_by_arg(device, "TeleGPS");
	if (!tty)
		tty = cc_usbdevs_find_by_arg(device, "TeleMega");
	if (!tty)
		tty = cc_usbdevs_find_by_arg(device, "TeleMetrum");
	if (!tty)
		tty = getenv("ALTOS_TTY");
	if (!tty)
		tty="/dev/ttyACM0";

	cc = cc_usb_open(tty);

	if (!cc)
		exit(1);

	if (!do_gps(cc))
		ret = 1;
	done(cc, ret);
}
