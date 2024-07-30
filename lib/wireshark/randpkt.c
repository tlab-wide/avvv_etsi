/*
 * randpkt.c
 * ---------
 * Creates random packet traces. Useful for debugging sniffers by testing
 * assumptions about the veracity of the data found in the packet.
 *
 * Copyright (C) 1999 by Gilbert Ramirez <gram@alumni.rice.edu>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <config.h>

#include <glib.h>

#include <stdio.h>
#include <stdlib.h>
#include <ui/clopts_common.h>
#include <ui/failure_message.h>
#include <ui/cmdarg_err.h>
#include <ui/exit_codes.h>
#include <wsutil/file_util.h>
#include <wsutil/filesystem.h>
#include <wsutil/privileges.h>
#include <cli_main.h>

#ifdef HAVE_PLUGINS
#include <wsutil/plugins.h>
#endif

#include <wsutil/report_message.h>
#include <wsutil/wslog.h>

#include <wsutil/ws_getopt.h>

#include "randpkt_core/randpkt_core.h"

/* Additional exit codes */
#define INVALID_TYPE 2
#define CLOSE_ERROR  2

/*
 * Report an error in command-line arguments.
 */
static void
randpkt_cmdarg_err(const char *msg_format, va_list ap)
{
	fprintf(stderr, "randpkt: ");
	vfprintf(stderr, msg_format, ap);
	fprintf(stderr, "\n");
}

/*
 * Report additional information for an error in command-line arguments.
 */
static void
randpkt_cmdarg_err_cont(const char *msg_format, va_list ap)
{
	vfprintf(stderr, msg_format, ap);
	fprintf(stderr, "\n");
}

/* Print usage statement and exit program */
static void
usage(gboolean is_error)
{
	FILE *output;
	char** abbrev_list;
	char** longname_list;
	unsigned i = 0;

	if (!is_error) {
		output = stdout;
	}
	else {
		output = stderr;
	}

	fprintf(output, "Usage: randpkt [-b maxbytes] [-c count] [-t type] [-r] filename\n");
	fprintf(output, "Default max bytes (per packet) is 5000\n");
	fprintf(output, "Default count is 1000.\n");
	fprintf(output, "-r: random packet type selection\n");
	fprintf(output, "\n");
	fprintf(output, "Types:\n");

	/* Get the examples list */
	randpkt_example_list(&abbrev_list, &longname_list);
	while (abbrev_list[i] && longname_list[i]) {
		fprintf(output, "\t%-16s%s\n", abbrev_list[i], longname_list[i]);
		i++;
	}

	g_strfreev(abbrev_list);
	g_strfreev(longname_list);

	fprintf(output, "\nIf type is not specified, a random packet will be chosen\n\n");
}

int
main(int argc, char *argv[])
{
	char *init_progfile_dir_error;
	static const struct report_message_routines randpkt_report_routines = {
		failure_message,
		failure_message,
		open_failure_message,
		read_failure_message,
		write_failure_message,
		cfile_open_failure_message,
		cfile_dump_open_failure_message,
		cfile_read_failure_message,
		cfile_write_failure_message,
		cfile_close_failure_message
	};
	int opt;
	int produce_type = -1;
	char *produce_filename = NULL;
	int produce_max_bytes = 5000;
	int produce_count = 1000;
	randpkt_example *example;
	guint8* type = NULL;
	int allrandom = FALSE;
	wtap_dumper *savedump;
	int ret = EXIT_SUCCESS;
	static const struct ws_option long_options[] = {
		{"help", ws_no_argument, NULL, 'h'},
		{0, 0, 0, 0 }
	};

	cmdarg_err_init(randpkt_cmdarg_err, randpkt_cmdarg_err_cont);

	/* Initialize log handler early so we can have proper logging during startup. */
	ws_log_init("randpkt", vcmdarg_err);

	/* Early logging command-line initialization. */
	ws_log_parse_args(&argc, argv, vcmdarg_err, INVALID_OPTION);

	/*
	 * Get credential information for later use.
	 */
	init_process_policies();

	/*
	 * Attempt to get the pathname of the directory containing the
	 * executable file.
	 */
	init_progfile_dir_error = init_progfile_dir(argv[0]);
	if (init_progfile_dir_error != NULL) {
		fprintf(stderr,
			"capinfos: Can't get pathname of directory containing the capinfos program: %s.\n",
			init_progfile_dir_error);
		g_free(init_progfile_dir_error);
	}

	init_report_message("randpkt", &randpkt_report_routines);

	wtap_init(TRUE);

#ifdef _WIN32
	create_app_running_mutex();
#endif /* _WIN32 */

	while ((opt = ws_getopt_long(argc, argv, "b:c:ht:r", long_options, NULL)) != -1) {
		switch (opt) {
			case 'b':	/* max bytes */
				produce_max_bytes = get_positive_int(ws_optarg, "max bytes");
				if (produce_max_bytes > 65536) {
					cmdarg_err("max bytes is > 65536");
					ret = INVALID_OPTION;
					goto clean_exit;
				}
				break;

			case 'c':	/* count */
				produce_count = get_positive_int(ws_optarg, "count");
				break;

			case 't':	/* type of packet to produce */
				type = g_strdup(ws_optarg);
				break;

			case 'h':
				usage(FALSE);
				goto clean_exit;
				break;

			case 'r':
				allrandom = TRUE;
				break;

			default:
				usage(TRUE);
				ret = INVALID_OPTION;
				goto clean_exit;
				break;
		}
	}

	/* any more command line parameters? */
	if (argc > ws_optind) {
		produce_filename = argv[ws_optind];
	} else {
		usage(TRUE);
		ret = INVALID_OPTION;
		goto clean_exit;
	}

	if (!allrandom) {
		produce_type = randpkt_parse_type(type);
		g_free(type);

		example = randpkt_find_example(produce_type);
		if (!example) {
			ret = INVALID_OPTION;
			goto clean_exit;
		}

		ret = randpkt_example_init(example, produce_filename, produce_max_bytes);
		if (ret != EXIT_SUCCESS)
			goto clean_exit;
		randpkt_loop(example, produce_count, 0);
	} else {
		if (type) {
			fprintf(stderr, "Can't set type in random mode\n");
			ret = INVALID_TYPE;
			goto clean_exit;
		}

		produce_type = randpkt_parse_type(NULL);
		example = randpkt_find_example(produce_type);
		if (!example) {
			ret = INVALID_OPTION;
			goto clean_exit;
		}
		ret = randpkt_example_init(example, produce_filename, produce_max_bytes);
		if (ret != EXIT_SUCCESS)
			goto clean_exit;

		while (produce_count-- > 0) {
			randpkt_loop(example, 1, 0);
			produce_type = randpkt_parse_type(NULL);

			savedump = example->dump;

			example = randpkt_find_example(produce_type);
			if (!example) {
				ret = INVALID_OPTION;
				goto clean_exit;
			}
			example->dump = savedump;
			example->filename = produce_filename;
		}
	}
	if (!randpkt_example_close(example)) {
		ret = CLOSE_ERROR;
	}

clean_exit:
	wtap_cleanup();
	return ret;
}

/*
 * Editor modelines  -  https://www.wireshark.org/tools/modelines.html
 *
 * Local variables:
 * c-basic-offset: 8
 * tab-width: 8
 * indent-tabs-mode: t
 * End:
 *
 * vi: set shiftwidth=8 tabstop=8 noexpandtab:
 * :indentSize=8:tabSize=8:noTabs=false:
 */
