/* console.h
 * Console log handler routines
 *
 * Wireshark - Network traffic analyzer
 * By Gerald Combs <gerald@wireshark.org>
 * Copyright 1998 Gerald Combs
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef __CONSOLE_H__
#define __CONSOLE_H__

#ifdef _WIN32 /* Needed for console I/O */
#include <fcntl.h>
#include <conio.h>
#include <ui/win32/console_win32.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <wsutil/wslog.h>

/** The GUI log writer.
 */
void
console_log_writer(const char *domain, enum ws_log_level level,
                                   ws_log_time_t timestamp,
                                   const char *file, int line, const char *func,
                                   const char *user_format, va_list user_ap,
                                   void *user_data);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __CONSOLE_H__ */
