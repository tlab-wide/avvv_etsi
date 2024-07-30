/* wsutil/to_str.h
 * Definitions for utilities to convert various other types to strings.
 *
 * Wireshark - Network traffic analyzer
 * By Gerald Combs <gerald@wireshark.org>
 * Copyright 1998 Gerald Combs
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef __WSUTIL_TO_STR_H__
#define __WSUTIL_TO_STR_H__

#include <glib.h>

#include <ws_symbol_export.h>
#include <wsutil/wmem/wmem.h>
#include <wsutil/inet_ipv6.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * guint8_to_hex()
 *
 * Output guint8 hex representation to 'out', and return pointer after last character (out + 2).
 * It will always output full representation (padded with 0).
 *
 * String is not NUL terminated by this routine.
 * There needs to be at least 2 bytes in the buffer.
 */
WS_DLL_PUBLIC char *guint8_to_hex(char *out, guint8 val);

/**
 * word_to_hex()
 *
 * Output guint16 hex representation to 'out', and return pointer after last character (out + 4).
 * It will always output full representation (padded with 0).
 *
 * String is not NUL terminated by this routine.
 * There needs to be at least 4 bytes in the buffer.
 */
WS_DLL_PUBLIC char *word_to_hex(char *out, guint16 word);

/**
 * word_to_hex_punct()
 *
 * Output guint16 hex representation to 'out', and return pointer after last character.
 * Each byte will be separated with punct character (cannot be NUL).
 * It will always output full representation (padded with 0).
 *
 * String is not NUL terminated by this routine.
 * There needs to be at least 5 bytes in the buffer.
 */
WS_DLL_PUBLIC char *word_to_hex_punct(char *out, guint16 word, char punct);

/**
 * word_to_hex_npad()
 *
 * Output guint16 hex representation to 'out', and return pointer after last character.
 * Value is not padded.
 *
 * String is not NUL terminated by this routine.
 * There needs to be at least 4 bytes in the buffer.
 */
WS_DLL_PUBLIC char *word_to_hex_npad(char *out, guint16 word);

/**
 * dword_to_hex()
 *
 * Output guint32 hex representation to 'out', and return pointer after last character.
 * It will always output full representation (padded with 0).
 *
 * String is not NUL terminated by this routine.
 * There needs to be at least 8 bytes in the buffer.
 */
WS_DLL_PUBLIC char *dword_to_hex(char *out, guint32 dword);

/**
 * dword_to_hex_punct()
 *
 * Output guint32 hex representation to 'out', and return pointer after last character.
 * Each byte will be separated with punct character (cannot be NUL).
 * It will always output full representation (padded with 0).
 *
 * String is not NUL terminated by this routine.
 * There needs to be at least 11 bytes in the buffer.
 */
WS_DLL_PUBLIC char *dword_to_hex_punct(char *out, guint32 dword, char punct);

/**
 * qword_to_hex()
 *
 * Output guint64 hex representation to 'out', and return pointer after last character.
 * It will always output full representation (padded with 0).
 *
 * String is not NUL terminated by this routine.
 * There needs to be at least 16 bytes in the buffer.
 */
WS_DLL_PUBLIC char *qword_to_hex(char *out, guint64 qword);

/**
 * qword_to_hex_punct()
 *
 * Output guint64 hex representation to 'out', and return pointer after last character.
 * Each byte will be separated with punct character (cannot be NUL).
 * It will always output full representation (padded with 0).
 *
 * String is not NUL terminated by this routine.
 * There needs to be at least 22 bytes in the buffer.
 */
WS_DLL_PUBLIC char *qword_to_hex_punct(char *out, guint64 qword, char punct);

/**
 * bytes_to_hexstr()
 *
 * Output hex representation of guint8 array, and return pointer after last character.
 * It will always output full representation (padded with 0).
 *
 * String is not NUL terminated by this routine.
 * There needs to be at least len * 2 bytes in the buffer.
 */
WS_DLL_PUBLIC char *bytes_to_hexstr(char *out, const guint8 *ad, size_t len);

/**
 * bytes_to_hexstr_punct()
 *
 * Output hex representation of guint8 array, and return pointer after last character.
 * Each byte will be separated with punct character (cannot be NUL).
 * It will always output full representation (padded with 0).
 *
 * String is not NUL terminated by this routine.
 * There needs to be at least len * 3 - 1 bytes in the buffer.
 */
WS_DLL_PUBLIC char *bytes_to_hexstr_punct(char *out, const guint8 *ad, size_t len, char punct);

/* Max string length for displaying byte string.  */
#define	MAX_BYTE_STR_LEN	72

/** Turn an array of bytes into a string showing the bytes in hex,
 *  separated by a punctuation character.
 *
 * @param scope memory allocation scheme used
 * @param ad A pointer to the byte array
 * @param len The length of the byte array
 * @param punct The punctuation character
 * @param max Maximum string length, plus ellipsis if present
 * @return A pointer to the formatted string
 *
 * @see bytes_to_str()
 */
WS_DLL_PUBLIC gchar *bytes_to_str_punct_max(wmem_allocator_t *scope, const guint8 *ad, size_t len, const char punct, size_t max);

#define bytes_to_str_punct(scope, ad, len, punct) bytes_to_str_punct_max(scope, ad, len, punct, MAX_BYTE_STR_LEN)

/** Turn an array of bytes into a string showing the bytes in hex.
 *
 * @param scope memory allocation scheme used
 * @param bd A pointer to the byte array
 * @param bd_len The length of the byte array
 * @param max Maximum string length, plus ellipsis if present
 * @return A pointer to the formatted string
 */
WS_DLL_PUBLIC char *bytes_to_str_max(wmem_allocator_t *scope, const guint8 *bd, size_t bd_len, size_t max);

#define bytes_to_str(scope, bd, len) bytes_to_str_max(scope, bd, len, MAX_BYTE_STR_LEN)

/**
 * oct_to_str_back()
 *
 * Output guint32 octal representation backward (last character will be written on ptr - 1),
 * and return pointer to first character.
 *
 * String is not NUL terminated by this routine.
 * There needs to be at least 12 bytes in the buffer.
 */
WS_DLL_PUBLIC char *oct_to_str_back(char *ptr, guint32 value);

/**
 * oct64_to_str_back()
 *
 * Output guint64 octal representation backward (last character will be written on ptr - 1),
 * and return pointer to first character.
 *
 * String is not NUL terminated by this routine.
 * There needs to be at least 12 bytes in the buffer.
 */
WS_DLL_PUBLIC char *oct64_to_str_back(char *ptr, guint64 value);

/**
 * hex_to_str_back()
 *
 * Output guint32 hex representation backward (last character will be written on ptr - 1),
 * and return pointer to first character.
 * This routine will output for sure (can output more) 'len' decimal characters (number padded with '0').
 *
 * String is not NUL terminated by this routine.
 * There needs to be at least 2 + MAX(8, len) bytes in the buffer.
 */
WS_DLL_PUBLIC char *hex_to_str_back_len(char *ptr, guint32 value, int len);

/**
 * hex64_to_str_back()
 *
 * Output guint64 hex representation backward (last character will be written on ptr - 1),
 * and return pointer to first character.
 * This routine will output for sure (can output more) 'len' decimal characters (number padded with '0').
 *
 * String is not NUL terminated by this routine.
 * There needs to be at least 2 + MAX(16, len) bytes in the buffer.
 */
WS_DLL_PUBLIC char *hex64_to_str_back_len(char *ptr, guint64 value, int len);

/**
 * uint_to_str_back()
 *
 * Output guint32 decimal representation backward (last character will be written on ptr - 1),
 * and return pointer to first character.
 *
 * String is not NUL terminated by this routine.
 * There needs to be at least 10 bytes in the buffer.
 */
WS_DLL_PUBLIC char *uint_to_str_back(char *ptr, guint32 value);

/**
 * uint64_str_back()
 *
 * Output guint64 decimal representation backward (last character will be written on ptr - 1),
 * and return pointer to first character.
 *
 * String is not NUL terminated by this routine.
 * There needs to be at least 20 bytes in the buffer.
 */
WS_DLL_PUBLIC char *uint64_to_str_back(char *ptr, guint64 value);

/**
 * uint_to_str_back_len()
 *
 * Output guint32 decimal representation backward (last character will be written on ptr - 1),
 * and return pointer to first character.
 * This routine will output for sure (can output more) 'len' decimal characters (number padded with '0').
 *
 * String is not NUL terminated by this routine.
 * There needs to be at least MAX(10, len) bytes in the buffer.
 */
WS_DLL_PUBLIC char *uint_to_str_back_len(char *ptr, guint32 value, int len);

/**
 * uint64_to_str_back_len()
 *
 * Output guint64 decimal representation backward (last character will be written on ptr - 1),
 * and return pointer to first character.
 * This routine will output for sure (can output more) 'len' decimal characters (number padded with '0').
 *
 * String is not NUL terminated by this routine.
 * There needs to be at least MAX(20, len) bytes in the buffer.
 */
WS_DLL_PUBLIC char *uint64_to_str_back_len(char *ptr, guint64 value, int len);

/**
 * int_to_str_back()
 *
 * Output gint32 decimal representation backward (last character will be written on ptr - 1),
 * and return pointer to first character.
 *
 * String is not NUL terminated by this routine.
 * There needs to be at least 11 bytes in the buffer.
 */
WS_DLL_PUBLIC char *int_to_str_back(char *ptr, gint32 value);

/**
 * int64_to_str_back()
 *
 * Output gint64 decimal representation backward (last character will be written on ptr - 1),
 * and return pointer to first character.
 *
 * String is not NUL terminated by this routine.
 * There needs to be at least 21 bytes in the buffer.
 */
WS_DLL_PUBLIC char *int64_to_str_back(char *ptr, gint64 value);

WS_DLL_PUBLIC void guint32_to_str_buf(guint32 u, gchar *buf, int buf_len);

WS_DLL_PUBLIC void guint64_to_str_buf(guint64 u, gchar *buf, int buf_len);

WS_DLL_PUBLIC void ip_to_str_buf(const guint8 *ad, gchar *buf, const int buf_len);

WS_DLL_PUBLIC char *ip_to_str(wmem_allocator_t *scope, const guint8 *ad);

/* Returns length of the result. */
WS_DLL_PUBLIC void ip6_to_str_buf(const ws_in6_addr *ad, gchar *buf, size_t buf_size);

WS_DLL_PUBLIC char *ip6_to_str(wmem_allocator_t *scope, const ws_in6_addr *ad);

WS_DLL_PUBLIC gchar *ipxnet_to_str_punct(wmem_allocator_t *scope, const guint32 ad, const char punct);

WS_DLL_PUBLIC gchar *eui64_to_str(wmem_allocator_t *scope, const guint64 ad);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __TO_STR_H__  */
