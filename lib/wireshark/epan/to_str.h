/* to_str.h
 * Definitions for utilities to convert various other types to strings.
 *
 * Wireshark - Network traffic analyzer
 * By Gerald Combs <gerald@wireshark.org>
 * Copyright 1998 Gerald Combs
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef __TO_STR_H__
#define __TO_STR_H__

#include <glib.h>

#include "wsutil/nstime.h"
#include <wsutil/inet_addr.h>
#include "time_fmt.h"
#include <epan/packet_info.h>
#include <epan/ipv6.h>
#include "ws_symbol_export.h"
#include <epan/wmem_scopes.h>
#include <wsutil/to_str.h>

#define GUID_STR_LEN     37
#define MAX_ADDR_STR_LEN 256
#define VINES_ADDR_LEN   6
#define EUI64_STR_LEN    24
#define AX25_ADDR_LEN    7
#define FCWWN_ADDR_LEN   8

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*
 * These are utility functions which convert various types to strings,
 * but for which no more specific module applies.
 */

/*
 ************** Address
 */

WS_DLL_PUBLIC gchar *address_to_str(wmem_allocator_t *scope, const address *addr);

WS_DLL_PUBLIC gchar *address_with_resolution_to_str(wmem_allocator_t *scope, const address *addr);

/*
 * address_to_name takes as input an "address", as defined in address.h.
 *
 * If the address is of a type that can be translated into a name, and the
 * user has activated name resolution, and the name can be resolved, it
 * returns a string containing the translated name.
 *
 * Otherwise, it returns NULL.
 */
WS_DLL_PUBLIC const gchar *address_to_name(const address *addr);

/*
 * address_to_display takes as input an "address", as defined in address.h .
 *
 * If the address is of a type that can be translated into a name, and the
 * user has activated name resolution, and the name can be resolved, it
 * returns a string containing the translated name.
 *
 * Otherwise, if the address is of type AT_NONE, it returns "NONE".
 *
 * Otherwise, it returns a string containing the result of address_to_str
 * on the argument, which should be a string representation for the address,
 * e.g. "10.10.10.10" for IPv4 address 10.10.10.10.
 */
WS_DLL_PUBLIC gchar *address_to_display(wmem_allocator_t *allocator, const address *addr);

WS_DLL_PUBLIC void address_to_str_buf(const address *addr, gchar *buf, int buf_len);

WS_DLL_PUBLIC const gchar *port_type_to_str (port_type type);

/*
 ************** TVB
 */

WS_DLL_PUBLIC gchar* tvb_address_with_resolution_to_str(wmem_allocator_t *scope, tvbuff_t *tvb, int type, const gint offset);

#define tvb_ether_to_str(scope, tvb, offset) tvb_address_to_str(scope, tvb, AT_ETHER, offset)

#define tvb_ip_to_str(scope, tvb, offset) tvb_address_to_str(scope, tvb, AT_IPv4, offset)

#define tvb_ip6_to_str(scope, tvb, offset) tvb_address_to_str(scope, tvb, AT_IPv6, offset)

#define tvb_fcwwn_to_str(scope, tvb, offset) tvb_address_to_str(scope, tvb, AT_FCWWN, offset)

#define tvb_fc_to_str(scope, tvb, offset) tvb_address_to_str(scope, tvb, AT_FC, offset)

#define tvb_eui64_to_str(scope, tvb, offset) tvb_address_to_str(scope, tvb, AT_EUI64, offset)

/** Turn an address type retrieved from a tvb into a string.
 *
 * @param scope memory allocation scheme used
 * @param tvb tvbuff to retrieve address
 * @param type address type to retrieve
 * @param offset offset into tvb to retrieve address
 * @return A pointer to the formatted string
 *
 */
WS_DLL_PUBLIC gchar* tvb_address_to_str(wmem_allocator_t *scope, tvbuff_t *tvb, int type, const gint offset);

/** Turn an address type retrieved from a tvb into a string.
 *
 * @param scope memory allocation scheme used
 * @param tvb tvbuff to retrieve address
 * @param type address type to retrieve
 * @param offset offset into tvb to retrieve address
 * @param length The length of the string
 * @return A pointer to the formatted string
 *
 */
WS_DLL_PUBLIC gchar* tvb_address_var_to_str(wmem_allocator_t *scope, tvbuff_t *tvb, address_type type, const gint offset, int length);

/*
 ************** Time
 */

WS_DLL_PUBLIC gchar *abs_time_to_str(wmem_allocator_t *scope, const nstime_t*, const absolute_time_display_e fmt,
                                                                                    gboolean show_zone);

WS_DLL_PUBLIC gchar *abs_time_secs_to_str(wmem_allocator_t *scope, const time_t, const absolute_time_display_e fmt,
                                                                                    gboolean show_zone);

WS_DLL_PUBLIC void display_epoch_time(gchar *, int, const time_t, gint32, const to_str_time_res_t);

WS_DLL_PUBLIC void display_signed_time(gchar *, int, const gint64, gint32, const to_str_time_res_t);

WS_DLL_PUBLIC gchar *signed_time_secs_to_str(wmem_allocator_t *scope, const gint32 time_val);

WS_DLL_PUBLIC gchar *unsigned_time_secs_to_str(wmem_allocator_t *scope, const guint32);

WS_DLL_PUBLIC gchar *signed_time_msecs_to_str(wmem_allocator_t *scope, gint32 time_val);

WS_DLL_PUBLIC gchar *rel_time_to_str(wmem_allocator_t *scope, const nstime_t *);

WS_DLL_PUBLIC gchar *rel_time_to_secs_str(wmem_allocator_t *scope, const nstime_t *);

/*
 ************** Misc
 */

WS_DLL_PUBLIC gchar *guid_to_str_buf(const e_guid_t *, gchar *, int);

WS_DLL_PUBLIC gchar *guid_to_str(wmem_allocator_t *scope, const e_guid_t *);

WS_DLL_PUBLIC char *decode_bits_in_field(wmem_allocator_t *scope, const guint bit_offset, const gint no_of_bits, const guint64 value, const guint encoding);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __TO_STR_H__  */
