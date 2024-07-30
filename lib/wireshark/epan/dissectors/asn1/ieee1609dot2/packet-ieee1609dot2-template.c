/* packet-IEEE1609dot2.c
 * Routines for IEEE 1609.2
 * Copyright 2018, Anders Broman <anders.broman@ericsson.com>
 *
 * Wireshark - Network traffic analyzer
 * By Gerald Combs <gerald@wireshark.org>
 * Copyright 1998 Gerald Combs
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/* Also contains IEEE std 1609.12
 * section 4.1.3 PSID allocations
 */

#include "config.h"

#include <stdlib.h>
#include <time.h>

#include <epan/packet.h>
#include <epan/conversation.h>
#include <epan/oids.h>
#include <epan/asn1.h>
#include <epan/proto_data.h>

#include "packet-oer.h"
#include "packet-ieee1609dot2.h"

#define PNAME  "IEEE1609dot2"
#define PSNAME "IEEE1609dot2"
#define PFNAME "ieee1609dot2"

void proto_register_ieee1609dot2(void);
void proto_reg_handoff_ieee1609dot2(void);

/* Initialize the protocol and registered fields */
int proto_ieee1609dot2 = -1;
dissector_handle_t proto_ieee1609dot2_handle = NULL;
#include "packet-ieee1609dot2-hf.c"

/* Initialize the subtree pointers */
static int ett_ieee1609dot2_ssp = -1;
#include "packet-ieee1609dot2-ett.c"

static dissector_table_t unsecured_data_subdissector_table;
static dissector_table_t ssp_subdissector_table;

typedef struct ieee1609_private_data {
  tvbuff_t *unsecured_data;
  guint64 psidssp; // psid for Service Specific Permissions
} ieee1609_private_data_t;

void
ieee1609dot2_set_next_default_psid(packet_info *pinfo, guint32 psid)
{
  p_add_proto_data(wmem_file_scope(), pinfo, proto_ieee1609dot2, 0, GUINT_TO_POINTER(psid));
}

#include "packet-ieee1609dot2-fn.c"


static void
ieee1609dot2_NinetyDegreeInt_fmt(gchar *s, guint32 v)
{
  gint32 lat = (gint32)v;
  if (lat == 900000001) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable(%d)", lat);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%u°%u'%.3f\"%c (%d)",
               abs(lat) / 10000000,
               abs(lat) % 10000000 * 6 / 1000000,
               abs(lat) % 10000000 * 6 % 1000000 * 6.0 / 100000.0,
               (lat >= 0) ? 'N' : 'S',
               lat);
  }
}

static void
ieee1609dot2_OneEightyDegreeInt_fmt(gchar *s, guint32 v)
{
  gint32 lng = (gint32)v;
  if (lng == 1800000001) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable(%d)", lng);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%u°%u'%.3f\"%c (%d)",
               abs(lng) / 10000000,
               abs(lng) % 10000000 * 6 / 1000000,
               abs(lng) % 10000000 * 6 % 1000000 * 6.0 / 100000.0,
               (lng >= 0) ? 'E' : 'W',
               lng);
  }
}

static void
ieee1609dot2_ElevInt_fmt(gchar *s, guint32 v)
{
  // Range is from -4096 to 61439 in units of one-tenth of a meter
  gint32 alt = (gint32)v - 4096;
  g_snprintf(s, ITEM_LABEL_LENGTH, "%.2fm (%u)", alt * 0.1, v);
}

static void
ieee1609dot2_Time32_fmt(gchar *s, guint32 v)
{
  time_t secs = v + 1072915200 - 5;
  struct tm *tm = gmtime(&secs);
  g_snprintf(s, ITEM_LABEL_LENGTH, "%u-%02u-%02u %02u:%02u:%02u (%u)",
    tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec, v
  );
}

static void
ieee1609dot2_Time64_fmt(gchar *s, guint64 v)
{
  time_t secs = v / 1000000 + 1072915200 - 5;
  guint32 usecs = v % 1000000;
  struct tm *tm = gmtime(&secs);
  g_snprintf(s, ITEM_LABEL_LENGTH, "%u-%02u-%02u %02u:%02u:%02u.%06u (%" G_GUINT64_FORMAT ")",
    tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec, usecs, v
  );
}

/*--- proto_register_ieee1609dot2 ----------------------------------------------*/
void proto_register_ieee1609dot2(void) {

  /* List of fields */
  static hf_register_info hf[] = {
#include "packet-ieee1609dot2-hfarr.c"
  };

  /* List of subtrees */
  static gint *ett[] = {
#include "packet-ieee1609dot2-ettarr.c"
        &ett_ieee1609dot2_ssp,
  };

  /* Register protocol */
  proto_ieee1609dot2 = proto_register_protocol(PNAME, PSNAME, PFNAME);

  /* Register fields and subtrees */
  proto_register_field_array(proto_ieee1609dot2, hf, array_length(hf));
  proto_register_subtree_array(ett, array_length(ett));

  proto_ieee1609dot2_handle = register_dissector("ieee1609dot2.data", dissect_Ieee1609Dot2Data_PDU, proto_ieee1609dot2);

  // See TS17419_ITS-AID_AssignedNumbers
  unsecured_data_subdissector_table = register_dissector_table("ieee1609dot2.psid",
        "ATS-AID/PSID based dissector for unsecured/signed data", proto_ieee1609dot2, FT_UINT32, BASE_HEX);
  ssp_subdissector_table = register_dissector_table("ieee1609dot2.ssp",
        "ATS-AID/PSID based dissector for Service Specific Permissions (SSP)", proto_ieee1609dot2, FT_UINT32, BASE_HEX);
}

void proto_reg_handoff_ieee1609dot2(void) {
    dissector_add_string("media_type", "application/x-its", proto_ieee1609dot2_handle);
    dissector_add_string("media_type", "application/x-its-request", proto_ieee1609dot2_handle);
    dissector_add_string("media_type", "application/x-its-response", proto_ieee1609dot2_handle);

    //dissector_add_uint_range_with_preference("udp.port", "56000,56001", proto_ieee1609dot2_handle);

}
