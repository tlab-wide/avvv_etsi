/* packet-its-template.c
 *
 * Intelligent Transport Systems Applications dissectors
 * Coyright 2018, C. Guerber <cguerber@yahoo.com>
 *
 * Wireshark - Network traffic analyzer
 * By Gerald Combs <gerald@wireshark.org>
 * Copyright 1998 Gerald Combs
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */


/*
 * Implemented:
 * CA (CAM)                           ETSI EN 302 637-2   V1.4.1 (2019-01)
 * DEN (DENM)                         ETSI EN 302 637-3   V1.3.0 (2018-08)
 * RLT (MAPEM)                        ETSI TS 103 301     V1.2.1 (2018-08)
 * TLM (SPATEM)                       ETSI TS 103 301     V1.2.1 (2018-08)
 * IVI (IVIM)                         ETSI TS 103 301     V1.2.1 (2018-08)
 * TLC (SREM)                         ETSI TS 103 301     V1.2.1 (2018-08)
 * TLC (SSEM)                         ETSI TS 103 301     V1.2.1 (2018-08)
 * EVCSN POI (EVCSN POI message)      ETSI TS 101 556-1
 * TPG (TRM, TCM, VDRM, VDPM, EOFM)   ETSI TS 101 556-2
 * Charging (EV-RSR, SRM, SCM)        ETSI TS 101 556-3
 * GPC (RTCMEM)                       ETSI TS 103 301
 *
 * Not supported:
 * SA (SAEM)                          ETSI TS 102 890-1
 * CTL (CTLM)                         ETSI TS 102 941
 * CRL (CRLM)                         ETSI TS 102 941
 * Certificate request                ETSI TS 102 941
 */
#include "config.h"

#include <math.h>
#include <epan/packet.h>
#include <epan/expert.h>
#include <epan/decode_as.h>
#include <epan/proto_data.h>
#include <epan/exceptions.h>
#include <epan/conversation.h>
#include <epan/tap.h>
#include <wsutil/utf8_entities.h>
#include "packet-ber.h"
#include "packet-per.h"

#include "packet-its.h"
#include "packet-ieee1609dot2.h"

/*
 * Well Known Ports definitions as per:
 *
 * ETSI TS 103 248 v1.2.1 (2018-08)
 * Intelligent Transport Systems (ITS);
 * GeoNetworking;
 * Port Numbers for the Basic Transport Protocol (BTP)
 *
 * BTP port   Facilities service      Related standard
 * number     or Application
 * values
 * 2001       CA (CAM)                ETSI EN 302 637-2  V1.4.1 (2019-01)
 * 2002       DEN (DENM)              ETSI EN 302 637-3
 * 2003       RLT (MAPEM)             ETSI TS 103 301     V1.2.1 (2018-08)
 * 2004       TLM (SPATEM)            ETSI TS 103 301     V1.2.1 (2018-08)
 * 2005       SA (SAEM)               ETSI TS 102 890-1
 * 2006       IVI (IVIM)              ETSI TS 103 301     V1.2.1 (2018-08)
 * 2007       TLC (SREM)              ETSI TS 103 301     V1.2.1 (2018-08)
 * 2008       TLC (SSEM)              ETSI TS 103 301     V1.2.1 (2018-08)
 * 2009       Allocated               Allocated for "Intelligent Transport
 *                                    System (ITS); Vehicular Communications;
 *                                    Basic Set of Applications; Specification
 *                                    of the Collective Perception Service"
 * 2010       EVCSN POI (EVCSN POI    ETSI TS 101 556-1
 *            message)
 * 2011       TPG (TRM, TCM, VDRM,    ETSI TS 101 556-2
 *            VDPM, EOFM)
 * 2012       Charging (EV-RSR,       ETSI TS 101 556-3
 *            SRM, SCM)
 * 2013       GPC (RTCMEM)            ETSI TS 103 301     V1.2.1 (2018-08)
 * 2014       CTL (CTLM)              ETSI TS 102 941
 * 2015       CRL (CRLM)              ETSI TS 102 941
 * 2016       Certificate request     ETSI TS 102 941
 */

// Applications Well Known Ports
#define ITS_WKP_CA         2001
#define ITS_WKP_DEN        2002
#define ITS_WKP_RLT        2003
#define ITS_WKP_TLM        2004
#define ITS_WKP_SA         2005
#define ITS_WKP_IVI        2006
#define ITS_WKP_TLC_SREM   2007
#define ITS_WKP_TLC_SSEM   2008
#define ITS_WKP_CPS        2009
#define ITS_WKP_EVCSN      2010
#define ITS_WKP_TPG        2011
#define ITS_WKP_CHARGING   2012
#define ITS_WKP_GPC        2013
#define ITS_WKP_CTL        2014
#define ITS_WKP_CRL        2015
#define ITS_WKP_CERTIF_REQ 2016

/*
 * Prototypes
 */
void proto_reg_handoff_its(void);
void proto_register_its(void);

static expert_field ei_its_no_sub_dis = EI_INIT;

// TAP
static int its_tap = -1;

// Protocols
static int proto_its = -1;
static int proto_its_denm = -1;
static int proto_its_denmv1 = -1;
static int proto_its_cam = -1;
static int proto_its_camv1 = -1;
static int proto_its_evcsn = -1;
static int proto_its_evrsr = -1;
static int proto_its_ivimv1 = -1;
static int proto_its_ivim = -1;
static int proto_its_tistpg = -1;
static int proto_its_ssem = -1;
static int proto_its_srem = -1;
static int proto_its_rtcmem = -1;
static int proto_its_mapemv1 = -1;
static int proto_its_mapem = -1;
static int proto_its_spatemv1 = -1;
static int proto_its_spatem = -1;
static int proto_its_cpm = -1;
static int proto_addgrpc = -1;

/*
 * DENM SSP
 */
static int hf_denmssp_version = -1;
static int hf_denmssp_flags = -1;
static int hf_denmssp_trafficCondition = -1;
static int hf_denmssp_accident = -1;
static int hf_denmssp_roadworks = -1;
static int hf_denmssp_adverseWeatherConditionAdhesion = -1;
static int hf_denmssp_hazardousLocationSurfaceCondition = -1;
static int hf_denmssp_hazardousLocationObstacleOnTheRoad = -1;
static int hf_denmssp_hazardousLocationAnimalOnTheRoad = -1;
static int hf_denmssp_humanPresenceOnTheRoad = -1;
static int hf_denmssp_wrongWayDriving = -1;
static int hf_denmssp_rescueAndRecoveryWorkInProgress = -1;
static int hf_denmssp_ExtremeWeatherCondition = -1;
static int hf_denmssp_adverseWeatherConditionVisibility = -1;
static int hf_denmssp_adverseWeatherConditionPrecipitation = -1;
static int hf_denmssp_slowVehicle = -1;
static int hf_denmssp_dangerousEndOfQueue = -1;
static int hf_denmssp_vehicleBreakdown = -1;
static int hf_denmssp_postCrash = -1;
static int hf_denmssp_humanProblem = -1;
static int hf_denmssp_stationaryVehicle = -1;
static int hf_denmssp_emergencyVehicleApproaching = -1;
static int hf_denmssp_hazardousLocationDangerousCurve = -1;
static int hf_denmssp_collisionRisk = -1;
static int hf_denmssp_signalViolation = -1;
static int hf_denmssp_dangerousSituation = -1;

/*
 * CAM SSP
 */
static int hf_camssp_version = -1;
static int hf_camssp_flags = -1;
static int hf_camssp_cenDsrcTollingZone = -1;
static int hf_camssp_publicTransport = -1;
static int hf_camssp_specialTransport = -1;
static int hf_camssp_dangerousGoods = -1;
static int hf_camssp_roadwork = -1;
static int hf_camssp_rescue = -1;
static int hf_camssp_emergency = -1;
static int hf_camssp_safetyCar = -1;
static int hf_camssp_closedLanes = -1;
static int hf_camssp_requestForRightOfWay = -1;
static int hf_camssp_requestForFreeCrossingAtATrafficLight = -1;
static int hf_camssp_noPassing = -1;
static int hf_camssp_noPassingForTrucks = -1;
static int hf_camssp_speedLimit = -1;
static int hf_camssp_reserved = -1;

static gint ett_denmssp_flags = -1;
static gint ett_camssp_flags = -1;

// Subdissectors
static dissector_table_t its_version_subdissector_table;
static dissector_table_t its_msgid_subdissector_table;
static dissector_table_t regionid_subdissector_table;
static dissector_table_t cam_pt_activation_table;

typedef struct its_private_data {
    enum regext_type_enum type;
    guint32 region_id;
    guint32 cause_code;
} its_private_data_t;

typedef struct its_pt_activation_data {
    guint32 type;
    tvbuff_t *data;
} its_pt_activation_data_t;

// Specidic dissector for content of open type for regional extensions
static int dissect_regextval_pdu(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_)
{
    its_private_data_t *re = (its_private_data_t*)data;
    // XXX What to do when region_id = noRegion? Test length is zero?
    if (!dissector_try_uint_new(regionid_subdissector_table, ((guint32) re->region_id<<16) + (guint32) re->type, tvb, pinfo, tree, FALSE, NULL))
        call_data_dissector(tvb, pinfo, tree);
    return tvb_captured_length(tvb);
}

static int dissect_denmssp_pdu(tvbuff_t *tvb, packet_info *pinfo _U_, proto_tree *tree, void *data _U_)
{
    static int * const denmssp_flags[] = {
        &hf_denmssp_trafficCondition,
        &hf_denmssp_accident,
        &hf_denmssp_roadworks,
        &hf_denmssp_adverseWeatherConditionAdhesion,
        &hf_denmssp_hazardousLocationSurfaceCondition,
        &hf_denmssp_hazardousLocationObstacleOnTheRoad,
        &hf_denmssp_hazardousLocationAnimalOnTheRoad,
        &hf_denmssp_humanPresenceOnTheRoad,
        &hf_denmssp_wrongWayDriving,
        &hf_denmssp_rescueAndRecoveryWorkInProgress,
        &hf_denmssp_ExtremeWeatherCondition,
        &hf_denmssp_adverseWeatherConditionVisibility,
        &hf_denmssp_adverseWeatherConditionPrecipitation,
        &hf_denmssp_slowVehicle,
        &hf_denmssp_dangerousEndOfQueue,
        &hf_denmssp_vehicleBreakdown,
        &hf_denmssp_postCrash,
        &hf_denmssp_humanProblem,
        &hf_denmssp_stationaryVehicle,
        &hf_denmssp_emergencyVehicleApproaching,
        &hf_denmssp_hazardousLocationDangerousCurve,
        &hf_denmssp_collisionRisk,
        &hf_denmssp_signalViolation,
        &hf_denmssp_dangerousSituation,
        NULL
    };

    guint32 version;

    proto_tree_add_item_ret_uint(tree, hf_denmssp_version, tvb, 0, 1, ENC_BIG_ENDIAN, &version);
    if (version == 1) {
        proto_tree_add_bitmask(tree, tvb, 1, hf_denmssp_flags, ett_denmssp_flags, denmssp_flags, ENC_BIG_ENDIAN);
    }
    return tvb_reported_length(tvb);
}

static int dissect_camssp_pdu(tvbuff_t *tvb, packet_info *pinfo _U_, proto_tree *tree, void *data _U_)
{
    static int * const camssp_flags[] = {
        &hf_camssp_cenDsrcTollingZone,
        &hf_camssp_publicTransport,
        &hf_camssp_specialTransport,
        &hf_camssp_dangerousGoods,
        &hf_camssp_roadwork,
        &hf_camssp_rescue,
        &hf_camssp_emergency,
        &hf_camssp_safetyCar,
        &hf_camssp_closedLanes,
        &hf_camssp_requestForRightOfWay,
        &hf_camssp_requestForFreeCrossingAtATrafficLight,
        &hf_camssp_noPassing,
        &hf_camssp_noPassingForTrucks,
        &hf_camssp_speedLimit,
        &hf_camssp_reserved,
        NULL
    };

    guint32 version;

    proto_tree_add_item_ret_uint(tree, hf_camssp_version, tvb, 0, 1, ENC_BIG_ENDIAN, &version);
    if (version == 1) {
        proto_tree_add_bitmask(tree, tvb, 1, hf_camssp_flags, ett_camssp_flags, camssp_flags, ENC_BIG_ENDIAN);
    }
    return tvb_reported_length(tvb);
}

// Generated by asn2wrs
#include "packet-its-hf.c"

// CauseCode/SubCauseCode management
static int hf_its_trafficConditionSubCauseCode = -1;
static int hf_its_accidentSubCauseCode = -1;
static int hf_its_roadworksSubCauseCode = -1;
static int hf_its_adverseWeatherCondition_PrecipitationSubCauseCode = -1;
static int hf_its_adverseWeatherCondition_VisibilitySubCauseCode = -1;
static int hf_its_adverseWeatherCondition_AdhesionSubCauseCode = -1;
static int hf_its_adverseWeatherCondition_ExtremeWeatherConditionSubCauseCode = -1;
static int hf_its_hazardousLocation_AnimalOnTheRoadSubCauseCode = -1;
static int hf_its_hazardousLocation_ObstacleOnTheRoadSubCauseCode = -1;
static int hf_its_hazardousLocation_SurfaceConditionSubCauseCode = -1;
static int hf_its_hazardousLocation_DangerousCurveSubCauseCode = -1;
static int hf_its_humanPresenceOnTheRoadSubCauseCode = -1;
static int hf_its_wrongWayDrivingSubCauseCode = -1;
static int hf_its_rescueAndRecoveryWorkInProgressSubCauseCode = -1;
static int hf_its_slowVehicleSubCauseCode = -1;
static int hf_its_dangerousEndOfQueueSubCauseCode = -1;
static int hf_its_vehicleBreakdownSubCauseCode = -1;
static int hf_its_postCrashSubCauseCode = -1;
static int hf_its_humanProblemSubCauseCode = -1;
static int hf_its_stationaryVehicleSubCauseCode = -1;
static int hf_its_emergencyVehicleApproachingSubCauseCode = -1;
static int hf_its_collisionRiskSubCauseCode = -1;
static int hf_its_signalViolationSubCauseCode = -1;
static int hf_its_dangerousSituationSubCauseCode = -1;

static gint ett_its = -1;

#include "packet-its-ett.c"

// Deal with cause/subcause code management
struct { CauseCodeType_enum cause; int* hf; } cause_to_subcause[] = {
    { trafficCondition, &hf_its_trafficConditionSubCauseCode },
    { accident, &hf_its_accidentSubCauseCode },
    { roadworks, &hf_its_roadworksSubCauseCode },
    { adverseWeatherCondition_Precipitation, &hf_its_adverseWeatherCondition_PrecipitationSubCauseCode },
    { adverseWeatherCondition_Visibility, &hf_its_adverseWeatherCondition_VisibilitySubCauseCode },
    { adverseWeatherCondition_Adhesion, &hf_its_adverseWeatherCondition_AdhesionSubCauseCode },
    { adverseWeatherCondition_ExtremeWeatherCondition, &hf_its_adverseWeatherCondition_ExtremeWeatherConditionSubCauseCode },
    { hazardousLocation_AnimalOnTheRoad, &hf_its_hazardousLocation_AnimalOnTheRoadSubCauseCode },
    { hazardousLocation_ObstacleOnTheRoad, &hf_its_hazardousLocation_ObstacleOnTheRoadSubCauseCode },
    { hazardousLocation_SurfaceCondition, &hf_its_hazardousLocation_SurfaceConditionSubCauseCode },
    { hazardousLocation_DangerousCurve, &hf_its_hazardousLocation_DangerousCurveSubCauseCode },
    { humanPresenceOnTheRoad, &hf_its_humanPresenceOnTheRoadSubCauseCode },
    { wrongWayDriving, &hf_its_wrongWayDrivingSubCauseCode },
    { rescueAndRecoveryWorkInProgress, &hf_its_rescueAndRecoveryWorkInProgressSubCauseCode },
    { slowVehicle, &hf_its_slowVehicleSubCauseCode },
    { dangerousEndOfQueue, &hf_its_dangerousEndOfQueueSubCauseCode },
    { vehicleBreakdown, &hf_its_vehicleBreakdownSubCauseCode },
    { postCrash, &hf_its_postCrashSubCauseCode },
    { humanProblem, &hf_its_humanProblemSubCauseCode },
    { stationaryVehicle, &hf_its_stationaryVehicleSubCauseCode },
    { emergencyVehicleApproaching, &hf_its_emergencyVehicleApproachingSubCauseCode },
    { collisionRisk, &hf_its_collisionRiskSubCauseCode },
    { signalViolation, &hf_its_signalViolationSubCauseCode },
    { dangerousSituation, &hf_its_dangerousSituationSubCauseCode },
    { reserved, NULL },
};

static int*
find_subcause_from_cause(CauseCodeType_enum cause)
{
    int idx = 0;

    while (cause_to_subcause[idx].hf && (cause_to_subcause[idx].cause != cause))
        idx++;

    return cause_to_subcause[idx].hf?cause_to_subcause[idx].hf:&hf_its_subCauseCode;
}

#include "packet-its-fn.c"

static void
its_latitude_fmt(gchar *s, guint32 v)
{
  gint32 lat = (gint32)v;
  if (lat == 900000001) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", lat);
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
its_longitude_fmt(gchar *s, guint32 v)
{
  gint32 lng = (gint32)v;
  if (lng == 1800000001) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", lng);
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
its_altitude_fmt(gchar *s, guint32 v)
{
  gint32 alt = (gint32)v;
  if (alt == 800001) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", alt);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%.2fm (%d)", alt * 0.01, alt);
  }
}

static void
its_delta_latitude_fmt(gchar *s, guint32 v)
{
  gint32 lat = (gint32)v;
  if (lat == 131072) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", lat);
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
its_delta_longitude_fmt(gchar *s, guint32 v)
{
  gint32 lng = (gint32)v;
  if (lng == 131072) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", lng);
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
its_delta_altitude_fmt(gchar *s, guint32 v)
{
  gint32 alt = (gint32)v;
  if (alt == 12800) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", alt);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%.2fm (%d)", alt * 0.01, alt);
  }
}

static void
its_path_delta_time_fmt(gchar *s, guint32 v)
{
  gint32 dt = (gint32)v;
  g_snprintf(s, ITEM_LABEL_LENGTH, "%.2fs (%d)", dt * 0.01, dt);
}


static void
its_sax_length_fmt(gchar *s, guint32 v)
{
  if (v == 4095) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", v);
  } else if (v == 4094) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "outOfRange (%d)", v);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%.2fm (%d)", v * 0.01, v);
  }
}

static void
its_heading_value_fmt(gchar *s, guint32 v)
{
  const gchar *p = try_val_to_str(v, VALS(its_HeadingValue_vals));
  if (p) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%s (%d)", p, v);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%.1f° (%d)", v * 0.1, v);
  }
}

static void
its_heading_confidence_fmt(gchar *s, guint32 v)
{
  if (v == 127) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", v);
  } else if (v == 126) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "outOfRange (%d)", v);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%.1f° (%d)", v * 0.1, v);
  }
}

static void
its_speed_value_fmt(gchar *s, guint32 v)
{
  if (v == 0) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "standstill (%d)", v);
  } else if (v == 16383) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", v);
  } else {
    double vms = v * 0.01;
    g_snprintf(s, ITEM_LABEL_LENGTH, "%.2fm/s = %.1fkm/h (%d)",
            vms, vms * 3.6, v);
  }
}

static void
its_speed_confidence_fmt(gchar *s, guint32 v)
{
  if (v == 127) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", v);
  } else if (v == 126) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "outOfRange (%d)", v);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%.2fm/s (%d)", v * 0.01, v);
  }
}

static void
its_vehicle_length_value_fmt(gchar *s, guint32 v)
{
  if (v == 1023) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", v);
  } else if (v == 1022) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "outOfRange (%d)", v);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%.1fm (%d)", v * 0.1, v);
  }
}

static void
its_vehicle_width_fmt(gchar *s, guint32 v)
{
  if (v == 62) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", v);
  } else if (v == 61) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "outOfRange (%d)", v);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%.1fm (%d)", v * 0.1, v);
  }
}

static void
its_acceleration_value_fmt(gchar *s, guint32 v)
{
  gint32 acc = (gint32)v;
  if (acc == 161) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", v);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%.1fm/s² (%d)", acc * 0.1, acc);
  }
}

static void
its_acceleration_confidence_fmt(gchar *s, guint32 v)
{
  if (v == 102) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", v);
  } else if (v == 101) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "outOfRange (%d)", v);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%.1fm/s² (%d)", v * 0.1, v);
  }
}

static void
its_curvature_value_fmt(gchar *s, guint32 v)
{
  gint32 curv = (gint32)v;
  if (curv == 0) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "straight (%d)", v);
  } else if (curv == 30001) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", v);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%.3fm %s (%d)",
               30000.0 / curv,
               (curv > 0) ? "left" : "right",
               curv);
  }
}

static void
its_yaw_rate_value_fmt(gchar *s, guint32 v)
{
  gint32 yaw = (gint32)v;
  if (yaw == 0) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "straight (%d)", v);
  } else if (yaw == 32767) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", v);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%.2f°/s %s (%d)",
               yaw * 0.01,
               (yaw > 0) ? "left" : "right",
               yaw);
  }
}

static void
its_swa_value_fmt(gchar *s, guint32 v)
{
  gint32 swa = (gint32)v;
  if (swa == 0) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "straight (%d)", v);
  } else if (swa == 512) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", v);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%.1f° %s (%d)",
               swa * 1.5,
               (swa > 0) ? "left" : "right",
               swa);
  }
}

static void
its_swa_confidence_fmt(gchar *s, guint32 v)
{
  if (v == 127) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", v);
  } else if (v == 126) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "outOfRange (%d)", v);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%.1f° (%d)", v * 1.5, v);
  }
}

static void
dsrc_moi_fmt(gchar *s, guint32 v)
{
  if (v == 527040) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "invalid (%d)", v);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%ud %02u:%02u (%d)",
            v / 1440, v % 1440 / 60, v % 60, v);
  }
}

static void
dsrc_dsecond_fmt(gchar *s, guint32 v)
{
  if (v == 65535) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", v);
  } else if ((61000 <= v) && (v <= 65534)) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "reserved (%d)", v);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%02u.%03u (%d)",
            v / 1000, v % 1000, v);
  }
}

static void
dsrc_time_mark_fmt(gchar *s, guint32 v)
{
  if (v == 36001) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unknown (%d)", v);
  } else if (v == 36000) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "moreThanHour (%d)", v);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%02u:%02u.%u (%d)",
            v / 600, v % 600 / 10, v % 10, v);
  }
}

static const value_string dsrc_TimeIntervalConfidence_vals[] = {
  {   0, "21% probability" },
  {   1, "36% probability" },
  {   2, "47% probability" },
  {   3, "56% probability" },
  {   4, "62% probability" },
  {   5, "68% probability" },
  {   6, "73% probability" },
  {   7, "77% probability" },
  {   8, "81% probability" },
  {   9, "85% probability" },
  {  10, "88% probability" },
  {  11, "91% probability" },
  {  12, "94% probability" },
  {  13, "96% probability" },
  {  14, "98% probability" },
  {  15, "10% probability" },
  { 0, NULL }
};

static void
dsrc_velocity_fmt(gchar *s, guint32 v)
{
  if (v == 8191) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", v);
  } else {
    double vms = v * 0.02;
    g_snprintf(s, ITEM_LABEL_LENGTH, "%.2fm/s = %ukm/h (%d)",
            vms, (int)lround(vms * 3.6), v);
  }
}

static void
dsrc_angle_fmt(gchar *s, guint32 v)
{
  g_snprintf(s, ITEM_LABEL_LENGTH, "%.2f° (%d)", v * 0.0125, v);
}

static void
dsrc_delta_time_fmt(gchar *s, guint32 v)
{
  gint32 dt = (gint32)v;
  if (dt == -122) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unknown (%d)", dt);
  } else if (dt == -121) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "moreThanMinus20Minutes (%d)", dt);
  } else if (dt == 121) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "moreThanPlus20Minutes (%d)", dt);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%s%d:%02u (%d)",
            (dt < 0) ? "-" : "", abs(dt) / 6, abs(dt) % 6 * 10, dt);
  }
}

static void
cpm_general_confidence_fmt(gchar *s, guint32 v)
{
  if (v == 0) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unknown (%u)", v);
  } else if (v == 101) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%u)", v);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%u%% (%u)", v, v);
  }
}

static void
cpm_distance_value_fmt(gchar *s, guint32 v)
{
  gint32 sv = (gint32)v;
  g_snprintf(s, ITEM_LABEL_LENGTH, "%.2fm (%d)", sv * 0.01, sv);
}

static void
cpm_distance_confidence_fmt(gchar *s, guint32 v)
{
  if (v == 102) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", v);
  } else if (v == 101) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "outOfRange (%d)", v);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%.2fm (%d)", v * 0.01, v);
  }
}

static void
cpm_speed_value_ext_fmt(gchar *s, guint32 v)
{
  gint32 sv = (gint32)v;
  if (sv == 0) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "standstill (%d)", sv);
  } else if (sv == 16383) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", sv);
  } else {
    double vms = sv * 0.01;
    g_snprintf(s, ITEM_LABEL_LENGTH, "%.2fm/s = %.1fkm/h (%d)",
            vms, vms * 3.6, sv);
  }
}

static void
cpm_cartesian_angle_value_fmt(gchar *s, guint32 v)
{
  if (v == 3601) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", v);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%.1f° (%d)", v * 0.1, v);
  }
}

static void
cpm_angle_confidence_fmt(gchar *s, guint32 v)
{
  if (v == 127) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", v);
  } else if (v == 126) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "outOfRange (%d)", v);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%.1f° (%d)", v * 0.1, v);
  }
}

static void
cpm_object_dimension_value_fmt(gchar *s, guint32 v)
{
  g_snprintf(s, ITEM_LABEL_LENGTH, "%.1fm (%d)", v * 0.1, v);
}

static void
cpm_object_dimension_confidence_fmt(gchar *s, guint32 v)
{
  if (v == 102) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "unavailable (%d)", v);
  } else if (v == 101) {
    g_snprintf(s, ITEM_LABEL_LENGTH, "outOfRange (%d)", v);
  } else {
    g_snprintf(s, ITEM_LABEL_LENGTH, "%.2fm (%d)", v * 0.01, v);
  }
}

static int
dissect_its_PDU(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data)
{
  proto_item *its_item;
  proto_tree *its_tree;

  col_set_str(pinfo->cinfo, COL_PROTOCOL, "ITS");
  col_clear(pinfo->cinfo, COL_INFO);

  its_item = proto_tree_add_item(tree, proto_its, tvb, 0, -1, ENC_NA);
  its_tree = proto_item_add_subtree(its_item, ett_its);

  return dissect_its_ItsPduHeader_PDU(tvb, pinfo, its_tree, data);
}

// Decode As...
static void
its_msgid_prompt(packet_info *pinfo, gchar *result)
{
    guint32 msgid = GPOINTER_TO_UINT(p_get_proto_data(pinfo->pool, pinfo, hf_its_messageID, pinfo->curr_layer_num));

    g_snprintf(result, MAX_DECODE_AS_PROMPT_LEN, "MsgId (%s%u)", UTF8_RIGHTWARDS_ARROW, msgid);
}

static gpointer
its_msgid_value(packet_info *pinfo)
{
    return p_get_proto_data(pinfo->pool, pinfo, hf_its_messageID, pinfo->curr_layer_num);
}

// Registration of protocols
void proto_register_its(void)
{
    static hf_register_info hf_its[] = {
        #include "packet-its-hfarr.c"

    { &hf_its_roadworksSubCauseCode,
      { "roadworksSubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_RoadworksSubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},
    { &hf_its_postCrashSubCauseCode,
      { "postCrashSubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_PostCrashSubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},
    { &hf_its_vehicleBreakdownSubCauseCode,
      { "vehicleBreakdownSubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_VehicleBreakdownSubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},
    { &hf_its_dangerousSituationSubCauseCode,
      { "dangerousSituationSubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_DangerousSituationSubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},
    { &hf_its_dangerousEndOfQueueSubCauseCode,
      { "dangerousEndOfQueueSubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_DangerousEndOfQueueSubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},
    { &hf_its_rescueAndRecoveryWorkInProgressSubCauseCode,
      { "rescueAndRecoveryWorkInProgressSubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_RescueAndRecoveryWorkInProgressSubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},
    { &hf_its_signalViolationSubCauseCode,
      { "signalViolationSubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_SignalViolationSubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},
    { &hf_its_collisionRiskSubCauseCode,
      { "collisionRiskSubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_CollisionRiskSubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},
    { &hf_its_hazardousLocation_AnimalOnTheRoadSubCauseCode,
      { "hazardousLocation_AnimalOnTheRoadSubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_HazardousLocation_AnimalOnTheRoadSubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},
    { &hf_its_hazardousLocation_ObstacleOnTheRoadSubCauseCode,
      { "hazardousLocation_ObstacleOnTheRoadSubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_HazardousLocation_ObstacleOnTheRoadSubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},
    { &hf_its_hazardousLocation_SurfaceConditionSubCauseCode,
      { "hazardousLocation_SurfaceConditionSubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_HazardousLocation_SurfaceConditionSubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},
    { &hf_its_hazardousLocation_DangerousCurveSubCauseCode,
      { "hazardousLocation_DangerousCurveSubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_HazardousLocation_DangerousCurveSubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},
    { &hf_its_emergencyVehicleApproachingSubCauseCode,
      { "emergencyVehicleApproachingSubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_EmergencyVehicleApproachingSubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},
    { &hf_its_humanProblemSubCauseCode,
      { "humanProblemSubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_HumanProblemSubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},
    { &hf_its_stationaryVehicleSubCauseCode,
      { "stationaryVehicleSubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_StationaryVehicleSubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},
    { &hf_its_slowVehicleSubCauseCode,
      { "slowVehicleSubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_SlowVehicleSubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},
    { &hf_its_adverseWeatherCondition_PrecipitationSubCauseCode,
      { "adverseWeatherCondition_PrecipitationSubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_AdverseWeatherCondition_PrecipitationSubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},
    { &hf_its_adverseWeatherCondition_VisibilitySubCauseCode,
      { "adverseWeatherCondition_VisibilitySubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_AdverseWeatherCondition_VisibilitySubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},
    { &hf_its_adverseWeatherCondition_AdhesionSubCauseCode,
      { "adverseWeatherCondition_AdhesionSubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_AdverseWeatherCondition_AdhesionSubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},
    { &hf_its_adverseWeatherCondition_ExtremeWeatherConditionSubCauseCode,
      { "adverseWeatherCondition_ExtremeWeatherConditionSubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_AdverseWeatherCondition_ExtremeWeatherConditionSubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},
    { &hf_its_wrongWayDrivingSubCauseCode,
      { "wrongWayDrivingSubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_WrongWayDrivingSubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},
    { &hf_its_humanPresenceOnTheRoadSubCauseCode,
      { "humanPresenceOnTheRoadSubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_HumanPresenceOnTheRoadSubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},
    { &hf_its_accidentSubCauseCode,
      { "accidentSubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_AccidentSubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},
    { &hf_its_trafficConditionSubCauseCode,
      { "trafficConditionSubCauseCode", "its.subCauseCode",
        FT_UINT32, BASE_DEC, VALS(its_TrafficConditionSubCauseCode_vals), 0,
        "SubCauseCodeType", HFILL }},

    /*
     * DENM SSP
     */
    { &hf_denmssp_version, { "Version", "its.ssp.denm.version", FT_UINT8, BASE_DEC, NULL, 0, NULL, HFILL }},
    { &hf_denmssp_flags, { "Allowed to sign", "its.ssp.denm.flags", FT_UINT24, BASE_HEX, NULL, 0, NULL, HFILL }},
    { &hf_denmssp_trafficCondition,
        { "trafficCondition",                     "its.denm.ssp.trafficCondition",
            FT_UINT24, BASE_DEC, NULL, 0x800000, NULL, HFILL }},
    { &hf_denmssp_accident,
        { "accident",                             "its.denm.ssp.accident",
            FT_UINT24, BASE_DEC, NULL, 0x400000, NULL, HFILL }},
    { &hf_denmssp_roadworks,
        { "roadworks",                            "its.denm.ssp.roadworks",
            FT_UINT24, BASE_DEC, NULL, 0x200000, NULL, HFILL }},
    { &hf_denmssp_adverseWeatherConditionAdhesion,
        { "adverseWeatherConditionAdhesion",      "its.denm.ssp.advWxConditionAdhesion",
            FT_UINT24, BASE_DEC, NULL, 0x100000, NULL, HFILL }},
    { &hf_denmssp_hazardousLocationSurfaceCondition,
        { "hazardousLocationSurfaceCondition",    "its.denm.ssp.hazLocationSurfaceCondition",
            FT_UINT24, BASE_DEC, NULL, 0x080000, NULL, HFILL }},
    { &hf_denmssp_hazardousLocationObstacleOnTheRoad,
        { "hazardousLocationObstacleOnTheRoad",   "its.denm.ssp.hazLocationObstacleOnTheRoad",
            FT_UINT24, BASE_DEC, NULL, 0x040000, NULL, HFILL }},
    { &hf_denmssp_hazardousLocationAnimalOnTheRoad,
        { "hazardousLocationAnimalOnTheRoad",     "its.denm.ssp.hazLocationAnimalOnTheRoad",
            FT_UINT24, BASE_DEC, NULL, 0x020000, NULL, HFILL }},
    { &hf_denmssp_humanPresenceOnTheRoad,
        { "humanPresenceOnTheRoad",               "its.denm.ssp.humanPresenceOnTheRoad",
            FT_UINT24, BASE_DEC, NULL, 0x010000, NULL, HFILL }},
    { &hf_denmssp_wrongWayDriving,
        { "wrongWayDriving",                      "its.denm.ssp.wrongWayDriving",
            FT_UINT24, BASE_DEC, NULL, 0x008000, NULL, HFILL }},
    { &hf_denmssp_rescueAndRecoveryWorkInProgress,
        { "rescueAndRecoveryWorkInProgress",      "its.denm.ssp.rescueAndRecoveryWorkInProgress",
            FT_UINT24, BASE_DEC, NULL, 0x004000, NULL, HFILL }},
    { &hf_denmssp_ExtremeWeatherCondition,
        { "ExtremeWeatherCondition",              "its.denm.ssp.ExtremeWxCondition",
            FT_UINT24, BASE_DEC, NULL, 0x002000, NULL, HFILL }},
    { &hf_denmssp_adverseWeatherConditionVisibility,
        { "adverseWeatherConditionVisibility",    "its.denm.ssp.advWxConditionVisibility",
            FT_UINT24, BASE_DEC, NULL, 0x001000, NULL, HFILL }},
    { &hf_denmssp_adverseWeatherConditionPrecipitation,
        { "adverseWeatherConditionPrecipitation", "its.denm.ssp.advWxConditionPrecipitation",
            FT_UINT24, BASE_DEC, NULL, 0x000800, NULL, HFILL }},
    { &hf_denmssp_slowVehicle,
        { "slowVehicle",                          "its.denm.ssp.slowVehicle",
            FT_UINT24, BASE_DEC, NULL, 0x000400, NULL, HFILL }},
    { &hf_denmssp_dangerousEndOfQueue,
        { "dangerousEndOfQueue",                  "its.denm.ssp.dangerousEndOfQueue",
            FT_UINT24, BASE_DEC, NULL, 0x000200, NULL, HFILL }},
    { &hf_denmssp_vehicleBreakdown,
        { "vehicleBreakdown",                     "its.denm.ssp.vehicleBreakdown",
            FT_UINT24, BASE_DEC, NULL, 0x000100, NULL, HFILL }},
    { &hf_denmssp_postCrash,
        { "postCrash",                            "its.denm.ssp.postCrash",
            FT_UINT24, BASE_DEC, NULL, 0x000080, NULL, HFILL }},
    { &hf_denmssp_humanProblem,
        { "humanProblem",                         "its.denm.ssp.humanProblem",
            FT_UINT24, BASE_DEC, NULL, 0x000040, NULL, HFILL }},
    { &hf_denmssp_stationaryVehicle,
        { "stationaryVehicle",                    "its.denm.ssp.stationaryVehicle",
            FT_UINT24, BASE_DEC, NULL, 0x000020, NULL, HFILL }},
    { &hf_denmssp_emergencyVehicleApproaching,
        { "emergencyVehicleApproaching",          "its.denm.ssp.emergencyVehicleApproaching",
            FT_UINT24, BASE_DEC, NULL, 0x000010, NULL, HFILL }},
    { &hf_denmssp_hazardousLocationDangerousCurve,
        { "hazardousLocationDangerousCurve",      "its.denm.ssp.hazLocationDangerousCurve",
            FT_UINT24, BASE_DEC, NULL, 0x000008, NULL, HFILL }},
    { &hf_denmssp_collisionRisk,
        { "collisionRisk",                        "its.denm.ssp.collisionRisk",
            FT_UINT24, BASE_DEC, NULL, 0x000004, NULL, HFILL }},
    { &hf_denmssp_signalViolation,
        { "signalViolation",                      "its.denm.ssp.signalViolation",
            FT_UINT24, BASE_DEC, NULL, 0x000002, NULL, HFILL }},
    { &hf_denmssp_dangerousSituation,
        { "dangerousSituation",                   "its.denm.ssp.dangerousSituation",
            FT_UINT24, BASE_DEC, NULL, 0x000001, NULL, HFILL }},

    /*
     * CAM SSP
     */
    { &hf_camssp_version, { "Version", "its.ssp.cam.version", FT_UINT8, BASE_DEC, NULL, 0, NULL, HFILL }},
    { &hf_camssp_flags, { "Allowed to sign", "its.ssp.cam.flags", FT_UINT16, BASE_HEX, NULL, 0, NULL, HFILL }},
    { &hf_camssp_cenDsrcTollingZone, { "cenDsrcTollingZone", "its.ssp.cam.cenDsrcTollingZone", FT_UINT16, BASE_DEC, NULL, 0x8000, NULL, HFILL }},
    { &hf_camssp_publicTransport, { "publicTransport", "its.ssp.cam.publicTransport", FT_UINT16, BASE_DEC, NULL, 0x4000, NULL, HFILL }},
    { &hf_camssp_specialTransport, { "specialTransport", "its.ssp.cam.specialTransport", FT_UINT16, BASE_DEC, NULL, 0x2000, NULL, HFILL }},
    { &hf_camssp_dangerousGoods, { "dangerousGoods", "its.ssp.cam.dangerousGoods", FT_UINT16, BASE_DEC, NULL, 0x1000, NULL, HFILL }},
    { &hf_camssp_roadwork, { "roadwork", "its.ssp.cam.roadwork", FT_UINT16, BASE_DEC, NULL, 0x0800, NULL, HFILL }},
    { &hf_camssp_rescue, { "rescue", "its.ssp.cam.rescue", FT_UINT16, BASE_DEC, NULL, 0x0400, NULL, HFILL }},
    { &hf_camssp_emergency, { "emergency", "its.ssp.cam.emergency", FT_UINT16, BASE_DEC, NULL, 0x0200, NULL, HFILL }},
    { &hf_camssp_safetyCar, { "safetyCar", "its.ssp.cam.safetyCar", FT_UINT16, BASE_DEC, NULL, 0x0100, NULL, HFILL }},
    { &hf_camssp_closedLanes, { "closedLanes", "its.ssp.cam.closedLanes", FT_UINT16, BASE_DEC, NULL, 0x0080, NULL, HFILL }},
    { &hf_camssp_requestForRightOfWay, { "requestForRightOfWay", "its.ssp.cam.requestForRightOfWay", FT_UINT16, BASE_DEC, NULL, 0x0040, NULL, HFILL }},
    { &hf_camssp_requestForFreeCrossingAtATrafficLight, { "reqFreeCrossTrafLight", "its.ssp.cam.requestForFreeCrossingAtATrafficLight", FT_UINT16, BASE_DEC, NULL, 0x0020, NULL, HFILL }},
    { &hf_camssp_noPassing, { "noPassing", "its.ssp.cam.noPassing", FT_UINT16, BASE_DEC, NULL, 0x0010, NULL, HFILL }},
    { &hf_camssp_noPassingForTrucks, { "noPassingForTrucks", "its.ssp.cam.noPassingForTrucks", FT_UINT16, BASE_DEC, NULL, 0x0008, NULL, HFILL }},
    { &hf_camssp_speedLimit, { "speedLimit", "its.ssp.cam.speedLimit", FT_UINT16, BASE_DEC, NULL, 0x0004, NULL, HFILL }},
    { &hf_camssp_reserved, { "reserved", "its.ssp.cam.reserved", FT_UINT16, BASE_DEC, NULL, 0x0003, NULL, HFILL }},
    };

    static gint *ett[] = {
        &ett_its,
        &ett_denmssp_flags,
        &ett_camssp_flags,
        #include "packet-its-ettarr.c"
    };

    static ei_register_info ei[] = {
    { &ei_its_no_sub_dis, { "its.no_subdissector", PI_PROTOCOL, PI_NOTE, "No subdissector found for this Message id/protocol version combination", EXPFILL }},
    };

    expert_module_t* expert_its;

    proto_its = proto_register_protocol("Intelligent Transport Systems", "ITS", "its");

    proto_register_field_array(proto_its, hf_its, array_length(hf_its));

    proto_register_subtree_array(ett, array_length(ett));

    expert_its = expert_register_protocol(proto_its);

    expert_register_field_array(expert_its, ei, array_length(ei));

    register_dissector("its", dissect_its_PDU, proto_its);

    // Register subdissector table
    its_version_subdissector_table = register_dissector_table("its.version", "ITS version", proto_its, FT_UINT8, BASE_DEC);
    its_msgid_subdissector_table = register_dissector_table("its.msg_id", "ITS message id", proto_its, FT_UINT32, BASE_DEC);
    regionid_subdissector_table = register_dissector_table("dsrc.regionid", "DSRC RegionId", proto_its, FT_UINT32, BASE_DEC);
    cam_pt_activation_table = register_dissector_table("cam.ptat", "CAM PtActivationType", proto_its, FT_UINT32, BASE_DEC);

    proto_its_denm = proto_register_protocol_in_name_only("ITS message - DENM", "DENM", "its.message.denm", proto_its, FT_BYTES);
    proto_its_denmv1 = proto_register_protocol_in_name_only("ITS message - DENMv1", "DENMv1", "its.message.denmv1", proto_its, FT_BYTES);
    proto_its_cam = proto_register_protocol_in_name_only("ITS message - CAM", "CAM", "its.message.cam", proto_its, FT_BYTES);
    proto_its_camv1 = proto_register_protocol_in_name_only("ITS message - CAMv1", "CAMv1", "its.message.camv1", proto_its, FT_BYTES);
    proto_its_spatemv1 = proto_register_protocol_in_name_only("ITS message - SPATEMv1", "SPATEMv1", "its.message.spatemv1", proto_its, FT_BYTES);
    proto_its_spatem = proto_register_protocol_in_name_only("ITS message - SPATEM", "SPATEM", "its.message.spatem", proto_its, FT_BYTES);
    proto_its_mapemv1 = proto_register_protocol_in_name_only("ITS message - MAPEMv1", "MAPEMv1", "its.message.mapemv1", proto_its, FT_BYTES);
    proto_its_mapem = proto_register_protocol_in_name_only("ITS message - MAPEM", "MAPEM", "its.message.mapem", proto_its, FT_BYTES);
    proto_its_ivimv1 = proto_register_protocol_in_name_only("ITS message - IVIMv1", "IVIMv1", "its.message.ivimv1", proto_its, FT_BYTES);
    proto_its_ivim = proto_register_protocol_in_name_only("ITS message - IVIM", "IVIM", "its.message.ivim", proto_its, FT_BYTES);
    proto_its_evrsr = proto_register_protocol_in_name_only("ITS message - EVRSR", "EVRSR", "its.message.evrsr", proto_its, FT_BYTES);
    proto_its_srem = proto_register_protocol_in_name_only("ITS message - SREM", "SREM", "its.message.srem", proto_its, FT_BYTES);
    proto_its_ssem = proto_register_protocol_in_name_only("ITS message - SSEM", "SSEM", "its.message.ssem", proto_its, FT_BYTES);
    proto_its_rtcmem = proto_register_protocol_in_name_only("ITS message - RTCMEM", "RTCMEM", "its.message.rtcmem", proto_its, FT_BYTES);
    proto_its_evcsn = proto_register_protocol_in_name_only("ITS message - EVCSN", "EVCSN", "its.message.evcsn", proto_its, FT_BYTES);
    proto_its_tistpg = proto_register_protocol_in_name_only("ITS message - TISTPG", "TISTPG", "its.message.tistpg", proto_its, FT_BYTES);
    proto_its_cpm = proto_register_protocol_in_name_only("ITS message - CPM", "CPM", "its.message.cpm", proto_its, FT_BYTES);

    proto_addgrpc = proto_register_protocol_in_name_only("DSRC Addition Grp C (EU)", "ADDGRPC", "dsrc.addgrpc", proto_its, FT_BYTES);

    // Decode as
    static build_valid_func its_da_build_value[1] = {its_msgid_value};
    static decode_as_value_t its_da_values = {its_msgid_prompt, 1, its_da_build_value};
    static decode_as_t its_da = {"its", "its.msg_id", 1, 0, &its_da_values, NULL, NULL,
                                    decode_as_default_populate_list, decode_as_default_reset, decode_as_default_change, NULL};

    register_decode_as(&its_da);
}

#define BTP_SUBDISS_SZ 2
#define BTP_PORTS_SZ   12

#define ITS_CAM_PROT_VER 2
#define ITS_CAM_PROT_VERv1 1
#define ITS_DENM_PROT_VER 2
#define ITS_DENM_PROT_VERv1 1
#define ITS_SPATEM_PROT_VERv1 1
#define ITS_SPATEM_PROT_VER 2
#define ITS_MAPEM_PROT_VERv1 1
#define ITS_MAPEM_PROT_VER 2
#define ITS_IVIM_PROT_VERv1 1
#define ITS_IVIM_PROT_VER 2
#define ITS_SREM_PROT_VER 2
#define ITS_SSEM_PROT_VER 2
#define ITS_RTCMEM_PROT_VER 2
#define ITS_TIS_TPG_PROT_VER 1
#define ITS_CPM_PROT_VER 1

void proto_reg_handoff_its(void)
{
    const char *subdissector[BTP_SUBDISS_SZ] = { "btpa.port", "btpb.port" };
    const guint16 ports[BTP_PORTS_SZ] = { ITS_WKP_DEN, ITS_WKP_CA, ITS_WKP_EVCSN, ITS_WKP_CHARGING, ITS_WKP_IVI, ITS_WKP_TPG, ITS_WKP_TLC_SSEM, ITS_WKP_GPC, ITS_WKP_TLC_SREM, ITS_WKP_RLT, ITS_WKP_TLM, ITS_WKP_CPS };
    int sdIdx, pIdx;
    dissector_handle_t its_handle_;

    // Register well known ports to btp subdissector table (BTP A and B)
    its_handle_ = create_dissector_handle(dissect_its_PDU, proto_its);
    for (sdIdx=0; sdIdx < BTP_SUBDISS_SZ; sdIdx++) {
        for (pIdx=0; pIdx < BTP_PORTS_SZ; pIdx++) {
            dissector_add_uint(subdissector[sdIdx], ports[pIdx], its_handle_);
        }
    }

    // Enable decode as for its pdu's send via udp
    dissector_add_for_decode_as("udp.port", its_handle_);

    dissector_add_for_decode_as("tcp.port", its_handle_);

    dissector_add_uint("its.msg_id", (ITS_DENM_PROT_VER << 16) + ITS_DENM,          create_dissector_handle( dissect_denm_DecentralizedEnvironmentalNotificationMessage_PDU, proto_its_denm ));
    dissector_add_uint("its.msg_id", (ITS_DENM_PROT_VERv1 << 16) + ITS_DENM,        create_dissector_handle( dissect_denmv1_DecentralizedEnvironmentalNotificationMessageV1_PDU, proto_its_denmv1 ));
    dissector_add_uint("its.msg_id", (ITS_CAM_PROT_VER << 16) + ITS_CAM,            create_dissector_handle( dissect_cam_CoopAwareness_PDU, proto_its_cam ));
    dissector_add_uint("its.msg_id", (ITS_CAM_PROT_VERv1 << 16) + ITS_CAM,          create_dissector_handle( dissect_camv1_CoopAwarenessV1_PDU, proto_its_camv1));
    dissector_add_uint("its.msg_id", (ITS_SPATEM_PROT_VERv1 << 16) + ITS_SPATEM,    create_dissector_handle( dissect_dsrc_SPAT_PDU, proto_its_spatemv1 ));
    dissector_add_uint("its.msg_id", (ITS_SPATEM_PROT_VER << 16) + ITS_SPATEM,      create_dissector_handle( dissect_dsrc_SPAT_PDU, proto_its_spatem ));
    dissector_add_uint("its.msg_id", (ITS_MAPEM_PROT_VERv1 << 16) + ITS_MAPEM,      create_dissector_handle( dissect_dsrc_MapData_PDU, proto_its_mapemv1 ));
    dissector_add_uint("its.msg_id", (ITS_MAPEM_PROT_VER << 16) + ITS_MAPEM,        create_dissector_handle( dissect_dsrc_MapData_PDU, proto_its_mapem ));
    dissector_add_uint("its.msg_id", (ITS_IVIM_PROT_VERv1 << 16) + ITS_IVIM,        create_dissector_handle( dissect_ivi_IviStructure_PDU, proto_its_ivimv1 ));
    dissector_add_uint("its.msg_id", (ITS_IVIM_PROT_VER << 16) + ITS_IVIM,          create_dissector_handle( dissect_ivi_IviStructure_PDU, proto_its_ivim ));
    dissector_add_uint("its.msg_id", ITS_EV_RSR,                                    create_dissector_handle( dissect_evrsr_EV_RSR_MessageBody_PDU, proto_its_evrsr ));
    dissector_add_uint("its.msg_id", (ITS_SREM_PROT_VER << 16) + ITS_SREM,          create_dissector_handle( dissect_dsrc_SignalRequestMessage_PDU, proto_its_srem ));
    dissector_add_uint("its.msg_id", (ITS_SSEM_PROT_VER << 16) + ITS_SSEM,          create_dissector_handle( dissect_dsrc_SignalStatusMessage_PDU, proto_its_ssem ));
    dissector_add_uint("its.msg_id", (ITS_RTCMEM_PROT_VER << 16) + ITS_RTCMEM,      create_dissector_handle( dissect_dsrc_RTCMcorrections_PDU, proto_its_rtcmem ));
    dissector_add_uint("its.msg_id", ITS_EVCSN,                                     create_dissector_handle( dissect_evcsn_EVChargingSpotNotificationPOIMessage_PDU, proto_its_evcsn ));
    dissector_add_uint("its.msg_id", (ITS_TIS_TPG_PROT_VER << 16) + ITS_TISTPGTRANSACTION, create_dissector_handle( dissect_tistpg_TisTpgTransaction_PDU, proto_its_tistpg ));
    dissector_add_uint("its.msg_id", (ITS_CPM_PROT_VER << 16) + ITS_CPM,            create_dissector_handle(dissect_cpm_CollectivePerceptionMessage_PDU, proto_its_cpm));

    dissector_add_for_decode_as("udp.port", create_dissector_handle( dissect_cam_CoopAwarensssssess_PDU, proto_its_cam ));

    /* Missing definitions: ITS_POI, ITS_SAEM */

    dissector_add_uint("dsrc.regionid", (addGrpC<<16)+Reg_ConnectionManeuverAssist, create_dissector_handle(dissect_AddGrpC_ConnectionManeuverAssist_addGrpC_PDU, proto_addgrpc ));
    dissector_add_uint("dsrc.regionid", (addGrpC<<16)+Reg_GenericLane, create_dissector_handle(dissect_AddGrpC_ConnectionTrajectory_addGrpC_PDU, proto_addgrpc ));
    dissector_add_uint("dsrc.regionid", (addGrpC<<16)+Reg_NodeAttributeSetXY, create_dissector_handle(dissect_AddGrpC_NodeAttributeSet_addGrpC_PDU, proto_addgrpc ));
    dissector_add_uint("dsrc.regionid", (addGrpC<<16)+Reg_IntersectionState, create_dissector_handle(dissect_AddGrpC_IntersectionState_addGrpC_PDU, proto_addgrpc ));
    dissector_add_uint("dsrc.regionid", (addGrpC<<16)+Reg_MapData,create_dissector_handle(dissect_AddGrpC_MapData_addGrpC_PDU, proto_addgrpc ));
    dissector_add_uint("dsrc.regionid", (addGrpC<<16)+Reg_Position3D, create_dissector_handle(dissect_AddGrpC_Position3D_addGrpC_PDU, proto_addgrpc ));
    dissector_add_uint("dsrc.regionid", (addGrpC<<16)+Reg_RestrictionUserType, create_dissector_handle(dissect_AddGrpC_RestrictionUserType_addGrpC_PDU, proto_addgrpc ));
    dissector_add_uint("dsrc.regionid", (addGrpC<<16)+Reg_SignalStatusPackage, create_dissector_handle(dissect_AddGrpC_SignalStatusPackage_addGrpC_PDU, proto_addgrpc ));
    dissector_add_uint("dsrc.regionid", (addGrpC<<16)+Reg_LaneAttributes, create_dissector_handle(dissect_AddGrpC_LaneAttributes_addGrpC_PDU, proto_addgrpc ));
    dissector_add_uint("dsrc.regionid", (addGrpC<<16)+Reg_MovementEvent, create_dissector_handle(dissect_AddGrpC_MovementEvent_addGrpC_PDU, proto_addgrpc ));
    dissector_add_uint("dsrc.regionid", (addGrpC<<16)+Reg_RequestorDescription, create_dissector_handle(dissect_AddGrpC_RequestorDescription_addGrpC_PDU, proto_addgrpc ));

    dissector_add_uint("ieee1609dot2.ssp", psid_den_basic_services, create_dissector_handle(dissect_denmssp_pdu, proto_its_denm));
    dissector_add_uint("ieee1609dot2.ssp", psid_ca_basic_services,  create_dissector_handle(dissect_camssp_pdu, proto_its_cam));
    dissector_add_uint("geonw.ssp", psid_den_basic_services, create_dissector_handle(dissect_denmssp_pdu, proto_its_denm));
    dissector_add_uint("geonw.ssp", psid_ca_basic_services,  create_dissector_handle(dissect_camssp_pdu, proto_its_cam));

    its_tap = register_tap("its");
}

/*
 * Editor modelines  -  https://www.wireshark.org/tools/modelines.html
 *
 * Local variables:
 * c-basic-offset: 4
 * tab-width: 8
 * indent-tabs-mode: nil
 * End:
 *
 * vi: set shiftwidth=4 tabstop=8 expandtab:
 * :indentSize=4:tabSize=8:noTabs=true:
 */
