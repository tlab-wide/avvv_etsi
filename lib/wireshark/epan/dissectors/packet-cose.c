/* packet-cose.c
 * Routines for CBOR Object Signing and Encryption (COSE) dissection
 * References:
 *     RFC 8152: https://tools.ietf.org/html/rfc8152
 *     RFC 8949: https://tools.ietf.org/html/rfc8949
 *
 * Copyright 2019-2021, Brian Sipos <brian.sipos@gmail.com>
 *
 * Wireshark - Network traffic analyzer
 * By Gerald Combs <gerald@wireshark.org>
 * Copyright 1998 Gerald Combs
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include <config.h>

#include "packet-cose.h"
#include <epan/wscbor.h>
#include <epan/packet.h>
#include <epan/prefs.h>
#include <epan/proto.h>
#include <epan/expert.h>
#include <epan/exceptions.h>
#include <stdio.h>
#include <inttypes.h>

void proto_register_cose(void);
void proto_reg_handoff_cose(void);

/// Glib logging "domain" name
static const char *LOG_DOMAIN = "COSE";
/// Protocol column name
static const char *const proto_name_cose = "COSE";

/// Protocol preferences and defaults

/// Protocol handles
static int proto_cose = -1;

/// Dissect opaque CBOR data
static dissector_handle_t handle_cbor = NULL;
/// Dissector handles
static dissector_handle_t handle_cose_msg_hdr = NULL;
static dissector_handle_t handle_cose_msg_tagged = NULL;
static dissector_handle_t handle_cose_sign = NULL;
static dissector_handle_t handle_cose_sign1 = NULL;
static dissector_handle_t handle_cose_encrypt = NULL;
static dissector_handle_t handle_cose_encrypt0 = NULL;
static dissector_handle_t handle_cose_mac = NULL;
static dissector_handle_t handle_cose_mac0 = NULL;
static dissector_handle_t handle_cose_key = NULL;
static dissector_handle_t handle_cose_key_set = NULL;

/// Dissect opaque data
static dissector_table_t table_media = NULL;
/// Dissect extension items
static dissector_table_t table_cose_msg_tag = NULL;
static dissector_table_t table_header = NULL;
static dissector_table_t table_keyparam = NULL;

static const val64_string alg_vals[] = {
    {-65535, "RS1"},
    {-259, "RS512"},
    {-258, "RS384"},
    {-257, "RS256"},
    {-47, "ES256K"},
    {-45, "SHAKE256"},
    {-44, "SHA-512"},
    {-43, "SHA-384"},
    {-39, "PS512"},
    {-38, "PS384"},
    {-37, "PS256"},
    {-36, "ES512"},
    {-35, "ES384"},
    {-34, "ECDH-SS + A256KW"},
    {-33, "ECDH-SS + A192KW"},
    {-32, "ECDH-SS + A128KW"},
    {-31, "ECDH-ES + A256KW"},
    {-30, "ECDH-ES + A192KW"},
    {-29, "ECDH-ES + A128KW"},
    {-28, "ECDH-SS + HKDF-512"},
    {-27, "ECDH-SS + HKDF-256"},
    {-26, "ECDH-ES + HKDF-512"},
    {-25, "ECDH-ES + HKDF-256"},
    {-18, "SHAKE128"},
    {-17, "SHA-512/256"},
    {-16, "SHA-256"},
    {-15, "SHA-256/64"},
    {-14, "SHA-1"},
    {-13, "direct+HKDF-AES-256"},
    {-12, "direct+HKDF-AES-128"},
    {-11, "direct+HKDF-SHA-512"},
    {-10, "direct+HKDF-SHA-256"},
    {-8, "EdDSA"},
    {-7, "ES256"},
    {-6, "direct"},
    {-5, "A256KW"},
    {-4, "A192KW"},
    {-3, "A128KW"},
    {0, "Reserved"},
    {1, "A128GCM"},
    {2, "A192GCM"},
    {3, "A256GCM"},
    {4, "HMAC 256/64"},
    {5, "HMAC 256/256"},
    {6, "HMAC 384/384"},
    {7, "HMAC 512/512"},
    {10, "AES-CCM-16-64-128"},
    {11, "AES-CCM-16-64-256"},
    {12, "AES-CCM-64-64-128"},
    {13, "AES-CCM-64-64-256"},
    {14, "AES-MAC 128/64"},
    {15, "AES-MAC 256/64"},
    {24, "ChaCha20/Poly1305"},
    {25, "AES-MAC 128/128"},
    {26, "AES-MAC 256/128"},
    {30, "AES-CCM-16-128-128"},
    {31, "AES-CCM-16-128-256"},
    {32, "AES-CCM-64-128-128"},
    {33, "AES-CCM-64-128-256"},
    {34, "IV-GENERATION"},
    {0, NULL},
};

static const val64_string kty_vals[] = {
    {0, "Reserved"},
    {1, "OKP"},
    {2, "EC2"},
    {3, "RSA"},
    {4, "Symmetric"},
    {5, "HSS-LMS"},
    {0, NULL},
};

static const val64_string keyops_vals[] = {
    {1, "sign"},
    {2, "verify"},
    {3, "encrypt"},
    {4, "decrypt"},
    {5, "key wrap"},
    {6, "key unwrap"},
    {7, "derive key"},
    {8, "derive bits"},
    {9, "MAC create"},
    {10, "MAC verify"},
    {0, NULL},
};

static const val64_string crv_vals[] = {
    {0, "Reserved"},
    {1, "P-256"},
    {2, "P-384"},
    {3, "P-521"},
    {4, "X25519"},
    {5, "X448"},
    {6, "Ed25519"},
    {7, "Ed448"},
    {0, NULL},
};

static int hf_msg_tag = -1;
static int hf_hdr_prot_bstr = -1;
static int hf_hdr_unprot = -1;
static int hf_payload_null = -1;
static int hf_payload_bstr = -1;
static int hf_cose_signature_list = -1;
static int hf_cose_signature = -1;
static int hf_signature = -1;
static int hf_ciphertext_null = -1;
static int hf_ciphertext_bstr = -1;
static int hf_cose_recipient_list = -1;
static int hf_cose_recipient = -1;
static int hf_tag = -1;

static int hf_hdr_label_int = -1;
static int hf_hdr_label_tstr = -1;

static int hf_hdr_salt = -1;
static int hf_hdr_static_key = -1;
static int hf_hdr_ephem_key = -1;
static int hf_hdr_alg_int = -1;
static int hf_hdr_alg_tstr = -1;
static int hf_hdr_crit_list = -1;
static int hf_hdr_ctype_uint = -1;
static int hf_hdr_ctype_tstr = -1;
static int hf_hdr_kid = -1;
static int hf_hdr_iv = -1;
static int hf_hdr_piv = -1;
static int hf_hdr_x5bag = -1;
static int hf_hdr_x5chain = -1;
static int hf_hdr_x5t = -1;
static int hf_hdr_x5t_hash = -1;
static int hf_hdr_x5u = -1;

static int hf_key = -1;

static int hf_keyparam_kty_int = -1;
static int hf_keyparam_kty_tstr = -1;
static int hf_keyparam_keyops_list = -1;
static int hf_keyparam_keyops_int = -1;
static int hf_keyparam_keyops_tstr = -1;
static int hf_keyparam_baseiv = -1;
static int hf_keyparam_crv_int = -1;
static int hf_keyparam_crv_tstr = -1;
static int hf_keyparam_xcoord = -1;
static int hf_keyparam_ycoord = -1;
static int hf_keyparam_dcoord = -1;
static int hf_keyparam_k = -1;

/// Field definitions
static hf_register_info fields[] = {
    {&hf_msg_tag, {"Message type tag", "cose.msgtag", FT_UINT64, BASE_DEC, NULL, 0x0, NULL, HFILL}},
    {&hf_hdr_prot_bstr, {"Protected Headers (bstr)", "cose.msg.prot_bstr", FT_BYTES, BASE_NONE, NULL, 0x0, NULL, HFILL}},
    {&hf_hdr_unprot, {"Unprotected Headers", "cose.msg.unprot", FT_NONE, BASE_NONE, NULL, 0x0, NULL, HFILL}},

    {&hf_payload_null, {"Payload Detached", "cose.msg.detached_payload", FT_NONE, BASE_NONE, NULL, 0x0, NULL, HFILL}},
    {&hf_payload_bstr, {"Payload", "cose.msg.payload", FT_BYTES, BASE_NONE, NULL, 0x0, NULL, HFILL}},
    {&hf_signature, {"Signature", "cose.msg.signature", FT_BYTES, BASE_NONE, NULL, 0x0, NULL, HFILL}},
    {&hf_cose_signature_list, {"Signature List, Count", "cose.msg.signature_list", FT_UINT64, BASE_DEC, NULL, 0x0, NULL, HFILL}},
    {&hf_cose_signature, {"COSE_Signature", "cose.msg.cose_signature", FT_NONE, BASE_NONE, NULL, 0x0, NULL, HFILL}},
    {&hf_ciphertext_null, {"Ciphertext Detached", "cose.msg.detached_ciphertext", FT_NONE, BASE_NONE, NULL, 0x0, NULL, HFILL}},
    {&hf_ciphertext_bstr, {"Ciphertext", "cose.msg.ciphertext", FT_BYTES, BASE_NONE, NULL, 0x0, NULL, HFILL}},
    {&hf_cose_recipient_list, {"Recipient List, Count", "cose.msg.recipient_list", FT_UINT64, BASE_DEC, NULL, 0x0, NULL, HFILL}},
    {&hf_cose_recipient, {"COSE_Recipient", "cose.msg.cose_recipient", FT_NONE, BASE_NONE, NULL, 0x0, NULL, HFILL}},
    {&hf_tag, {"Tag", "cose.msg.mac_tag", FT_BYTES, BASE_NONE, NULL, 0x0, NULL, HFILL}},

    {&hf_hdr_label_int, {"Label", "cose.header_label.int", FT_INT64, BASE_DEC, NULL, 0x0, NULL, HFILL}},
    {&hf_hdr_label_tstr, {"Label", "cose.header_label.tstr", FT_STRING, STR_UNICODE, NULL, 0x0, NULL, HFILL}},

    {&hf_hdr_salt, {"Salt", "cose.salt", FT_BYTES, BASE_NONE, NULL, 0x0, NULL, HFILL}},
    {&hf_hdr_static_key, {"Static Key", "cose.static_key", FT_NONE, BASE_NONE, NULL, 0x0, NULL, HFILL}},
    {&hf_hdr_ephem_key, {"Ephemeral Key", "cose.ephem_key", FT_NONE, BASE_NONE, NULL, 0x0, NULL, HFILL}},
    {&hf_hdr_alg_int, {"Algorithm", "cose.alg.int", FT_INT64, BASE_DEC | BASE_VAL64_STRING, VALS64(alg_vals), 0x0, NULL, HFILL}},
    {&hf_hdr_alg_tstr, {"Algorithm", "cose.alg.tstr", FT_STRING, STR_UNICODE, NULL, 0x0, NULL, HFILL}},
    {&hf_hdr_crit_list, {"Critical Headers, Count", "cose.crit", FT_INT64, BASE_DEC, NULL, 0x0, NULL, HFILL}},
    {&hf_hdr_ctype_uint, {"Content-Format", "cose.content-type.uint", FT_INT64, BASE_DEC, NULL, 0x0, NULL, HFILL}},
    {&hf_hdr_ctype_tstr, {"Content-Type", "cose.content-type.tstr", FT_STRING, STR_UNICODE, NULL, 0x0, NULL, HFILL}},
    {&hf_hdr_kid, {"Key identifier", "cose.kid", FT_BYTES, BASE_NONE, NULL, 0x0, NULL, HFILL}},
    {&hf_hdr_iv, {"IV", "cose.iv", FT_BYTES, BASE_NONE, NULL, 0x0, NULL, HFILL}},
    {&hf_hdr_piv, {"Partial IV", "cose.piv", FT_BYTES, BASE_NONE, NULL, 0x0, NULL, HFILL}},

    {&hf_hdr_x5bag, {"X509 Bag (x5bag)", "cose.x5bag", FT_NONE, BASE_NONE, NULL, 0x0, NULL, HFILL}},
    {&hf_hdr_x5chain, {"X509 Chain (x5chain)", "cose.x5chain", FT_NONE, BASE_NONE, NULL, 0x0, NULL, HFILL}},
    {&hf_hdr_x5t, {"X509 Thumbprint (x5t)", "cose.x5t", FT_NONE, BASE_NONE, NULL, 0x0, NULL, HFILL}},
    {&hf_hdr_x5t_hash, {"Hash Value", "cose.x5t.hash", FT_BYTES, BASE_NONE, NULL, 0x0, NULL, HFILL}},
    {&hf_hdr_x5u, {"X509 URI (x5u)", "cose.x5u", FT_STRING, STR_UNICODE, NULL, 0x0, NULL, HFILL}},

    {&hf_key, {"COSE_Key", "cose.key", FT_NONE, BASE_NONE, NULL, 0x0, NULL, HFILL}},

    {&hf_keyparam_kty_int, {"Key Type", "cose.kty.int", FT_INT64, BASE_DEC | BASE_VAL64_STRING, VALS64(kty_vals), 0x0, NULL, HFILL}},
    {&hf_keyparam_kty_tstr, {"Key Type", "cose.kty.tstr", FT_STRING, STR_UNICODE, NULL, 0x0, NULL, HFILL}},
    {&hf_keyparam_keyops_list, {"Key Operations", "cose.keyops", FT_NONE, BASE_NONE, NULL, 0x0, NULL, HFILL}},
    {&hf_keyparam_keyops_int, {"Operation", "cose.keyops.int", FT_INT64, BASE_DEC | BASE_VAL64_STRING, VALS64(keyops_vals), 0x0, NULL, HFILL}},
    {&hf_keyparam_keyops_tstr, {"Operation", "cose.keyops.tstr", FT_STRING, STR_UNICODE, NULL, 0x0, NULL, HFILL}},
    {&hf_keyparam_baseiv, {"Base IV", "cose.baseiv", FT_BYTES, BASE_NONE, NULL, 0x0, NULL, HFILL}},
    {&hf_keyparam_crv_int, {"Curve Type", "cose.crv.int", FT_INT64, BASE_DEC | BASE_VAL64_STRING, VALS64(crv_vals), 0x0, NULL, HFILL}},
    {&hf_keyparam_crv_tstr, {"Curve Type", "cose.crv.tstr", FT_STRING, STR_UNICODE, NULL, 0x0, NULL, HFILL}},
    {&hf_keyparam_xcoord, {"X-coordinate", "cose.key.xcoord", FT_BYTES, BASE_NONE, NULL, 0x0, NULL, HFILL}},
    {&hf_keyparam_ycoord, {"Y-coordinate", "cose.key.ycoord", FT_BYTES, BASE_NONE, NULL, 0x0, NULL, HFILL}},
    {&hf_keyparam_dcoord, {"Private Key", "cose.key.dcoord", FT_BYTES, BASE_NONE, NULL, 0x0, NULL, HFILL}},
    {&hf_keyparam_k, {"Key", "cose.key.k", FT_BYTES, BASE_NONE, NULL, 0x0, NULL, HFILL}},
};

static int ett_msg = -1;
static int ett_sig_list = -1;
static int ett_sig = -1;
static int ett_recip_list = -1;
static int ett_recip = -1;
static int ett_prot_bstr = -1;
static int ett_unprot = -1;
static int ett_hdr_map = -1;
static int ett_hdr_label = -1;
static int ett_hdr_static_key = -1;
static int ett_hdr_ephem_key = -1;
static int ett_hdr_crit_list = -1;
static int ett_hdr_x5cert_list = -1;
static int ett_hdr_x5t_list = -1;
static int ett_key = -1;
static int ett_key_set = -1;
static int ett_keyops_list = -1;
/// Tree structures
static int *ett[] = {
    &ett_msg,
    &ett_sig_list,
    &ett_sig,
    &ett_recip_list,
    &ett_recip,
    &ett_prot_bstr,
    &ett_unprot,
    &ett_hdr_map,
    &ett_hdr_label,
    &ett_hdr_static_key,
    &ett_hdr_ephem_key,
    &ett_hdr_crit_list,
    &ett_hdr_x5cert_list,
    &ett_hdr_x5t_list,
    &ett_key,
    &ett_key_set,
    &ett_keyops_list,
};

static expert_field ei_invalid_tag = EI_INIT;
static expert_field ei_value_partial_decode = EI_INIT;
static ei_register_info expertitems[] = {
    {&ei_invalid_tag, { "cose.invalid_tag", PI_UNDECODED, PI_WARN, "COSE dissector did not match any known tag", EXPFILL}},
    {&ei_value_partial_decode, { "cose.partial_decode", PI_MALFORMED, PI_WARN, "Value is only partially decoded", EXPFILL}},
};

guint cose_param_key_hash(gconstpointer ptr) {
    const cose_param_key_t *obj = (const cose_param_key_t *)ptr;
    guint val = 0;
    if (obj->principal) {
        val ^= g_int64_hash(obj->principal);
    }
    if (obj->label) {
        val ^= g_variant_hash(obj->label);
    }
    return val;
}

gboolean cose_param_key_equal(gconstpointer a, gconstpointer b) {
    const cose_param_key_t *aobj = (const cose_param_key_t *)a;
    const cose_param_key_t *bobj = (const cose_param_key_t *)b;

    if (aobj->principal && bobj->principal) {
        if (!g_variant_equal(aobj->principal, bobj->principal)) {
            return FALSE;
        }
    }
    else if ((aobj->principal == NULL) != (bobj->principal == NULL)) {
        return FALSE;
    }

    gboolean match;
    if (aobj->label && bobj->label) {
        match = g_variant_equal(aobj->label, bobj->label);
    }
    else {
        // don't care
        match = FALSE;
    }
    return match;
}

/** Dissect an ID-value pair within a context.
 *
 * @param dis_table The cose_param_key_t dissector table.
 * @param[in,out] ctx The context from other pairs.
 */
static void dissect_header_pair(dissector_table_t dis_table, cose_header_context_t *ctx, tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, gint *offset) {
    wscbor_chunk_t *chunk_label = wscbor_chunk_read(wmem_packet_scope(), tvb, offset);

    proto_item *item_label = NULL;
    proto_tree *volatile tree_label = NULL;
    tvbuff_t *volatile tvb_value = NULL;

    cose_param_key_t key = { 0 };

    switch (chunk_label->type_major) {
        case CBOR_TYPE_UINT:
        case CBOR_TYPE_NEGINT: {
            gint64 *label = wscbor_require_int64(wmem_packet_scope(), chunk_label);
            item_label = proto_tree_add_cbor_int64(tree, hf_hdr_label_int, pinfo, tvb, chunk_label, label);
            if (label) {
                key.label = ctx->label =
                    g_variant_new_int64(*label);
            }
            break;
        }
        case CBOR_TYPE_STRING: {
            const char *label = wscbor_require_tstr(wmem_packet_scope(), chunk_label);
            item_label = proto_tree_add_cbor_tstr(tree, hf_hdr_label_tstr, pinfo, tvb, chunk_label);
            if (label) {
                key.label = ctx->label =
                    g_variant_new_string(label);
            }
            break;
        }
        default:
            break;
    }

    // First attempt with context then without
    key.principal = ctx->principal;
    dissector_handle_t dissector = dissector_get_custom_table_handle(dis_table, &key);
    if (!dissector) {
        key.principal = NULL;
        dissector = dissector_get_custom_table_handle(dis_table, &key);
    }
    tree_label = proto_item_add_subtree(item_label, ett_hdr_label);

    // Peek into the value as tvb
    const gint offset_value = *offset;
    wscbor_skip_next_item(wmem_packet_scope(), tvb, offset);
    tvb_value = tvb_new_subset_length(tvb, offset_value, *offset - offset_value);

    gint sublen = 0;
    if (dissector) {
        sublen = call_dissector_only(dissector, tvb_value, pinfo, tree_label, ctx);
        if ((sublen < 0) ||
            ((sublen > 0) && ((guint)sublen < tvb_reported_length(tvb_value)))) {
            expert_add_info(pinfo, proto_tree_get_parent(tree), &ei_value_partial_decode);
        }
    }
    if (ctx->label) {
        g_variant_unref(ctx->label);
        ctx->label = NULL;
    }
    if (sublen == 0) {
        TRY {
            call_dissector(handle_cbor, tvb_value, pinfo, tree_label);
        }
        CATCH_ALL {}
        ENDTRY;
    }
}

/** Dissect an entire header map, either for messages, recipients, or keys.
 *
 * @param dis_table The cose_param_key_t dissector table.
 * @param tvb The source data.
 * @param tree The parent of the header map.
 * @param[in,out] offset The data offset.
 */
static void dissect_header_map(dissector_table_t dis_table, tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, gint *offset) {
    wscbor_chunk_t *chunk_hdr_map = wscbor_chunk_read(wmem_packet_scope(), tvb, offset);
    wscbor_require_map(chunk_hdr_map);
    proto_item *item_hdr_map = proto_tree_get_parent(tree);
    wscbor_chunk_mark_errors(pinfo, item_hdr_map, chunk_hdr_map);
    if (!wscbor_skip_if_errors(wmem_packet_scope(), tvb, offset, chunk_hdr_map)) {
        proto_tree *tree_hdr_map = proto_item_add_subtree(item_hdr_map, ett_hdr_map);

        cose_header_context_t *ctx = wmem_new0(wmem_packet_scope(), cose_header_context_t);

        for (guint64 ix = 0; ix < chunk_hdr_map->head_value; ++ix) {
            dissect_header_pair(dis_table, ctx, tvb, pinfo, tree_hdr_map, offset);
        }

        if (ctx->principal) {
            g_variant_unref(ctx->principal);
        }
        wmem_free(wmem_packet_scope(), ctx);
    }

    proto_item_set_len(item_hdr_map, *offset - chunk_hdr_map->start);
}

static int dissect_cose_msg_header_map(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_) {
    gint offset = 0;
    dissect_header_map(table_header, tvb, pinfo, tree, &offset);
    return offset;
}

/** Indicate the tag which informed the message type.
 */
static void dissect_msg_tag(tvbuff_t *tvb, packet_info *pinfo _U_, proto_tree *tree_msg, const wscbor_chunk_t *chunk_msg _U_, const void *data) {
    if (!data) {
        return;
    }
    const wscbor_tag_t *tag = (const wscbor_tag_t *)data;

    proto_tree_add_uint64(tree_msg, hf_msg_tag, tvb, tag->start, tag->length, tag->value);
}

/** Common behavior for pair of header maps.
 */
static void dissect_headers(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, gint *offset) {
    // Protected in bstr
    wscbor_chunk_t *chunk_prot_bstr = wscbor_chunk_read(wmem_packet_scope(), tvb, offset);
    tvbuff_t *tvb_prot = wscbor_require_bstr(wmem_packet_scope(), chunk_prot_bstr);
    proto_item *item_prot_bstr = proto_tree_add_cbor_bstr(tree, hf_hdr_prot_bstr, pinfo, tvb, chunk_prot_bstr);
    if (tvb_prot) {
        proto_tree *tree_prot = proto_item_add_subtree(item_prot_bstr, ett_prot_bstr);

        if (tvb_reported_length(tvb_prot) > 0) {
            dissect_cose_msg_header_map(tvb_prot, pinfo, tree_prot, NULL);
        }
    }

    // Unprotected
    tvbuff_t *tvb_unprot = tvb_new_subset_remaining(tvb, *offset);
    proto_item *item_unprot = proto_tree_add_item(tree, hf_hdr_unprot, tvb_unprot, 0, -1, ENC_NA);
    proto_tree *tree_unprot = proto_item_add_subtree(item_unprot, ett_unprot);
    const int sublen = dissect_cose_msg_header_map(tvb_unprot, pinfo, tree_unprot, NULL);
    *offset += sublen;
    proto_item_set_len(item_unprot, sublen);
}

/** Common behavior for payload.
 */
static void dissect_payload(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, gint *offset) {
    wscbor_chunk_t *chunk = wscbor_chunk_read(wmem_packet_scope(), tvb, offset);
    if (chunk->type_major == CBOR_TYPE_FLOAT_CTRL) {
        proto_tree_add_cbor_ctrl(tree, hf_payload_null, pinfo, tvb, chunk);
    }
    else {
        wscbor_require_bstr(wmem_packet_scope(), chunk);
        proto_tree_add_cbor_bstr(tree, hf_payload_bstr, pinfo, tvb, chunk);
    }
}
static void dissect_signature(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, gint *offset) {
    wscbor_chunk_t *chunk = wscbor_chunk_read(wmem_packet_scope(), tvb, offset);
    wscbor_require_bstr(wmem_packet_scope(), chunk);
    proto_tree_add_cbor_bstr(tree, hf_signature, pinfo, tvb, chunk);
}
static void dissect_cose_signature(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, gint *offset) {
    wscbor_chunk_t *chunk_sig = wscbor_chunk_read(wmem_packet_scope(), tvb, offset);
    wscbor_require_array_size(chunk_sig, 3, 3);
    proto_item *item_sig = proto_tree_add_cbor_container(tree, hf_cose_signature, pinfo, tvb, chunk_sig);
    if (!wscbor_skip_if_errors(wmem_packet_scope(), tvb, offset, chunk_sig)) {
        proto_tree *tree_sig = proto_item_add_subtree(item_sig, ett_sig);

        dissect_headers(tvb, pinfo, tree_sig, offset);
        dissect_signature(tvb, pinfo, tree_sig, offset);
    }
    proto_item_set_len(item_sig, *offset - chunk_sig->start);
}
static void dissect_ciphertext(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, gint *offset) {
    wscbor_chunk_t *chunk = wscbor_chunk_read(wmem_packet_scope(), tvb, offset);
    if (chunk->type_major == CBOR_TYPE_FLOAT_CTRL) {
        proto_tree_add_cbor_ctrl(tree, hf_ciphertext_null, pinfo, tvb, chunk);
    }
    else {
        wscbor_require_bstr(wmem_packet_scope(), chunk);
        proto_tree_add_cbor_bstr(tree, hf_ciphertext_bstr, pinfo, tvb, chunk);
    }
}
static void dissect_cose_recipient(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, gint *offset);
static void dissect_cose_recipient_list(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, gint *offset) {
    wscbor_chunk_t *chunk_list = wscbor_chunk_read(wmem_packet_scope(), tvb, offset);
    wscbor_require_array(chunk_list);
    proto_item *item_list = proto_tree_add_cbor_container(tree, hf_cose_recipient_list, pinfo, tvb, chunk_list);
    if (!wscbor_skip_if_errors(wmem_packet_scope(), tvb, offset, chunk_list)) {
        proto_tree *tree_recip_list = proto_item_add_subtree(item_list, ett_recip_list);

        for (guint64 ix = 0; ix < chunk_list->head_value; ++ix) {
            dissect_cose_recipient(tvb, pinfo, tree_recip_list, offset);
        }
    }
    proto_item_set_len(item_list, *offset - chunk_list->start);
}
static void dissect_cose_recipient(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, gint *offset) {
    wscbor_chunk_t *chunk_recip = wscbor_chunk_read(wmem_packet_scope(), tvb, offset);
    wscbor_require_array_size(chunk_recip, 3, 4);
    proto_item *item_recip = proto_tree_add_cbor_container(tree, hf_cose_recipient, pinfo, tvb, chunk_recip);
    if (!wscbor_skip_if_errors(wmem_packet_scope(), tvb, offset, chunk_recip)) {
        proto_tree *tree_recip = proto_item_add_subtree(item_recip, ett_recip);

        dissect_headers(tvb, pinfo, tree_recip, offset);
        dissect_ciphertext(tvb, pinfo, tree_recip, offset);
        if (chunk_recip->head_value > 3) {
            dissect_cose_recipient_list(tvb, pinfo, tree_recip, offset);
        }
    }
    proto_item_set_len(item_recip, *offset - chunk_recip->start);

}
static void dissect_tag(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, gint *offset) {
    wscbor_chunk_t *chunk = wscbor_chunk_read(wmem_packet_scope(), tvb, offset);
    wscbor_require_bstr(wmem_packet_scope(), chunk);
    proto_tree_add_cbor_bstr(tree, hf_tag, pinfo, tvb, chunk);
}

// Top-level protocol dissectors
static int dissect_cose_sign(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data) {
    gint offset = 0;

    wscbor_chunk_t *chunk_msg = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    wscbor_require_array_size(chunk_msg, 4, 4);
    proto_item *item_msg = proto_tree_add_cbor_container(tree, proto_cose, pinfo, tvb, chunk_msg);
    proto_item_append_text(item_msg, ": COSE_Sign");
    if (!wscbor_skip_if_errors(wmem_packet_scope(), tvb, &offset, chunk_msg)) {
        proto_tree *tree_msg = proto_item_add_subtree(item_msg, ett_msg);
        dissect_msg_tag(tvb, pinfo, tree_msg, chunk_msg, data);

        dissect_headers(tvb, pinfo, tree_msg, &offset);
        dissect_payload(tvb, pinfo, tree_msg, &offset);

        wscbor_chunk_t *chunk_sig_list = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
        wscbor_require_array(chunk_sig_list);
        proto_item *item_sig_list = proto_tree_add_cbor_container(tree_msg, hf_cose_signature_list, pinfo, tvb, chunk_sig_list);
        if (!wscbor_skip_if_errors(wmem_packet_scope(), tvb, &offset, chunk_sig_list)) {
            proto_tree *tree_sig_list = proto_item_add_subtree(item_sig_list, ett_sig_list);

            for (guint64 ix = 0; ix < chunk_sig_list->head_value; ++ix) {
                dissect_cose_signature(tvb, pinfo, tree_sig_list, &offset);
            }
        }
        proto_item_set_len(item_sig_list, offset - chunk_sig_list->start);
    }

    return offset;
}
static int dissect_cose_sign1(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data) {
    gint offset = 0;

    wscbor_chunk_t *chunk_msg = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    wscbor_require_array_size(chunk_msg, 4, 4);
    proto_item *item_msg = proto_tree_add_cbor_container(tree, proto_cose, pinfo, tvb, chunk_msg);
    proto_item_append_text(item_msg, ": COSE_Sign1");
    if (!wscbor_skip_if_errors(wmem_packet_scope(), tvb, &offset, chunk_msg)) {
        proto_tree *tree_msg = proto_item_add_subtree(item_msg, ett_msg);
        dissect_msg_tag(tvb, pinfo, tree_msg, chunk_msg, data);

        dissect_headers(tvb, pinfo, tree_msg, &offset);
        dissect_payload(tvb, pinfo, tree_msg, &offset);
        dissect_signature(tvb, pinfo, tree_msg, &offset);
    }

    return offset;
}
static int dissect_cose_encrypt(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data) {
    gint offset = 0;

    wscbor_chunk_t *chunk_msg = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    wscbor_require_array_size(chunk_msg, 4, 4);
    proto_item *item_msg = proto_tree_add_cbor_container(tree, proto_cose, pinfo, tvb, chunk_msg);
    proto_item_append_text(item_msg, ": COSE_Encrypt");
    if (!wscbor_skip_if_errors(wmem_packet_scope(), tvb, &offset, chunk_msg)) {
        proto_tree *tree_msg = proto_item_add_subtree(item_msg, ett_msg);
        dissect_msg_tag(tvb, pinfo, tree_msg, chunk_msg, data);

        dissect_headers(tvb, pinfo, tree_msg, &offset);
        dissect_ciphertext(tvb, pinfo, tree_msg, &offset);
        dissect_cose_recipient_list(tvb, pinfo, tree_msg, &offset);
    }

    return offset;
}
static int dissect_cose_encrypt0(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data) {
    gint offset = 0;

    wscbor_chunk_t *chunk_msg = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    wscbor_require_array_size(chunk_msg, 3, 3);
    proto_item *item_msg = proto_tree_add_cbor_container(tree, proto_cose, pinfo, tvb, chunk_msg);
    proto_item_append_text(item_msg, ": COSE_Encrypt0");
    if (!wscbor_skip_if_errors(wmem_packet_scope(), tvb, &offset, chunk_msg)) {
        proto_tree *tree_msg = proto_item_add_subtree(item_msg, ett_msg);
        dissect_msg_tag(tvb, pinfo, tree_msg, chunk_msg, data);

        dissect_headers(tvb, pinfo, tree_msg, &offset);
        dissect_ciphertext(tvb, pinfo, tree_msg, &offset);
    }

    return offset;
}
static int dissect_cose_mac(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data) {
    gint offset = 0;

    wscbor_chunk_t *chunk_msg = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    wscbor_require_array_size(chunk_msg, 5, 5);
    proto_item *item_msg = proto_tree_add_cbor_container(tree, proto_cose, pinfo, tvb, chunk_msg);
    proto_item_append_text(item_msg, ": COSE_Mac");
    if (!wscbor_skip_if_errors(wmem_packet_scope(), tvb, &offset, chunk_msg)) {
        proto_tree *tree_msg = proto_item_add_subtree(item_msg, ett_msg);
        dissect_msg_tag(tvb, pinfo, tree_msg, chunk_msg, data);

        dissect_headers(tvb, pinfo, tree_msg, &offset);
        dissect_payload(tvb, pinfo, tree_msg, &offset);
        dissect_tag(tvb, pinfo, tree_msg, &offset);
        dissect_cose_recipient_list(tvb, pinfo, tree_msg, &offset);
    }

    return offset;
}
static int dissect_cose_mac0(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data) {
    gint offset = 0;

    wscbor_chunk_t *chunk_msg = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    wscbor_require_array_size(chunk_msg, 4, 4);
    proto_item *item_msg = proto_tree_add_cbor_container(tree, proto_cose, pinfo, tvb, chunk_msg);
    proto_item_append_text(item_msg, ": COSE_Mac0");
    if (!wscbor_skip_if_errors(wmem_packet_scope(), tvb, &offset, chunk_msg)) {
        proto_tree *tree_msg = proto_item_add_subtree(item_msg, ett_msg);
        dissect_msg_tag(tvb, pinfo, tree_msg, chunk_msg, data);

        dissect_headers(tvb, pinfo, tree_msg, &offset);
        dissect_payload(tvb, pinfo, tree_msg, &offset);
        dissect_tag(tvb, pinfo, tree_msg, &offset);
    }

    return offset;
}

/** Dissect a tagged COSE message.
 */
static int dissect_cose_msg_tagged(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_) {
    gint offset = 0;

    // All messages have the same base structure, attempt all tags present
    wscbor_chunk_t *chunk = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    for (wmem_list_frame_t *it = wmem_list_head(chunk->tags); it;
            it = wmem_list_frame_next(it)) {
        wscbor_tag_t *tag = (wscbor_tag_t *) wmem_list_frame_data(it);
        // first usable tag wins
        dissector_handle_t dissector = dissector_get_custom_table_handle(table_cose_msg_tag, &(tag->value));
        if (!dissector) {
            continue;
        }
        g_log(LOG_DOMAIN, G_LOG_LEVEL_INFO, "main dissector using tag %" G_GUINT64_FORMAT, tag->value);
        int sublen = call_dissector_only(dissector, tvb, pinfo, tree, tag);
        if (sublen > 0) {
            return sublen;
        }
    }

    g_log(LOG_DOMAIN, G_LOG_LEVEL_WARNING, "main dissector did not match any known tag");
    proto_item *item_msg = proto_tree_add_item(tree, proto_cose, tvb, 0, -1, 0);
    expert_add_info(pinfo, item_msg, &ei_invalid_tag);
    return -1;
}

static void dissect_value_alg(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, gint *offset, GVariant **value) {
    wscbor_chunk_t *chunk = wscbor_chunk_read(wmem_packet_scope(), tvb, offset);
    switch (chunk->type_major) {
        case CBOR_TYPE_UINT:
        case CBOR_TYPE_NEGINT: {
            gint64 *val = wscbor_require_int64(wmem_packet_scope(), chunk);
            proto_tree_add_cbor_int64(tree, hf_hdr_alg_int, pinfo, tvb, chunk, val);
            if (value && val) {
                *value = g_variant_new_int64(*val);
            }
            break;
        }
        case CBOR_TYPE_STRING: {
            const char *val = wscbor_require_tstr(wmem_packet_scope(), chunk);
            proto_tree_add_cbor_tstr(tree, hf_hdr_alg_tstr, pinfo, tvb, chunk);
            if (value && val) {
                *value = g_variant_new_string(val);
            }
            break;
        }
        default:
            break;
    }
}

static int dissect_header_salt(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_) {
    gint offset = 0;

    wscbor_chunk_t *chunk = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    wscbor_require_bstr(wmem_packet_scope(), chunk);
    proto_tree_add_cbor_bstr(tree, hf_hdr_salt, pinfo, tvb, chunk);

    return offset;
}

static void dissect_value_cose_key(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, gint *offset) {
    dissect_header_map(table_keyparam, tvb, pinfo, tree, offset);
}

static int dissect_header_static_key(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_) {
    gint offset = 0;
    proto_item *item_ctr = proto_tree_add_item(tree, hf_hdr_static_key, tvb, 0, -1, ENC_NA);
    proto_tree *tree_ctr = proto_item_add_subtree(item_ctr, ett_hdr_static_key);
    dissect_value_cose_key(tvb, pinfo, tree_ctr, &offset);
    return offset;
}

static int dissect_header_ephem_key(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_) {
    gint offset = 0;
    proto_item *item_ctr = proto_tree_add_item(tree, hf_hdr_ephem_key, tvb, 0, -1, ENC_NA);
    proto_tree *tree_ctr = proto_item_add_subtree(item_ctr, ett_hdr_ephem_key);
    dissect_value_cose_key(tvb, pinfo, tree_ctr, &offset);
    return offset;
}

static int dissect_header_alg(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data) {
    cose_header_context_t *ctx = (cose_header_context_t *)data;
    gint offset = 0;

    dissect_value_alg(tvb, pinfo, tree, &offset, &(ctx->principal));

    return offset;
}

static int dissect_header_crit(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_) {
    gint offset = 0;

    wscbor_chunk_t *chunk_list = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    wscbor_require_array(chunk_list);
    proto_item *item_list = proto_tree_add_cbor_container(tree, hf_hdr_crit_list, pinfo, tvb, chunk_list);
    if (!wscbor_skip_if_errors(wmem_packet_scope(), tvb, &offset, chunk_list)) {
        proto_tree *tree_list = proto_item_add_subtree(item_list, ett_hdr_crit_list);

        for (guint64 ix = 0; ix < chunk_list->head_value; ++ix) {
            wscbor_chunk_t *chunk_label = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
            switch (chunk_label->type_major) {
                case CBOR_TYPE_UINT:
                case CBOR_TYPE_NEGINT: {
                    gint64 *label = wscbor_require_int64(wmem_packet_scope(), chunk_label);
                    proto_tree_add_cbor_int64(tree_list, hf_hdr_label_int, pinfo, tvb, chunk_label, label);
                    break;
                }
                case CBOR_TYPE_STRING: {
                    proto_tree_add_cbor_tstr(tree_list, hf_hdr_label_tstr, pinfo, tvb, chunk_label);
                    break;
                }
                default:
                    break;
            }
        }
    }

    proto_item_set_len(item_list, offset);
    return offset;
}

static int dissect_header_ctype(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_) {
    gint offset = 0;

    wscbor_chunk_t *chunk = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    switch (chunk->type_major) {
        case CBOR_TYPE_UINT: {
            guint64 *val = wscbor_require_uint64(wmem_packet_scope(), chunk);
            proto_tree_add_cbor_uint64(tree, hf_hdr_ctype_uint, pinfo, tvb, chunk, val);
            break;
        }
        case CBOR_TYPE_STRING: {
            proto_tree_add_cbor_tstr(tree, hf_hdr_ctype_tstr, pinfo, tvb, chunk);
            break;
        }
        default:
            break;
    }

    return offset;
}

static int dissect_header_kid(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_) {
    gint offset = 0;

    wscbor_chunk_t *chunk = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    wscbor_require_bstr(wmem_packet_scope(), chunk);
    proto_tree_add_cbor_bstr(tree, hf_hdr_kid, pinfo, tvb, chunk);

    return offset;
}

static int dissect_header_iv(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_) {
    gint offset = 0;

    wscbor_chunk_t *chunk = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    wscbor_require_bstr(wmem_packet_scope(), chunk);
    proto_tree_add_cbor_bstr(tree, hf_hdr_iv, pinfo, tvb, chunk);

    return offset;
}

static int dissect_header_piv(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_) {
    gint offset = 0;

    wscbor_chunk_t *chunk = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    wscbor_require_bstr(wmem_packet_scope(), chunk);
    proto_tree_add_cbor_bstr(tree, hf_hdr_piv, pinfo, tvb, chunk);

    return offset;
}

static void dissect_value_x5cert(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, gint *offset) {
    wscbor_chunk_t *chunk_item = wscbor_chunk_read(wmem_packet_scope(), tvb, offset);
    tvbuff_t *tvb_item = wscbor_require_bstr(wmem_packet_scope(), chunk_item);

    if (tvb_item) {
        // disallow column text rewrite
        gchar *info_text = wmem_strdup(wmem_packet_scope(), col_get_text(pinfo->cinfo, COL_INFO));

        TRY {
            dissector_try_string(
                table_media,
                "application/pkix-cert",
                tvb_item,
                pinfo,
                tree,
                NULL
            );
        }
        CATCH_ALL {}
        ENDTRY;

        col_add_str(pinfo->cinfo, COL_INFO, info_text);
    }

}
static void dissect_value_cosex509(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, int hfindex, gint *offset) {
    proto_item *item_ctr = proto_tree_add_item(tree, hfindex, tvb, 0, -1, ENC_NA);
    proto_tree *tree_ctr = proto_item_add_subtree(item_ctr, ett_hdr_x5cert_list);

    wscbor_chunk_t *chunk_ctr = wscbor_chunk_read(wmem_packet_scope(), tvb, offset);
    switch (chunk_ctr->type_major) {
        case CBOR_TYPE_ARRAY: {
            wscbor_require_array(chunk_ctr);
            if (!wscbor_skip_if_errors(wmem_packet_scope(), tvb, offset, chunk_ctr)) {
                for (guint64 ix = 0; ix < chunk_ctr->head_value; ++ix) {
                    dissect_value_x5cert(tvb, pinfo, tree_ctr, offset);
                }
            }
            break;
        }
        case CBOR_TYPE_BYTESTRING: {
            // re-read this chunk as cert
            *offset = chunk_ctr->start;
            dissect_value_x5cert(tvb, pinfo, tree_ctr, offset);
            break;
        }
        default:
            break;
    }

}
static int dissect_header_x5bag(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_) {
    gint offset = 0;
    dissect_value_cosex509(tvb, pinfo, tree, hf_hdr_x5bag, &offset);
    return offset;
}
static int dissect_header_x5chain(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_) {
    gint offset = 0;
    dissect_value_cosex509(tvb, pinfo, tree, hf_hdr_x5chain, &offset);
    return offset;
}

static int dissect_header_x5t(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_) {
    gint offset = 0;

    wscbor_chunk_t *chunk_list = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    wscbor_require_array_size(chunk_list, 2, 2);
    proto_item *item_list = proto_tree_add_cbor_container(tree, hf_hdr_x5t, pinfo, tvb, chunk_list);
    if (!wscbor_skip_if_errors(wmem_packet_scope(), tvb, &offset, chunk_list)) {
        proto_tree *tree_list = proto_item_add_subtree(item_list, ett_hdr_x5t_list);

        dissect_value_alg(tvb, pinfo, tree_list, &offset, NULL);

        wscbor_chunk_t *chunk_hash = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
        wscbor_require_bstr(wmem_packet_scope(), chunk_hash);
        proto_tree_add_cbor_bstr(tree_list, hf_hdr_x5t_hash, pinfo, tvb, chunk_hash);

    }

    return offset;
}

static int dissect_header_x5u(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_) {
    gint offset = 0;

    wscbor_chunk_t *chunk = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    wscbor_require_major_type(chunk, CBOR_TYPE_STRING);
    proto_tree_add_cbor_tstr(tree, hf_hdr_x5u, pinfo, tvb, chunk);

    return offset;
}

static int dissect_cose_key(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_) {
    gint offset = 0;

    proto_item *item_msg = proto_tree_add_item(tree, proto_cose, tvb, 0, -1, 0);
    proto_item_append_text(item_msg, ": COSE_Key");

    dissect_value_cose_key(tvb, pinfo, tree, &offset);

    return offset;
}

static int dissect_cose_key_set(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_) {
    gint offset = 0;

    wscbor_chunk_t *chunk_set = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    wscbor_require_array(chunk_set);
    proto_item *item_set = proto_tree_add_cbor_container(tree, proto_cose, pinfo, tvb, chunk_set);
    proto_item_append_text(item_set, ": COSE_KeySet");
    if (!wscbor_skip_if_errors(wmem_packet_scope(), tvb, &offset, chunk_set)) {
        proto_tree *tree_set = proto_item_add_subtree(item_set, ett_key_set);

        for (guint64 ix = 0; ix < chunk_set->head_value; ++ix) {
            proto_item *item_key = proto_tree_add_item(tree_set, hf_key, tvb, offset, -1, ENC_NA);
            proto_tree *tree_key = proto_item_add_subtree(item_key, ett_key);

            const gint offset_key = offset;
            dissect_value_cose_key(tvb, pinfo, tree_key, &offset);
            proto_item_set_len(item_key, offset - offset_key);
        }
    }

    proto_item_set_len(item_set, offset);
    return offset;
}

static int dissect_keyparam_kty(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data) {
    cose_header_context_t *ctx = (cose_header_context_t *)data;
    gint offset = 0;

    wscbor_chunk_t *chunk = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    switch (chunk->type_major) {
        case CBOR_TYPE_UINT:
        case CBOR_TYPE_NEGINT: {
            gint64 *val = wscbor_require_int64(wmem_packet_scope(), chunk);
            proto_tree_add_cbor_int64(tree, hf_keyparam_kty_int, pinfo, tvb, chunk, val);
            if (val) {
                ctx->principal = g_variant_new_int64(*val);
            }
            break;
        }
        case CBOR_TYPE_STRING: {
            const char *val = wscbor_require_tstr(wmem_packet_scope(), chunk);
            proto_tree_add_cbor_tstr(tree, hf_keyparam_kty_tstr, pinfo, tvb, chunk);
            if (val) {
                ctx->principal = g_variant_new_string(val);
            }
            break;
        }
        default:
            break;
    }

    return offset;
}

static int dissect_keyparam_keyops(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_) {
    gint offset = 0;

    wscbor_chunk_t *chunk_list = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    wscbor_require_array(chunk_list);
    proto_item *item_list = proto_tree_add_cbor_container(tree, hf_keyparam_keyops_list, pinfo, tvb, chunk_list);
    if (!wscbor_skip_if_errors(wmem_packet_scope(), tvb, &offset, chunk_list)) {
        proto_tree *tree_list = proto_item_add_subtree(item_list, ett_keyops_list);

        for (guint64 ix = 0; ix < chunk_list->head_value; ++ix) {
            wscbor_chunk_t *chunk = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
            switch (chunk->type_major) {
                case CBOR_TYPE_UINT:
                case CBOR_TYPE_NEGINT: {
                    gint64 *val = wscbor_require_int64(wmem_packet_scope(), chunk);
                    proto_tree_add_cbor_int64(tree_list, hf_keyparam_keyops_int, pinfo, tvb, chunk, val);
                    break;
                }
                case CBOR_TYPE_STRING: {
                    proto_tree_add_cbor_tstr(tree_list, hf_keyparam_keyops_tstr, pinfo, tvb, chunk);
                    break;
                }
                default:
                    break;
            }
        }
    }
    proto_item_set_len(item_list, offset - chunk_list->start);

    return offset;
}

static int dissect_keyparam_baseiv(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_) {
    gint offset = 0;

    wscbor_chunk_t *chunk = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    wscbor_require_bstr(wmem_packet_scope(), chunk);
    proto_tree_add_cbor_bstr(tree, hf_keyparam_baseiv, pinfo, tvb, chunk);

    return offset;
}

static int dissect_keyparam_crv(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_) {
    gint offset = 0;

    wscbor_chunk_t *chunk = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    switch (chunk->type_major) {
        case CBOR_TYPE_UINT:
        case CBOR_TYPE_NEGINT: {
            gint64 *val = wscbor_require_int64(wmem_packet_scope(), chunk);
            proto_tree_add_cbor_int64(tree, hf_keyparam_crv_int, pinfo, tvb, chunk, val);
            break;
        }
        case CBOR_TYPE_STRING: {
            proto_tree_add_cbor_tstr(tree, hf_keyparam_crv_tstr, pinfo, tvb, chunk);
            break;
        }
        default:
            break;
    }

    return offset;
}

static int dissect_keyparam_xcoord(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_) {
    gint offset = 0;

    wscbor_chunk_t *chunk = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    wscbor_require_bstr(wmem_packet_scope(), chunk);
    proto_tree_add_cbor_bstr(tree, hf_keyparam_xcoord, pinfo, tvb, chunk);

    return offset;
}

static int dissect_keyparam_ycoord(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_) {
    gint offset = 0;

    wscbor_chunk_t *chunk = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    switch (chunk->type_major) {
        case CBOR_TYPE_FLOAT_CTRL: {
            proto_tree_add_item(tree, hf_keyparam_ycoord, tvb, 0, 0, ENC_NA);
            break;
        }
        case CBOR_TYPE_BYTESTRING: {
            proto_tree_add_cbor_bstr(tree, hf_keyparam_ycoord, pinfo, tvb, chunk);
            break;
        }
        default:
            break;
    }

    return offset;
}

static int dissect_keyparam_dcoord(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_) {
    gint offset = 0;

    wscbor_chunk_t *chunk = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    wscbor_require_bstr(wmem_packet_scope(), chunk);
    proto_tree_add_cbor_bstr(tree, hf_keyparam_dcoord, pinfo, tvb, chunk);

    return offset;
}

static int dissect_keyparam_k(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data _U_) {
    gint offset = 0;

    wscbor_chunk_t *chunk = wscbor_chunk_read(wmem_packet_scope(), tvb, &offset);
    wscbor_require_bstr(wmem_packet_scope(), chunk);
    proto_tree_add_cbor_bstr(tree, hf_keyparam_k, pinfo, tvb, chunk);

    return offset;
}


/** Register a message dissector.
 */
static void register_msg_dissector(dissector_handle_t dis_h, guint64 tag_int, const char *media) {
    guint64 *key_int = g_new(guint64, 1);
    *key_int = tag_int;
    dissector_add_custom_table_handle("cose.msgtag", key_int, dis_h);

    if (media) {
        dissector_add_string("media_type", media, dis_h);
    }
}

/** Register a header dissector.
 */
static void register_header_dissector(dissector_t dissector, GVariant *label) {
    dissector_handle_t dis_h = create_dissector_handle(dissector, proto_cose);

    cose_param_key_t *key = g_new0(cose_param_key_t, 1);
    key->label = label;

    dissector_add_custom_table_handle("cose.header", key, dis_h);
}

/** Register a key parameter dissector.
 * @param dissector The dissector function.
 * @param kty The associated key type "kty" or NULL.
 */
static void register_keyparam_dissector(dissector_t dissector, GVariant *kty, GVariant *label) {
    dissector_handle_t dis_h = create_dissector_handle(dissector, proto_cose);

    cose_param_key_t *key = g_new0(cose_param_key_t, 1);
    if (kty) {
        g_variant_ref(kty);
        key->principal = kty;
    }
    key->label = label;

    dissector_add_custom_table_handle("cose.keyparam", key, dis_h);
}

/// Initialize for a new file load
static void cose_init(void) {
}

/// Cleanup after a file
static void cose_cleanup(void) {
}

/// Re-initialize after a configuration change
static void cose_reinit(void) {
}

/// Overall registration of the protocol
void proto_register_cose(void) {
    proto_cose = proto_register_protocol(
        "CBOR Object Signing and Encryption", /* name */
        proto_name_cose, /* short name */
        "cose" /* abbrev */
    );
    register_init_routine(&cose_init);
    register_cleanup_routine(&cose_cleanup);

    proto_register_field_array(proto_cose, fields, array_length(fields));
    proto_register_subtree_array(ett, array_length(ett));
    expert_module_t *expert = expert_register_protocol(proto_cose);
    expert_register_field_array(expert, expertitems, array_length(expertitems));

    handle_cose_msg_hdr = register_dissector("cose.msg.headers", dissect_cose_msg_header_map, proto_cose);

    table_cose_msg_tag = register_custom_dissector_table("cose.msgtag", "COSE Message Tag", proto_cose, g_int64_hash, g_int64_equal);
    handle_cose_msg_tagged = register_dissector("cose", dissect_cose_msg_tagged, proto_cose);
    handle_cose_sign = register_dissector("cose_sign", dissect_cose_sign, proto_cose);
    handle_cose_sign1 = register_dissector("cose_sign1", dissect_cose_sign1, proto_cose);
    handle_cose_encrypt = register_dissector("cose_encrypt", dissect_cose_encrypt, proto_cose);
    handle_cose_encrypt0 = register_dissector("cose_encrypt0", dissect_cose_encrypt0, proto_cose);
    handle_cose_mac = register_dissector("cose_mac", dissect_cose_mac, proto_cose);
    handle_cose_mac0 = register_dissector("cose_mac0", dissect_cose_mac0, proto_cose);

    table_header = register_custom_dissector_table("cose.header", "COSE Header Parameter", proto_cose, cose_param_key_hash, cose_param_key_equal);

    handle_cose_key = register_dissector("cose_key", dissect_cose_key, proto_cose);
    handle_cose_key_set = register_dissector("cose_key_set", dissect_cose_key_set, proto_cose);

    table_keyparam = register_custom_dissector_table("cose.keyparam", "COSE Key Parameter", proto_cose, cose_param_key_hash, cose_param_key_equal);

    module_t *module_cose = prefs_register_protocol(proto_cose, cose_reinit);
    (void)module_cose;
}

void proto_reg_handoff_cose(void) {
    table_media = find_dissector_table("media_type");
    handle_cbor = find_dissector("cbor");

    dissector_add_string("media_type", "application/cose", handle_cose_msg_tagged);
    // RFC 8152 tags and names (Table 26)
    register_msg_dissector(handle_cose_sign, 98, "application/cose; cose-type=\"cose-sign\"");
    register_msg_dissector(handle_cose_sign1, 18, "application/cose; cose-type=\"cose-sign1\"");
    register_msg_dissector(handle_cose_encrypt, 96, "application/cose; cose-type=\"cose-encrypt\"");
    register_msg_dissector(handle_cose_encrypt0, 16, "application/cose; cose-type=\"cose-encrypt0\"");
    register_msg_dissector(handle_cose_mac, 97, "application/cose; cose-type=\"cose-mac\"");
    register_msg_dissector(handle_cose_mac0, 17, "application/cose; cose-type=\"cose-mac0\"");

    // RFC 8152 header labels
    register_header_dissector(dissect_header_salt, g_variant_new_int64(-20));
    register_header_dissector(dissect_header_static_key, g_variant_new_int64(-2));
    register_header_dissector(dissect_header_ephem_key, g_variant_new_int64(-1));
    register_header_dissector(dissect_header_alg, g_variant_new_int64(1));
    register_header_dissector(dissect_header_crit, g_variant_new_int64(2));
    register_header_dissector(dissect_header_ctype, g_variant_new_int64(3));
    register_header_dissector(dissect_header_kid, g_variant_new_int64(4));
    register_header_dissector(dissect_header_iv, g_variant_new_int64(5));
    register_header_dissector(dissect_header_piv, g_variant_new_int64(6));
    // draft-ietf-cose-x509 header labels
    register_header_dissector(dissect_header_x5bag, g_variant_new_int64(32));
    register_header_dissector(dissect_header_x5chain, g_variant_new_int64(33));
    register_header_dissector(dissect_header_x5t, g_variant_new_int64(34));
    register_header_dissector(dissect_header_x5u, g_variant_new_int64(35));

    dissector_add_string("media_type", "application/cose-key", handle_cose_key);
    dissector_add_string("media_type", "application/cose-key-set", handle_cose_key_set);
    // RFC 8152 key parameter labels
    register_keyparam_dissector(dissect_keyparam_kty, NULL, g_variant_new_int64(1));
    register_keyparam_dissector(dissect_header_kid, NULL, g_variant_new_int64(2));
    register_keyparam_dissector(dissect_header_alg, NULL, g_variant_new_int64(3));
    register_keyparam_dissector(dissect_keyparam_keyops, NULL, g_variant_new_int64(4));
    register_keyparam_dissector(dissect_keyparam_baseiv, NULL, g_variant_new_int64(5));
    // kty-specific parameters
    {
        GVariant *kty = g_variant_new_int64(1);
        register_keyparam_dissector(dissect_keyparam_crv, kty, g_variant_new_int64(-1));
        register_keyparam_dissector(dissect_keyparam_xcoord, kty, g_variant_new_int64(-2));
        register_keyparam_dissector(dissect_keyparam_dcoord, kty, g_variant_new_int64(-3));
    }
    {
        GVariant *kty = g_variant_new_int64(2);
        register_keyparam_dissector(dissect_keyparam_crv, kty, g_variant_new_int64(-1));
        register_keyparam_dissector(dissect_keyparam_xcoord, kty, g_variant_new_int64(-2));
        register_keyparam_dissector(dissect_keyparam_ycoord, kty, g_variant_new_int64(-3));
        register_keyparam_dissector(dissect_keyparam_dcoord, kty, g_variant_new_int64(-4));
    }
    {
        GVariant *kty = g_variant_new_int64(4);
        register_keyparam_dissector(dissect_keyparam_k, kty, g_variant_new_int64(-1));
    }

    cose_reinit();
}
