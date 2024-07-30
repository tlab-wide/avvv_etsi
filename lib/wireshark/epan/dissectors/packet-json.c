/* packet-json.c
 * Routines for JSON dissection
 * References:
 *     RFC 4627: https://tools.ietf.org/html/rfc4627
 *     Website:  http://json.org/
 *
 * Copyright 2010, Jakub Zawadzki <darkjames-ws@darkjames.pl>
 *
 * Wireshark - Network traffic analyzer
 * By Gerald Combs <gerald@wireshark.org>
 * Copyright 1998 Gerald Combs
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#define NEW_PROTO_TREE_API

#include "config.h"

#include <epan/packet.h>
#include <epan/tvbparse.h>
#include <epan/proto_data.h>
#include <wsutil/wsjson.h>

#include <wsutil/str_util.h>
#include <wsutil/unicode-utils.h>

#include <wiretap/wtap.h>

#include "packet-http.h"
#include "packet-acdr.h"
#include "packet-gtpv2.h"

void proto_register_json(void);
void proto_reg_handoff_json(void);
static char* json_string_unescape(tvbparse_elem_t *tok, gboolean enclose_in_quotation_marks);


static dissector_handle_t json_handle;

static int proto_json = -1;
static int proto_json_3gpp = -1;

//Used to get AC DR proto data
static int proto_acdr = -1;

static gint ett_json = -1;
static gint ett_json_array = -1;
static gint ett_json_object = -1;
static gint ett_json_member = -1;
/* Define the trees for json compact form */
static gint ett_json_compact = -1;
static gint ett_json_array_compact = -1;
static gint ett_json_object_compact = -1;
static gint ett_json_member_compact = -1;
static gint ett_json_base64decoded_eps_ie = -1;

static header_field_info *hfi_json = NULL;

#define JSON_HFI_INIT HFI_INIT(proto_json)

static header_field_info hfi_json_array JSON_HFI_INIT =
	{ "Array", "json.array", FT_NONE, BASE_NONE, NULL, 0x00, "JSON array", HFILL };

static header_field_info hfi_json_object JSON_HFI_INIT =
	{ "Object", "json.object", FT_NONE, BASE_NONE, NULL, 0x00, "JSON object", HFILL };

static header_field_info hfi_json_member JSON_HFI_INIT =
	{ "Member", "json.member", FT_STRING, STR_UNICODE, NULL, 0x00, "JSON object member", HFILL };

static header_field_info hfi_json_key JSON_HFI_INIT =
	{ "Key", "json.key", FT_STRING, STR_UNICODE, NULL, 0x00, NULL, HFILL };

static header_field_info hfi_json_path JSON_HFI_INIT =
	{ "Path", "json.path", FT_STRING, STR_UNICODE, NULL, 0x00, NULL, HFILL };

static header_field_info hfi_json_path_with_value JSON_HFI_INIT =
	{ "Path with value", "json.path_with_value", FT_STRING, STR_UNICODE, NULL, 0x00, NULL, HFILL };

static header_field_info hfi_json_member_with_value JSON_HFI_INIT =
	{ "Member with value", "json.member_with_value", FT_STRING, STR_UNICODE, NULL, 0x00, NULL, HFILL };

static header_field_info hfi_json_value_string JSON_HFI_INIT = /* FT_STRINGZ? */
	{ "String value", "json.value.string", FT_STRING, STR_UNICODE, NULL, 0x00, "JSON string value", HFILL };

static header_field_info hfi_json_value_number JSON_HFI_INIT = /* FT_DOUBLE/ FT_INT64? */
	{ "Number value", "json.value.number", FT_STRING, BASE_NONE, NULL, 0x00, "JSON number value", HFILL };

static header_field_info hfi_json_value_false JSON_HFI_INIT =
	{ "False value", "json.value.false", FT_NONE, BASE_NONE, NULL, 0x00, "JSON false value", HFILL };

static header_field_info hfi_json_value_null JSON_HFI_INIT =
	{ "Null value", "json.value.null", FT_NONE, BASE_NONE, NULL, 0x00, "JSON null value", HFILL };

static header_field_info hfi_json_value_true JSON_HFI_INIT =
	{ "True value", "json.value.true", FT_NONE, BASE_NONE, NULL, 0x00, "JSON true value", HFILL };

static header_field_info hfi_json_value_nan JSON_HFI_INIT =
	{ "NaN value", "json.value.nan", FT_NONE, BASE_NONE, NULL, 0x00, "JSON NaN value", HFILL };

/* HFIs below are used only for compact form display */
static header_field_info hfi_json_array_compact JSON_HFI_INIT =
	{ "Array compact", "json.array_compact", FT_NONE, BASE_NONE, NULL, 0x00, "JSON array compact", HFILL };

static header_field_info hfi_json_object_compact JSON_HFI_INIT =
	{ "Object compact", "json.object_compact", FT_NONE, BASE_NONE, NULL, 0x00, "JSON object compact", HFILL };

static header_field_info hfi_json_member_compact JSON_HFI_INIT =
	{ "Member compact", "json.member_compact", FT_NONE, BASE_NONE, NULL, 0x00, "JSON member compact", HFILL };

static header_field_info hfi_json_array_item_compact JSON_HFI_INIT =
	{ "Array item compact", "json.array_item_compact", FT_NONE, BASE_NONE, NULL, 0x00, "JSON array item compact", HFILL };

static header_field_info hfi_json_binary_data JSON_HFI_INIT =
	{ "Binary data", "json.binary_data", FT_BYTES, BASE_NONE, NULL, 0x00, "JSON binary data", HFILL };

static header_field_info hfi_json_ignored_leading_bytes JSON_HFI_INIT =
	{ "Ignored leading bytes", "json.ignored_leading_bytes", FT_STRING, STR_UNICODE, NULL, 0x00, NULL, HFILL };

static int hf_json_3gpp_ueepspdnconnection = -1;
static int hf_json_3gpp_bearerlevelqos = -1;
static int hf_json_3gpp_epsbearersetup = -1;
static int hf_json_3gpp_forwardingbearercontexts = -1;
static int hf_json_3gpp_forwardingfteid = -1;
static int hf_json_3gpp_pgwnodename = -1;
static int hf_json_3gpp_pgws8cfteid = -1;
static int hf_json_3gpp_pgws8ufteid = -1;

/* json data decoding function XXXX only works for the compact form.
 * Callback function to further dissect json data
 * The first implementation is a 3GPP json element which carry an Base64 encoded GTPv2 IE
 * https://www.etsi.org/deliver/etsi_ts/129500_129599/129502/15.01.00_60/ts_129502v150100p.pdf
 */
typedef void(*json_data_decoder_func)(tvbuff_t* tvb, proto_tree* tree, packet_info* pinfo, int offset, int len);

/* Array of functions to dissect IEs
*/
typedef struct _json_ie {
    void(*json_data_decoder_func)(tvbuff_t* tvb, proto_tree* tree, packet_info* pinfo, int offset, int len);
} json_ie_t;

/* A struct to hold the hf and callback function stored in a hastable with the json key as key.
 * If the callback is null NULL the filter will be used useful to create filterable items in json.
 * XXX Todo: Implement hte UAT from the http dissector to enable the users to create filters? and/or
 * read config from file, similar to Diameter(filter only)?
 */
typedef struct {
	int *hf_id;
	json_data_decoder_func json_data_decoder;
} json_data_decoder_t;

/* Preferences */
static gboolean json_compact = FALSE;

static gboolean ignore_leading_bytes = FALSE;

static gboolean hide_extended_path_based_filtering = FALSE;

static tvbparse_wanted_t* want;
static tvbparse_wanted_t* want_ignore;

static dissector_handle_t text_lines_handle;

typedef enum {
	JSON_TOKEN_INVALID = -1,
	JSON_TOKEN_NUMBER = 0,
	JSON_TOKEN_STRING,
	JSON_TOKEN_FALSE,
	JSON_TOKEN_NULL,
	JSON_TOKEN_TRUE,
	JSON_TOKEN_NAN,

	/* not really tokens ... */
	JSON_OBJECT,
	JSON_ARRAY

} json_token_type_t;

typedef struct {
	wmem_stack_t *stack;
	wmem_stack_t *stack_compact; /* Used for compact json form only */
	wmem_stack_t *array_idx;	/* Used for compact json form only.
									Top item: -3.
									Object: < 0.
									Array -1: no key, -2: has key  */
	wmem_stack_t* stack_path;
	packet_info* pinfo;
} json_parser_data_t;

#define JSON_COMPACT_TOP_ITEM -3
#define JSON_COMPACT_OBJECT_WITH_KEY -2
#define JSON_COMPACT_OBJECT_WITHOUT_KEY -1
#define JSON_COMPACT_ARRAY 0

#define JSON_ARRAY_BEGIN(json_tvbparse_data) wmem_stack_push(json_tvbparse_data->array_idx, GINT_TO_POINTER(JSON_COMPACT_ARRAY))
#define JSON_OBJECT_BEGIN(json_tvbparse_data) wmem_stack_push(json_tvbparse_data->array_idx, GINT_TO_POINTER(JSON_COMPACT_OBJECT_WITHOUT_KEY))
#define JSON_ARRAY_OBJECT_END(json_tvbparse_data) wmem_stack_pop(json_tvbparse_data->array_idx)
#define JSON_INSIDE_ARRAY(idx) (idx >= JSON_COMPACT_ARRAY)
#define JSON_OBJECT_SET_HAS_KEY(idx) (idx == JSON_COMPACT_OBJECT_WITH_KEY)

static void
json_array_index_increment(json_parser_data_t *data)
{
	gint idx = GPOINTER_TO_INT(wmem_stack_pop(data->array_idx));
	idx++;
	wmem_stack_push(data->array_idx, GINT_TO_POINTER(idx));
}

static void
json_object_add_key(json_parser_data_t *data)
{
	wmem_stack_pop(data->array_idx);
	wmem_stack_push(data->array_idx, GINT_TO_POINTER(JSON_COMPACT_OBJECT_WITH_KEY));
}

static char*
json_string_unescape(tvbparse_elem_t* tok, gboolean enclose_in_quotation_marks)
{
	int read_index = 0;

	wmem_strbuf_t* output_string_buffer = wmem_strbuf_sized_new(wmem_packet_scope(), tok->len, tok->len + 2);

	if (enclose_in_quotation_marks == TRUE)
	{
		wmem_strbuf_append_c(output_string_buffer, '\"');
	}

	while (true)
	{
		// Do not overflow TVB
		if (!tvb_offset_exists(tok->tvb, tok->offset + read_index))
		{
			break;
		}
		// Do not overflow input string
		if (!(read_index < tok->len))
		{
			break;
		}

		guint8 current_character = tvb_get_guint8(tok->tvb, tok->offset + read_index);

		// character that IS NOT escaped
		if (current_character != '\\')
		{
			// A single UTF-8 character can cover more than one byte.
			// Copy all bytes that belong to that character and forward currend_index by that amount of bytes
			int utf8_character_length = ws_utf8_char_len(current_character);

			if (utf8_character_length <= 0)
			{
				break;
			}

			for (int i = 0; i < utf8_character_length; i++)
			{
				// If it is a character of length 1 these checks are redundant.
				// But it avoids a seperate code path since this loop works for lengths from 1 to 6
				// Do not overflow TVB
				if (!tvb_offset_exists(tok->tvb, tok->offset + read_index))
				{
					break;
				}
				// Do not overflow input string
				if (!(read_index < tok->len ))
				{
					break;
				}

				current_character = tvb_get_guint8(tok->tvb, tok->offset + read_index);
				read_index++;
				wmem_strbuf_append_c(output_string_buffer, current_character);
			}
		}
		// character that IS escaped
		else
		{
			read_index++;

			// Do not overflow TVB
			if (!tvb_offset_exists(tok->tvb, tok->offset + read_index))
			{
				break;
			}
			// Do not overflow input string
			if (!(read_index < tok->len))
			{
				break;
			}

			current_character = tvb_get_guint8(tok->tvb, tok->offset + read_index);

			if (current_character == '\"' || current_character == '\\' || current_character == '/')
			{
				read_index++;
				wmem_strbuf_append_c(output_string_buffer, current_character);
			}
			else if (current_character == 'b')
			{
				read_index++;
				wmem_strbuf_append_c(output_string_buffer, '\b');
			}
			else if (current_character == 'f')
			{
				read_index++;
				wmem_strbuf_append_c(output_string_buffer, '\f');
			}
			else if (current_character == 'n')
			{
				read_index++;
				wmem_strbuf_append_c(output_string_buffer, '\n');
			}
			else if (current_character == 'r')
			{
				read_index++;
				wmem_strbuf_append_c(output_string_buffer, '\r');
			}
			else if (current_character == 't')
			{
				read_index++;
				wmem_strbuf_append_c(output_string_buffer, '\t');
			}
			else if (current_character == 'u')
			{
				read_index++;

				guint32 code_point = 0;
				gboolean is_valid_unicode_character = TRUE;

				for (int i = 0; i < 4; i++)
				{
					// Do not overflow TVB
					if (!tvb_offset_exists(tok->tvb, tok->offset + read_index))
					{
						is_valid_unicode_character = FALSE;
						break;
					}
					// Do not overflow input string
					if (!(read_index < tok->len))
					{
						is_valid_unicode_character = FALSE;
						break;
					}

					current_character = tvb_get_guint8(tok->tvb, tok->offset + read_index);
					read_index++;

					int nibble = ws_xton(current_character);

					if(nibble < 0)
					{
						is_valid_unicode_character = FALSE;
						break;
					}

					code_point <<= 4;
					code_point |= nibble;
				}

				if ((IS_LEAD_SURROGATE(code_point)))
				{
					// Do not overflow TVB
					if (!tvb_offset_exists(tok->tvb, tok->offset + read_index))
					{
						break;
					}
					// Do not overflow input string
					if (!(read_index < tok->len))
					{
						break;
					}
					current_character = tvb_get_guint8(tok->tvb, tok->offset + read_index);

					if (current_character == '\\')
					{
						read_index++;

						// Do not overflow TVB
						if (!tvb_offset_exists(tok->tvb, tok->offset + read_index))
						{
							break;
						}
						// Do not overflow input string
						if (!(read_index < tok->len))
						{
							break;
						}

						current_character = tvb_get_guint8(tok->tvb, tok->offset + read_index);
						if (current_character == 'u') {
							guint16 lead_surrogate = code_point;
							guint16 trail_surrogate = 0;

							read_index++;

							for (int i = 0; i < 4; i++)
							{
								// Do not overflow TVB
								if (!tvb_offset_exists(tok->tvb, tok->offset + read_index))
								{
									is_valid_unicode_character = FALSE;
									break;
								}
								// Do not overflow input string
								if (!(read_index < tok->len))
								{
									is_valid_unicode_character = FALSE;
									break;
								}

								current_character = tvb_get_guint8(tok->tvb, tok->offset + read_index);
								read_index++;

								int nibble = ws_xton(current_character);

								if (nibble < 0)
								{
									is_valid_unicode_character = FALSE;
									break;
								}

								trail_surrogate <<= 4;
								trail_surrogate |= nibble;
							}

							if ((IS_TRAIL_SURROGATE(trail_surrogate)))
							{
								code_point = SURROGATE_VALUE(lead_surrogate, trail_surrogate);
							}
							else
							{
								is_valid_unicode_character = FALSE;
							}
						}
						else
						{
							read_index++;
							is_valid_unicode_character = FALSE;
						}
					}
					else
					{
						read_index++;
						is_valid_unicode_character = FALSE;
					}
				}
				else if ((IS_TRAIL_SURROGATE(code_point)))
				{
					is_valid_unicode_character = FALSE;
				}

				if (is_valid_unicode_character)
				{
					if (g_unichar_validate(code_point) && g_unichar_isprint(code_point))
					{
						gchar length_test_buffer[6];
						int utf8_character_length = (int)g_unichar_to_utf8(code_point, length_test_buffer);

						for (int i = 0; i < utf8_character_length; i++)
						{
							// Do not overflow TVB
							if (!tvb_offset_exists(tok->tvb, tok->offset + read_index))
							{
								break;
							}
							// Do not overflow input string
							if (!(read_index < tok->len))
							{
								break;
							}

							current_character = length_test_buffer[i];
							wmem_strbuf_append_c(output_string_buffer, current_character);

						}
					}
				}
				else
				{
					wmem_strbuf_append_unichar(output_string_buffer, 0xFFFD);
				}
			}
			else
			{
				/* not valid by JSON grammar (tvbparse rules should not allow it) */
				DISSECTOR_ASSERT_NOT_REACHED();
			}
		}
	}

	if (enclose_in_quotation_marks == TRUE)
	{
		wmem_strbuf_append_c(output_string_buffer, '\"');
	}

	char* output_string = wmem_strbuf_finalize(output_string_buffer);

	return output_string;
}

static GHashTable* header_fields_hash = NULL;

static proto_item*
json_key_lookup(proto_tree* tree, tvbparse_elem_t* tok, char* key_str, packet_info* pinfo, gboolean use_compact)
{
	proto_item* ti;
	int hf_id = -1;
	header_field_info* hfi;

	json_data_decoder_t* json_data_decoder_rec = (json_data_decoder_t*)g_hash_table_lookup(header_fields_hash, key_str);
	if (json_data_decoder_rec == NULL) {
		return NULL;
	}

	hf_id = *json_data_decoder_rec->hf_id;

	hfi = proto_registrar_get_nth(hf_id);
	DISSECTOR_ASSERT(hfi != NULL);

	if (use_compact) {
		int str_len = (int)strlen(key_str);
		ti = proto_tree_add_item(tree, hfi, tok->tvb, tok->offset + (4 + str_len), tok->len - (5 + str_len), ENC_NA);
		if (json_data_decoder_rec->json_data_decoder) {
			(*json_data_decoder_rec->json_data_decoder)(tok->tvb, tree, pinfo, tok->offset + (4 + str_len), tok->len - (5 + str_len));
		}
	} else {
		ti = proto_tree_add_item(tree, hfi, tok->tvb, tok->offset, tok->len, ENC_NA);
		if (json_data_decoder_rec->json_data_decoder) {
			(*json_data_decoder_rec->json_data_decoder)(tok->tvb, tree, pinfo, tok->offset, tok->len);
		}
	}
	return ti;

}

static char*
join_strings(char* string_a, char* string_b, char separator)
{
	if (string_a == NULL)
	{
		return NULL;
	}
	if (string_b == NULL)
	{
		return NULL;
	}

	wmem_strbuf_t* output_string_buffer = wmem_strbuf_new(wmem_packet_scope(), string_a);

	if (separator != '\0')
	{
		wmem_strbuf_append_c(output_string_buffer, separator);
	}

	wmem_strbuf_append(output_string_buffer, string_b);

	char* output_string = wmem_strbuf_finalize(output_string_buffer);
	return output_string;
}

static int
dissect_json(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void* data)
{
	proto_tree *json_tree = NULL;
	proto_item *ti = NULL;

	json_parser_data_t parser_data;
	tvbparse_t *tt;

	http_message_info_t *message_info;
	const char *data_name;
	int offset;

	/* Save pinfo*/
	parser_data.pinfo = pinfo;
	/* JSON dissector can be called in a JSON native file or when transported
	 * by another protocol, will make entry in the Protocol column on summary display accordingly
	 */
	wmem_list_frame_t *proto = wmem_list_frame_prev(wmem_list_tail(pinfo->layers));
	if (proto) {
		const char *name = proto_get_protocol_filter_name(GPOINTER_TO_INT(wmem_list_frame_data(proto)));

		if (strcmp(name, "frame")) {
			col_append_sep_str(pinfo->cinfo, COL_PROTOCOL, "/", "JSON");
			col_append_sep_str(pinfo->cinfo, COL_INFO, NULL, "JavaScript Object Notation");
		} else {
			col_set_str(pinfo->cinfo, COL_PROTOCOL, "JSON");
			col_set_str(pinfo->cinfo, COL_INFO, "JavaScript Object Notation");
		}
	}

	data_name = pinfo->match_string;
	if (! (data_name && data_name[0])) {
		/*
		 * No information from "match_string"
		 */
		message_info = (http_message_info_t *)data;
		if (message_info == NULL) {
			/*
			 * No information from dissector data
			 */
			data_name = NULL;
		} else {
			data_name = message_info->media_str;
			if (! (data_name && data_name[0])) {
				/*
				 * No information from dissector data
				 */
				data_name = NULL;
			}
		}
	}

	if (tree) {
		ti = proto_tree_add_item(tree, hfi_json, tvb, 0, -1, ENC_NA);
		json_tree = proto_item_add_subtree(ti, ett_json);

		if (data_name)
			proto_item_append_text(ti, ": %s", data_name);
	}

	offset = 0;
	/* XXX*/
	p_add_proto_data(pinfo->pool, pinfo, proto_json, 0, tvb);

	parser_data.stack = wmem_stack_new(wmem_packet_scope());
	wmem_stack_push(parser_data.stack, json_tree);

	// extended path based filtering
	parser_data.stack_path = wmem_stack_new(wmem_packet_scope());
	wmem_stack_push(parser_data.stack_path, "");
	wmem_stack_push(parser_data.stack_path, "");

	int buffer_length = (int)tvb_captured_length(tvb);
	if (ignore_leading_bytes)
	{
		while (offset < buffer_length)
		{
			guint8 current_character = tvb_get_guint8(tvb, offset);
			if (current_character == '[' || current_character == '{')
			{
				break;
			}
			offset++;
		}

		if(offset > 0)
		{
			proto_tree_add_item(json_tree ? json_tree : tree, &hfi_json_ignored_leading_bytes, tvb, 0, offset, ENC_NA);
		}
	}

	if (json_compact) {
		proto_tree* json_tree_compact = NULL;
		json_tree_compact = proto_tree_add_subtree(json_tree, tvb, 0, -1, ett_json_compact, NULL, "JSON compact form:");

		parser_data.stack_compact = wmem_stack_new(wmem_packet_scope());
		wmem_stack_push(parser_data.stack_compact, json_tree_compact);

		parser_data.array_idx = wmem_stack_new(wmem_packet_scope());
		wmem_stack_push(parser_data.array_idx, GINT_TO_POINTER(JSON_COMPACT_TOP_ITEM)); /* top element */
	}

	tt = tvbparse_init(pinfo->pool, tvb, offset, buffer_length - offset, &parser_data, want_ignore);

	/* XXX, only one json in packet? */
	while (tvbparse_get(tt, want))
	{ }

	offset = tvbparse_curr_offset(tt);

	proto_item_set_len(ti, offset);

	/* if we have some unparsed data, pass to data-text-lines dissector (?) */
	if (tvb_reported_length_remaining(tvb, offset) > 0) {
		tvbuff_t *next_tvb;

		next_tvb = tvb_new_subset_remaining(tvb, offset);

		call_dissector_with_data(text_lines_handle, next_tvb, pinfo, tree, data);
	} else if (data_name) {
		col_append_sep_fstr(pinfo->cinfo, COL_INFO, " ", "(%s)", data_name);
	}

	return tvb_captured_length(tvb);
}

/*
 * For dissecting JSON in a file; we don't get passed a media type.
 */
static int
dissect_json_file(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void* data _U_)
{
	return dissect_json(tvb, pinfo, tree, NULL);
}

static void
before_object(void *tvbparse_data, const void *wanted_data _U_, tvbparse_elem_t *tok) {
	json_parser_data_t *data = (json_parser_data_t *) tvbparse_data;

	proto_tree *tree = (proto_tree *)wmem_stack_peek(data->stack);
	proto_tree *subtree;
	proto_item *ti;

	ti = proto_tree_add_item(tree, &hfi_json_object, tok->tvb, tok->offset, tok->len, ENC_NA);

	subtree = proto_item_add_subtree(ti, ett_json_object);
	wmem_stack_push(data->stack, subtree);

	if (json_compact) {
		proto_tree *tree_compact = (proto_tree *)wmem_stack_peek(data->stack_compact);
		proto_tree *subtree_compact;
		proto_item *ti_compact;

		gint idx = GPOINTER_TO_INT(wmem_stack_peek(data->array_idx));

		if (JSON_INSIDE_ARRAY(idx)) {
			ti_compact = proto_tree_add_none_format(tree_compact, &hfi_json_object_compact, tok->tvb, tok->offset, tok->len, "%d:", idx);
			subtree_compact = proto_item_add_subtree(ti_compact, ett_json_object_compact);
			json_array_index_increment(data);
		} else {
			subtree_compact = tree_compact;
		}
		wmem_stack_push(data->stack_compact, subtree_compact);

		JSON_OBJECT_BEGIN(data);
	}
}

static void
after_object(void *tvbparse_data, const void *wanted_data _U_, tvbparse_elem_t *elem _U_) {
	json_parser_data_t *data = (json_parser_data_t *) tvbparse_data;

	wmem_stack_pop(data->stack);

	if (json_compact) {
		proto_tree *tree_compact = (proto_tree *)wmem_stack_peek(data->stack_compact);
		proto_item *parent_item = proto_tree_get_parent(tree_compact);

		gint idx = GPOINTER_TO_INT(wmem_stack_peek(data->array_idx));

		if (JSON_OBJECT_SET_HAS_KEY(idx))
			proto_item_append_text(parent_item, " {...}");
		else
			proto_item_append_text(parent_item, " {}");

		wmem_stack_pop(data->stack_compact);

		JSON_ARRAY_OBJECT_END(data);
	}
}

static void
before_member(void *tvbparse_data, const void *wanted_data _U_, tvbparse_elem_t *tok) {
	json_parser_data_t *data = (json_parser_data_t *) tvbparse_data;

	proto_tree *tree = (proto_tree *)wmem_stack_peek(data->stack);
	proto_tree *subtree;
	proto_item *ti;

	// tvb parse element covers the qutation marks which we don't want
	tvbparse_elem_t key_parse_element = tok->sub[0];
	key_parse_element.offset += 1;
	key_parse_element.len -= 2;
	char* key_string_without_quotation_marks = json_string_unescape(&key_parse_element, FALSE);

	char* key_string_with_quotation_marks = json_string_unescape(tok->sub, FALSE);

	ti = proto_tree_add_string(tree, &hfi_json_member, tok->tvb, tok->offset, tok->len, key_string_without_quotation_marks);

	subtree = proto_item_add_subtree(ti, ett_json_member);
	wmem_stack_push(data->stack, subtree);

	// extended path based filtering
	char* last_key_string = (char*)wmem_stack_pop(data->stack_path);
	char* base_path = (char*)wmem_stack_pop(data->stack_path);
	wmem_stack_push(data->stack_path, base_path);
	wmem_stack_push(data->stack_path, last_key_string);

	char* path = join_strings(base_path, key_string_without_quotation_marks, '/');
	wmem_stack_push(data->stack_path, path);
	wmem_stack_push(data->stack_path, key_string_without_quotation_marks);

	if (json_compact) {
		proto_tree *tree_compact = (proto_tree *)wmem_stack_peek(data->stack_compact);
		proto_tree *subtree_compact;
		proto_item *ti_compact = NULL;

		tvbparse_elem_t *key_tok = tok->sub;

		if (key_tok && key_tok->id == JSON_TOKEN_STRING) {
			ti_compact = json_key_lookup(tree_compact, tok, key_string_without_quotation_marks, data->pinfo, TRUE);
			if (!ti_compact) {
				ti_compact = proto_tree_add_none_format(tree_compact, &hfi_json_member_compact, tok->tvb, tok->offset, tok->len, "%s:", key_string_with_quotation_marks);
			}
		} else {
			ti_compact = proto_tree_add_item(tree_compact, &hfi_json_member_compact, tok->tvb, tok->offset, tok->len, ENC_NA);
		}

		subtree_compact = proto_item_add_subtree(ti_compact, ett_json_member_compact);
		wmem_stack_push(data->stack_compact, subtree_compact);
	}
}

static void
after_member(void *tvbparse_data, const void *wanted_data _U_, tvbparse_elem_t *tok) {
	json_parser_data_t *data = (json_parser_data_t *) tvbparse_data;

	proto_tree *tree = (proto_tree *)wmem_stack_pop(data->stack);

	tvbparse_elem_t* key_tok = tok->sub;
	if (tree && key_tok && key_tok->id == JSON_TOKEN_STRING) {

		tvbparse_elem_t key_parse_element = key_tok[0];
		key_parse_element.offset += 1;
		key_parse_element.len -= 2;
		char* key_string_without_quotation_marks = json_string_unescape(&key_parse_element, FALSE);

		proto_tree_add_string(tree, &hfi_json_key, key_tok->tvb, key_tok->offset, key_tok->len, key_string_without_quotation_marks);
	}

	// extended path based filtering
	wmem_stack_pop(data->stack_path); // Pop key
	char* path = (char*)wmem_stack_pop(data->stack_path);
	if (tree)
	{
		proto_item* path_item = proto_tree_add_string(tree, &hfi_json_path, tok->tvb, tok->offset, tok->len, path);
		proto_item_set_generated(path_item);
		if (hide_extended_path_based_filtering)
		{
			proto_item_set_hidden(path_item);
		}
	}

	if (json_compact) {
		wmem_stack_pop(data->stack_compact);
		json_object_add_key(data);
	}
}

static void
before_array(void *tvbparse_data, const void *wanted_data _U_, tvbparse_elem_t *tok) {
	json_parser_data_t *data = (json_parser_data_t *) tvbparse_data;

	proto_tree *tree = (proto_tree *)wmem_stack_peek(data->stack);
	proto_tree *subtree;
	proto_item *ti;

	ti = proto_tree_add_item(tree, &hfi_json_array, tok->tvb, tok->offset, tok->len, ENC_NA);

	subtree = proto_item_add_subtree(ti, ett_json_array);
	wmem_stack_push(data->stack, subtree);

	// extended path based filtering
	char* last_key_string = (char*)wmem_stack_pop(data->stack_path);
	char* base_path = (char*)wmem_stack_pop(data->stack_path);
	wmem_stack_push(data->stack_path, base_path);
	wmem_stack_push(data->stack_path, last_key_string);

	char* path = join_strings(base_path, "[]", '/');

	wmem_stack_push(data->stack_path, path);
	wmem_stack_push(data->stack_path, "[]");

	// Try key_lookup
	json_key_lookup(tree, tok, last_key_string, data->pinfo, FALSE);

	if (json_compact) {
		JSON_ARRAY_BEGIN(data);
	}
}

static void
after_array(void *tvbparse_data, const void *wanted_data _U_, tvbparse_elem_t *elem _U_) {
	json_parser_data_t *data = (json_parser_data_t *) tvbparse_data;

	wmem_stack_pop(data->stack);

	// extended path based filtering
	wmem_stack_pop(data->stack_path); // Pop key
	wmem_stack_pop(data->stack_path); // Pop path

	if (json_compact) {
		proto_tree *tree_compact = (proto_tree *)wmem_stack_peek(data->stack_compact);
		proto_item *parent_item = proto_tree_get_parent(tree_compact);

		gint idx = GPOINTER_TO_INT(wmem_stack_peek(data->array_idx));
		if (idx == 0)
			proto_item_append_text(parent_item, " []");
		else
			proto_item_append_text(parent_item, " [...]");

		JSON_ARRAY_OBJECT_END(data);
	}
}

static void
after_value(void *tvbparse_data, const void *wanted_data _U_, tvbparse_elem_t *tok) {
	json_parser_data_t *data = (json_parser_data_t *) tvbparse_data;

	proto_tree *tree = (proto_tree *)wmem_stack_peek(data->stack);
	json_token_type_t value_id = tok->sub ? (json_token_type_t)tok->sub->id : JSON_TOKEN_INVALID;

	if (!(value_id == JSON_TOKEN_STRING || value_id == JSON_TOKEN_NUMBER || value_id == JSON_TOKEN_FALSE
		|| value_id == JSON_TOKEN_NULL || value_id == JSON_TOKEN_TRUE || value_id == JSON_TOKEN_NAN))
	{
		return;
	}

	// extended path based filtering
	char* key_string = (char*)wmem_stack_pop(data->stack_path);
	char* path = (char*)wmem_stack_pop(data->stack_path);

	char* value_str = NULL;
	if (value_id == JSON_TOKEN_STRING && tok->len >= 2)
	{
		// tvb parse element covers the qutation marks which we don't want
		tvbparse_elem_t key_parse_element = tok[0];
		key_parse_element.offset += 1;
		key_parse_element.len -= 2;

		value_str = json_string_unescape(&key_parse_element, FALSE);
	}
	else
	{
		value_str = json_string_unescape(tok, FALSE);
	}

	char* path_with_value = join_strings(path, value_str, ':');
	char* memeber_with_value = join_strings(key_string, value_str, ':');
	proto_item* path_with_value_item = proto_tree_add_string(tree, &hfi_json_path_with_value, tok->tvb, tok->offset, tok->len, path_with_value);
	proto_item* member_with_value_item = proto_tree_add_string(tree, &hfi_json_member_with_value, tok->tvb, tok->offset, tok->len, memeber_with_value);

	proto_item_set_generated(path_with_value_item);
	proto_item_set_generated(member_with_value_item);

	if (hide_extended_path_based_filtering)
	{
		proto_item_set_hidden(path_with_value_item);
		proto_item_set_hidden(member_with_value_item);
	}

	wmem_stack_push(data->stack_path, path);
	wmem_stack_push(data->stack_path, key_string);

	switch (value_id) {
		case JSON_TOKEN_STRING:
			if (tok->len >= 2) {
				// Try key_lookup
				proto_item *key_lookup = NULL;
				key_lookup = json_key_lookup(tree, tok, key_string, data->pinfo, FALSE);
				if (!key_lookup) {
					proto_tree_add_string(tree, &hfi_json_value_string, tok->tvb, tok->offset, tok->len, value_str);
				}
			}
			else
			{
				proto_tree_add_item(tree, &hfi_json_value_string, tok->tvb, tok->offset, tok->len, ENC_ASCII | ENC_NA);
			}

			break;

		case JSON_TOKEN_NUMBER:
			/* XXX, convert to number */
			proto_tree_add_item(tree, &hfi_json_value_number, tok->tvb, tok->offset, tok->len, ENC_ASCII|ENC_NA);

			break;

		case JSON_TOKEN_FALSE:
			proto_tree_add_item(tree, &hfi_json_value_false, tok->tvb, tok->offset, tok->len, ENC_NA);

			break;

		case JSON_TOKEN_NULL:
			proto_tree_add_item(tree, &hfi_json_value_null, tok->tvb, tok->offset, tok->len, ENC_NA);

			break;

		case JSON_TOKEN_TRUE:
			proto_tree_add_item(tree, &hfi_json_value_true, tok->tvb, tok->offset, tok->len, ENC_NA);

			break;

		case JSON_TOKEN_NAN:
			proto_tree_add_item(tree, &hfi_json_value_nan, tok->tvb, tok->offset, tok->len, ENC_NA);

			break;

		default:
			proto_tree_add_format_text(tree, tok->tvb, tok->offset, tok->len);
			break;
	}

	if (json_compact) {
		proto_tree *tree_compact = (proto_tree *)wmem_stack_peek(data->stack_compact);

		gint idx = GPOINTER_TO_INT(wmem_stack_peek(data->array_idx));

		char *val_str = tvb_get_string_enc(wmem_packet_scope(), tok->tvb, tok->offset, tok->len, ENC_UTF_8);

		if (JSON_INSIDE_ARRAY(idx)) {
			proto_tree_add_none_format(tree_compact, &hfi_json_array_item_compact, tok->tvb, tok->offset, tok->len, "%d: %s", idx, val_str);
			json_array_index_increment(data);
		} else {
			proto_item *parent_item = proto_tree_get_parent(tree_compact);
			proto_item_append_text(parent_item, " %s", val_str);
		}
	}
}

static void
init_json_parser(void) {
	static tvbparse_wanted_t _want_object;
	static tvbparse_wanted_t _want_array;

	tvbparse_wanted_t *want_object, *want_array;
	tvbparse_wanted_t *want_member;
	tvbparse_wanted_t *want_string;
	tvbparse_wanted_t *want_number, *want_int;
	tvbparse_wanted_t *want_value;
	tvbparse_wanted_t *want_value_separator;

#define tvbparse_optional(id, private_data, before_cb, after_cb, wanted) \
	tvbparse_some(id, 0, 1, private_data, before_cb, after_cb, wanted)

	tvbparse_wanted_t *want_quot = tvbparse_char(-1,"\"",NULL,NULL,NULL);

	want_string = tvbparse_set_seq(JSON_TOKEN_STRING, NULL, NULL, NULL,
			want_quot,
			tvbparse_some(-1, 0, G_MAXINT, NULL, NULL, NULL,
				tvbparse_set_oneof(-1, NULL, NULL, NULL,
					tvbparse_not_chars(-1, 0, 0, "\"" "\\", NULL, NULL, NULL), /* XXX, without invalid unicode characters */
					tvbparse_set_seq(-1, NULL, NULL, NULL,
						tvbparse_char(-1, "\\", NULL, NULL, NULL),
						tvbparse_set_oneof(-1, NULL, NULL, NULL,
							tvbparse_chars(-1, 0, 1, "\"" "\\" "/bfnrt", NULL, NULL, NULL),
							tvbparse_set_seq(-1, NULL, NULL, NULL,
								tvbparse_char(-1, "u", NULL, NULL, NULL),
								tvbparse_chars(-1, 4, 4, "0123456789abcdefABCDEF", NULL, NULL, NULL),
								NULL),
							NULL),
						NULL),
					NULL)
				),
			want_quot,
			NULL);

	want_value_separator = tvbparse_char(-1, ",", NULL, NULL, NULL);

	/* int = zero / ( digit1-9 *DIGIT ) */
	want_int = tvbparse_set_oneof(-1, NULL, NULL, NULL,
			tvbparse_char(-1, "0", NULL, NULL, NULL),
			tvbparse_set_seq(-1, NULL, NULL, NULL,
				tvbparse_chars(-1, 1, 1, "123456789", NULL, NULL, NULL),
				tvbparse_optional(-1, NULL, NULL, NULL, /* tvbparse_chars() don't respect 0 as min_len ;/ */
					tvbparse_chars(-1, 0, 0, "0123456789", NULL, NULL, NULL)),
				NULL),
			NULL);

	/* number = [ minus ] int [ frac ] [ exp ] */
	want_number = tvbparse_set_seq(JSON_TOKEN_NUMBER, NULL, NULL, NULL,
			tvbparse_optional(-1, NULL, NULL, NULL, /* tvbparse_chars() don't respect 0 as min_len ;/ */
				tvbparse_chars(-1, 0, 1, "-", NULL, NULL, NULL)),
			want_int,
			/* frac = decimal-point 1*DIGIT */
			tvbparse_optional(-1, NULL, NULL, NULL,
				tvbparse_set_seq(-1, NULL, NULL, NULL,
					tvbparse_char(-1, ".", NULL, NULL, NULL),
					tvbparse_chars(-1, 1, 0, "0123456789", NULL, NULL, NULL),
					NULL)),
			/* exp = e [ minus / plus ] 1*DIGIT */
			tvbparse_optional(-1, NULL, NULL, NULL,
				tvbparse_set_seq(-1, NULL, NULL, NULL,
					tvbparse_char(-1, "eE", NULL, NULL, NULL),
					tvbparse_optional(-1, NULL, NULL, NULL, /* tvbparse_chars() don't respect 0 as min_len ;/ */
						tvbparse_chars(-1, 0, 1, "-+", NULL, NULL, NULL)),
					tvbparse_chars(-1, 1, 0, "0123456789", NULL, NULL, NULL),
					NULL)),
			NULL);

	/* value = false / null / true / object / array / number / string */
	want_value = tvbparse_set_oneof(-1, NULL, NULL, after_value,
			tvbparse_string(JSON_TOKEN_FALSE, "false", NULL, NULL, NULL),
			tvbparse_string(JSON_TOKEN_NULL, "null", NULL, NULL, NULL),
			tvbparse_string(JSON_TOKEN_TRUE, "true", NULL, NULL, NULL),
			tvbparse_string(JSON_TOKEN_NAN, "NaN", NULL, NULL, NULL),
			&_want_object,
			&_want_array,
			want_number,
			want_string,
			NULL);

	/* array = begin-array [ value *( value-separator value ) ] end-array */
	want_array = tvbparse_set_seq(JSON_ARRAY, NULL, before_array, after_array,
			tvbparse_char(-1, "[", NULL, NULL, NULL),
			tvbparse_optional(-1, NULL, NULL, NULL,
				tvbparse_set_seq(-1, NULL, NULL, NULL,
					want_value,
					tvbparse_some(-1, 0, G_MAXINT, NULL, NULL, NULL,
						tvbparse_set_seq(-1, NULL, NULL, NULL,
							want_value_separator,
							want_value,
							NULL)),
					NULL)
				),
			tvbparse_char(-1, "]", NULL, NULL, NULL),
			NULL);
	_want_array = *want_array;

	/* member = string name-separator value */
	want_member = tvbparse_set_seq(-1, NULL, before_member, after_member,
			want_string,
			tvbparse_char(-1, ":", NULL, NULL, NULL),
			want_value,
			NULL);

	/* object = begin-object [ member *( value-separator member ) ] end-object */
	want_object = tvbparse_set_seq(JSON_OBJECT, NULL, before_object, after_object,
			tvbparse_char(-1, "{", NULL, NULL, NULL),
			tvbparse_optional(-1, NULL, NULL, NULL,
				tvbparse_set_seq(-1, NULL, NULL, NULL,
					want_member,
					tvbparse_some(-1, 0, G_MAXINT, NULL, NULL, NULL,
						tvbparse_set_seq(-1, NULL, NULL, NULL,
							want_value_separator,
							want_member,
							NULL)),
					NULL)
				),
			tvbparse_char(-1, "}", NULL, NULL, NULL),
			NULL);
	_want_object = *want_object;

	want_ignore = tvbparse_chars(-1, 1, 0, " \t\r\n", NULL, NULL, NULL);

	/* JSON-text = object / array */
	want = tvbparse_set_oneof(-1, NULL, NULL, NULL,
		want_object,
		want_array,
		/* tvbparse_not_chars(-1, 1, 0, " \t\r\n", NULL, NULL, NULL), */
		NULL);

	/* XXX, heur? */
}

/* This function tries to understand if the payload is json or not */
static gboolean
dissect_json_heur(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data)
{
	guint len = tvb_captured_length(tvb);
	const guint8* buf = tvb_get_string_enc(wmem_packet_scope(), tvb, 0, len, ENC_ASCII);

	if (json_validate(buf, len) == FALSE)
		return FALSE;

	return (dissect_json(tvb, pinfo, tree, data) != 0);
}

/* This function tries to understand if the payload is sitting on top of AC DR */
static gboolean
dissect_json_acdr_heur(tvbuff_t *tvb, packet_info *pinfo, proto_tree *tree, void *data)
{
	guint acdr_prot = GPOINTER_TO_UINT(p_get_proto_data(pinfo->pool, pinfo, proto_acdr, 0));
	if (acdr_prot == ACDR_VoiceAI)
		return dissect_json_heur(tvb, pinfo, tree, data);
	return FALSE;
}

/* Functions to sub dissect json content */
static void
dissect_base64decoded_eps_ie(tvbuff_t* tvb, proto_tree* tree, packet_info* pinfo, int offset, int len)
{
	/* base64-encoded characters, encoding the
	 * EPS IE specified in 3GPP TS 29.274.
	 */
	proto_item* ti;
	proto_tree* sub_tree;
	tvbuff_t* bin_tvb = base64_tvb_to_new_tvb(tvb, offset, len);
	add_new_data_source(pinfo, bin_tvb, "Base64 decoded");
	ti = proto_tree_add_item(tree, &hfi_json_binary_data, bin_tvb, 0, -1, ENC_NA);
	sub_tree = proto_item_add_subtree(ti, ett_json_base64decoded_eps_ie);
	dissect_gtpv2_ie_common(bin_tvb, pinfo, sub_tree, 0, 0/* Message type 0, Reserved */, NULL);
}

static void
register_static_headers(void) {

	gchar* header_name;

	header_fields_hash = g_hash_table_new_full(g_str_hash, g_str_equal, g_free, g_free);

	/* Here hf[x].hfinfo.name is a header method which is used as key
	 * for matching ids while processing HTTP2 packets */
	static hf_register_info hf[] = {
		{
			&hf_json_3gpp_ueepspdnconnection,
			{"ueEpsPdnConnection", "json.3gpp.ueepspdnconnection",
				FT_STRING, BASE_NONE, NULL, 0x0,
				NULL, HFILL}
		},
		{
			&hf_json_3gpp_bearerlevelqos,
			{"bearerLevelQoS", "json.3gpp.bearerlevelqos",
				FT_STRING, BASE_NONE, NULL, 0x0,
				NULL, HFILL}
		},
		{
			&hf_json_3gpp_epsbearersetup,
			{"epsBearerSetup", "json.3gpp.epsbearersetup",
				FT_STRING, BASE_NONE, NULL, 0x0,
				NULL, HFILL}
		},
		{
			&hf_json_3gpp_forwardingbearercontexts,
			{"forwardingBearerContexts", "json.3gpp.forwardingbearercontexts",
				FT_STRING, BASE_NONE, NULL, 0x0,
				NULL, HFILL}
		},
		{
			&hf_json_3gpp_forwardingfteid,
			{"forwardingFTeid", "json.3gpp.forwardingfteid",
				FT_STRING, BASE_NONE, NULL, 0x0,
				NULL, HFILL}
		},
		{
			&hf_json_3gpp_pgwnodename,
			{"pgwNodeName", "json.3gpp.pgwnodename",
				FT_STRING, BASE_NONE, NULL, 0x0,
				NULL, HFILL}
		},
		{
			&hf_json_3gpp_pgws8cfteid,
			{"pgwS8cFteid", "json.3gpp.pgws8cfteid",
				FT_STRING, BASE_NONE, NULL, 0x0,
				NULL, HFILL}
		},
		{
			&hf_json_3gpp_pgws8ufteid,
			{"pgwS8uFteid", "json.3gpp.pgws8ufteid",
				FT_STRING, BASE_NONE, NULL, 0x0,
				NULL, HFILL}
		}
	};

	/* List of decoding functions the index matches the HF */
	static void(*json_decode_fn[])(tvbuff_t * tvb, proto_tree * tree, packet_info * pinfo, int offset, int len) = {
		dissect_base64decoded_eps_ie,   /* ueEpsPdnConnection */
		dissect_base64decoded_eps_ie,   /* bearerLevelQoS */
		dissect_base64decoded_eps_ie,   /* epsBearerSetup */
		dissect_base64decoded_eps_ie,   /* forwardingBearerContexts */
		dissect_base64decoded_eps_ie,   /* forwardingFTeid */
		dissect_base64decoded_eps_ie,   /* pgwNodeName */
		dissect_base64decoded_eps_ie,   /* pgwS8cFteid */
		dissect_base64decoded_eps_ie,   /* pgwS8uFteid */

		NULL,   /* NONE */
	};

	/* Hfs with functions */
	for (guint i = 0; i < G_N_ELEMENTS(hf); ++i) {
		header_name = g_strdup(hf[i].hfinfo.name);
		json_data_decoder_t* json_data_decoder_rec = g_new(json_data_decoder_t, 1);
		json_data_decoder_rec->hf_id = &hf[i].hfinfo.id;
		json_data_decoder_rec->json_data_decoder = json_decode_fn[i];
		g_hash_table_insert(header_fields_hash, header_name, json_data_decoder_rec);
	}

	proto_register_field_array(proto_json_3gpp, hf, G_N_ELEMENTS(hf));
}

void
proto_register_json(void)
{
	static gint *ett[] = {
		&ett_json,
		&ett_json_array,
		&ett_json_object,
		&ett_json_member,
		&ett_json_compact,
		&ett_json_array_compact,
		&ett_json_object_compact,
		&ett_json_member_compact,
		&ett_json_base64decoded_eps_ie,
	};

#ifndef HAVE_HFI_SECTION_INIT
	static header_field_info *hfi[] = {
		&hfi_json_array,
		&hfi_json_object,
		&hfi_json_member,
		&hfi_json_key,
		&hfi_json_path,
		&hfi_json_path_with_value,
		&hfi_json_member_with_value,
		&hfi_json_value_string,
		&hfi_json_value_number,
		&hfi_json_value_false,
		&hfi_json_value_null,
		&hfi_json_value_true,
		&hfi_json_value_nan,
		&hfi_json_array_compact,
		&hfi_json_object_compact,
		&hfi_json_member_compact,
		&hfi_json_array_item_compact,
		&hfi_json_binary_data,
		&hfi_json_ignored_leading_bytes
	};
#endif

	module_t *json_module;

	proto_json = proto_register_protocol("JavaScript Object Notation", "JSON", "json");
	hfi_json = proto_registrar_get_nth(proto_json);

	proto_register_fields(proto_json, hfi, array_length(hfi));
	proto_register_subtree_array(ett, array_length(ett));

	json_handle = register_dissector("json", dissect_json, proto_json);

	init_json_parser();

	json_module = prefs_register_protocol(proto_json, NULL);
	prefs_register_bool_preference(json_module, "compact_form",
		"Display JSON in compact form",
		"Display JSON like in browsers devtool",
		&json_compact);

	prefs_register_bool_preference(json_module, "ignore_leading_bytes",
		"Ignore leading non JSON bytes",
		"Leading bytes will be ignored until first '[' or '{' is found.",
		&ignore_leading_bytes);

	prefs_register_bool_preference(json_module, "hide_extended_path_based_filtering",
		"Hide extended path based filtering",
		"Hide extended path based filtering",
		&hide_extended_path_based_filtering);

	proto_json_3gpp = proto_register_protocol("JSON 3GPP", "JSON_3GPP", "json_3gpp");

	/* Fill hash table with static headers */
	register_static_headers();
}

void
proto_reg_handoff_json(void)
{
	dissector_handle_t json_file_handle = create_dissector_handle(dissect_json_file, proto_json);

	heur_dissector_add("hpfeeds", dissect_json_heur, "JSON over HPFEEDS", "json_hpfeeds", proto_json, HEURISTIC_ENABLE);
	heur_dissector_add("db-lsp", dissect_json_heur, "JSON over DB-LSP", "json_db_lsp", proto_json, HEURISTIC_ENABLE);
	heur_dissector_add("udp", dissect_json_acdr_heur, "JSON over AC DR", "json_acdr", proto_json, HEURISTIC_ENABLE);
	dissector_add_uint("wtap_encap", WTAP_ENCAP_JSON, json_file_handle);

	dissector_add_for_decode_as("udp.port", json_file_handle);

	dissector_add_string("media_type", "application/json", json_handle); /* RFC 4627 */
	dissector_add_string("media_type", "application/json-rpc", json_handle); /* JSON-RPC over HTTP */
	dissector_add_string("media_type", "application/jsonrequest", json_handle); /* JSON-RPC over HTTP */
	dissector_add_string("media_type", "application/dds-web+json", json_handle); /* DDS Web Integration Service over HTTP */
	dissector_add_string("media_type", "application/vnd.oma.lwm2m+json", json_handle); /* LWM2M JSON over CoAP */
	dissector_add_string("media_type", "application/problem+json", json_handle); /* RFC 7807 Problem Details for HTTP APIs*/
	dissector_add_string("media_type", "application/merge-patch+json", json_handle); /* RFC 7386 HTTP PATCH methods (RFC 5789) */
	dissector_add_string("media_type", "application/json-patch+json", json_handle); /* RFC 6902 JavaScript Object Notation (JSON) Patch */
	dissector_add_string("media_type", "application/x-ndjson", json_handle);
	dissector_add_string("grpc_message_type", "application/grpc+json", json_handle);
	dissector_add_uint_range_with_preference("tcp.port", "", json_file_handle); /* JSON-RPC over TCP */
	dissector_add_uint_range_with_preference("udp.port", "", json_file_handle); /* JSON-RPC over UDP */

	text_lines_handle = find_dissector_add_dependency("data-text-lines", proto_json);

	proto_acdr = proto_get_id_by_filter_name("acdr");
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
