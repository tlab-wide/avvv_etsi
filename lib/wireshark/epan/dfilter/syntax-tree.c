/*
 * Wireshark - Network traffic analyzer
 * By Gerald Combs <gerald@wireshark.org>
 * Copyright 2001 Gerald Combs
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "config.h"

#define WS_LOG_DOMAIN LOG_DOMAIN_DFILTER

#include "syntax-tree.h"
#include <wsutil/wmem/wmem.h>
#include <wsutil/str_util.h>
#include "sttype-test.h"

/* Keep track of sttype_t's via their sttype_id_t number */
static sttype_t* type_list[STTYPE_NUM_TYPES];


#define STNODE_MAGIC	0xe9b00b9e


void
sttype_init(void)
{
	sttype_register_function();
	sttype_register_pointer();
	sttype_register_range();
	sttype_register_set();
	sttype_register_string();
	sttype_register_test();
}

void
sttype_cleanup(void)
{
	/* nothing to do */
}


void
sttype_register(sttype_t *type)
{
	sttype_id_t	type_id;

	type_id = type->id;

	/* Check input */
	ws_assert(type_id < STTYPE_NUM_TYPES);

	/* Don't re-register. */
	ws_assert(type_list[type_id] == NULL);

	type_list[type_id] = type;
}

static sttype_t*
sttype_lookup(sttype_id_t type_id)
{
	sttype_t	*result;

	/* Check input */
	ws_assert(type_id < STTYPE_NUM_TYPES);

	result = type_list[type_id];

	/* Check output. */
	ws_assert(result != NULL);

	return result;
}

static void
_node_clear(stnode_t *node)
{
	ws_assert_magic(node, STNODE_MAGIC);
	if (node->type) {
		if (node->type->func_free && node->data) {
			node->type->func_free(node->data);
		}
	}
	else {
		ws_assert(!node->data);
	}

	node->type = NULL;
	node->flags = 0;
	node->data = NULL;
	g_free(node->repr_display);
	node->repr_display = NULL;
	g_free(node->repr_debug);
	node->repr_debug = NULL;
}

void
stnode_clear(stnode_t *node)
{
	_node_clear(node);
	g_free(node->token_value);
	node->token_value = NULL;
}

static void
_node_init(stnode_t *node, sttype_id_t type_id, gpointer data)
{
	sttype_t	*type;

	ws_assert_magic(node, STNODE_MAGIC);
	ws_assert(!node->type);
	ws_assert(!node->data);
	node->flags = 0;
	node->repr_display = NULL;
	node->repr_debug = NULL;

	if (type_id == STTYPE_UNINITIALIZED) {
		node->type = NULL;
		node->data = NULL;
	}
	else {
		/* Creating an initialized node with a NULL pointer is
		 * allowed and needs to be safe. The parser relies on that. */
		type = sttype_lookup(type_id);
		ws_assert(type);
		node->type = type;
		if (type->func_new) {
			node->data = type->func_new(data);
		}
		else {
			node->data = data;
		}
	}
}

void
stnode_init(stnode_t *node, sttype_id_t type_id, gpointer data,  const char *token_value)
{
	_node_init(node, type_id, data);
	ws_assert(node->token_value == NULL);
	node->token_value = g_strdup(token_value);
}

void
stnode_replace(stnode_t *node, sttype_id_t type_id, gpointer data)
{
	uint16_t flags = node->flags; /* Save flags. */
	_node_clear(node);
	_node_init(node, type_id, data);
	node->flags = flags;
}

stnode_t*
stnode_new(sttype_id_t type_id, gpointer data, const char *token_value)
{
	stnode_t	*node;

	node = g_new0(stnode_t, 1);
	node->magic = STNODE_MAGIC;

	stnode_init(node, type_id, data, token_value);

	return node;
}

stnode_t*
stnode_dup(const stnode_t *node)
{
	stnode_t *new;

	ws_assert_magic(node, STNODE_MAGIC);
	new = g_new(stnode_t, 1);
	new->magic = STNODE_MAGIC;
	new->flags = node->flags;
	new->token_value = g_strdup(node->token_value);
	new->repr_display = NULL;
	new->repr_debug = NULL;

	new->type = node->type;
	if (node->type == NULL)
		new->data = NULL;
	else if (node->type->func_dup)
		new->data = node->type->func_dup(node->data);
	else
		new->data = node->data;

	return new;
}

void
stnode_free(stnode_t *node)
{
	ws_assert_magic(node, STNODE_MAGIC);
	stnode_clear(node);
	g_free(node);
}

const char*
stnode_type_name(stnode_t *node)
{
	ws_assert_magic(node, STNODE_MAGIC);
	if (node->type)
		return node->type->name;
	else
		return "UNINITIALIZED";
}

sttype_id_t
stnode_type_id(stnode_t *node)
{
	ws_assert_magic(node, STNODE_MAGIC);
	if (node->type)
		return node->type->id;
	else
		return STTYPE_UNINITIALIZED;
}

gpointer
stnode_data(stnode_t *node)
{
	ws_assert_magic(node, STNODE_MAGIC);
	return node->data;
}

gpointer
stnode_steal_data(stnode_t *node)
{
	ws_assert_magic(node, STNODE_MAGIC);
	gpointer data = node->data;
	ws_assert(data);
	node->data = NULL;
	return data;
}

const char *
stnode_token_value(stnode_t *node)
{
	if (node->token_value) {
		return node->token_value;
	}
	return "<null token value>";
}

gboolean
stnode_inside_parens(stnode_t *node)
{
	return node->flags & STNODE_F_INSIDE_PARENS;
}

void
stnode_set_inside_parens(stnode_t *node, gboolean inside)
{
	if (inside) {
		node->flags |= STNODE_F_INSIDE_PARENS;
	}
	else {
		node->flags &= ~STNODE_F_INSIDE_PARENS;
	}
}

static char *
_node_tostr(stnode_t *node, gboolean pretty)
{
	char *s, *repr;

	if (node->type->func_tostr == NULL)
		s = g_strdup("FIXME");
	else
		s = node->type->func_tostr(node->data, pretty);

	if (pretty)
		return s;

	repr = g_strdup_printf("%s<%s>", stnode_type_name(node), s);
	g_free(s);
	return repr;
}

const char *
stnode_tostr(stnode_t *node, gboolean pretty)
{
	ws_assert_magic(node, STNODE_MAGIC);

	if (pretty && node->repr_display != NULL)
		return node->repr_display;

	if (!pretty && node->repr_debug != NULL)
		return node->repr_debug;

	char *str = _node_tostr(node, pretty);

	if (pretty)
		node->repr_display = str;
	else
		node->repr_debug = str;
	return str;
}

static char *
sprint_node(stnode_t *node)
{
	wmem_strbuf_t *buf = wmem_strbuf_new(NULL, NULL);

	wmem_strbuf_append_printf(buf, "stnode <%p> = {\n", (void *)node);
	wmem_strbuf_append_printf(buf, "\tmagic = 0x%"PRIx32"\n", node->magic);
	wmem_strbuf_append_printf(buf, "\ttype = <%p>\n", (void *)(node->type));
	wmem_strbuf_append_printf(buf, "\tdata = %s\n", stnode_todebug(node));
	wmem_strbuf_append_printf(buf, "\tflags (0x%04"PRIx16") = {\n", node->flags);
	wmem_strbuf_append_printf(buf, "\t\tinside_parens = %s\n",
					true_or_false(stnode_inside_parens(node)));
	wmem_strbuf_append(buf, "\t}\n");
	wmem_strbuf_append_printf(buf, "\ttoken_value = \"%s\"\n", stnode_token_value(node));
	wmem_strbuf_append(buf, "}\n");
	return wmem_strbuf_finalize(buf);
}

void
log_stnode_full(enum ws_log_level level,
			const char *file, int line, const char *func,
			stnode_t *node, const char *msg)
{
	if (!ws_log_msg_is_active(LOG_DOMAIN_DFILTER, level))
		return;

	char *str = sprint_node(node);
	ws_log_write_always_full(LOG_DOMAIN_DFILTER, level,
					file, line, func, "%s:\n%s", msg, str);
	g_free(str);
}

static void
indent(wmem_strbuf_t *buf, int level)
{
	for (int i = 0; i < level * 2; i++) {
		wmem_strbuf_append_c(buf, ' ');
	}
}

static void
visit_tree(wmem_strbuf_t *buf, stnode_t *node, int level)
{
	stnode_t *left, *right;

	if (stnode_type_id(node) == STTYPE_TEST) {
		wmem_strbuf_append_printf(buf, "%s(", stnode_todisplay(node));
		sttype_test_get(node, NULL, &left, &right);
		if (left && right) {
			wmem_strbuf_append_c(buf, '\n');
			indent(buf, level + 1);
			wmem_strbuf_append(buf, "LHS = ");
			visit_tree(buf, left, level + 1);
			wmem_strbuf_append_c(buf, '\n');
			indent(buf, level + 1);
			wmem_strbuf_append(buf, "RHS = ");
			visit_tree(buf, right, level + 1);
			wmem_strbuf_append(buf, "\n");
			indent(buf, level);
		}
		else if (left) {
			visit_tree(buf, left, level);
		}
		else if (right) {
			visit_tree(buf, right, level);
		}
		wmem_strbuf_append(buf, ")");
	}
	else {
		wmem_strbuf_append(buf, stnode_todebug(node));
	}
}

void
log_syntax_tree(enum ws_log_level level, stnode_t *root, const char *msg)
{
	if (!ws_log_msg_is_active(LOG_DOMAIN_DFILTER, level))
		return;

	wmem_strbuf_t *buf = wmem_strbuf_new(NULL, NULL);

	visit_tree(buf, root, 0);
	ws_log_write_always_full(LOG_DOMAIN_DFILTER, level, NULL, -1, NULL,
				"%s:\n%s", msg, wmem_strbuf_get_str(buf));
	wmem_strbuf_destroy(buf);
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
