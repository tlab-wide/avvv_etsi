/* funnel_string_dialog.h
 *
 * Wireshark - Network traffic analyzer
 * By Gerald Combs <gerald@wireshark.org>
 * Copyright 1998 Gerald Combs
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef FUNNEL_STRING_DIALOG_H
#define FUNNEL_STRING_DIALOG_H

#include <glib.h>

#include "epan/funnel.h"

#include <QDialog>

class QLineEdit;

namespace Ui {
class FunnelStringDialog;
class FunnelStringDialogHelper;
}

class FunnelStringDialog : public QDialog
{
    Q_OBJECT

public:
    explicit FunnelStringDialog(const QString title, const QList<QPair<QString, QString>> field_list, funnel_dlg_cb_t dialog_cb, void* dialog_cb_data, funnel_dlg_cb_data_free_t dialog_data_free_cb);
    ~FunnelStringDialog();

    // Funnel ops
    static void stringDialogNew(const QString title, const QList<QPair<QString, QString>> field_list, funnel_dlg_cb_t dialog_cb, void* dialog_cb_data, funnel_dlg_cb_data_free_t dialog_cb_data_free);

    void accept();
    void reject();

private slots:
    void on_buttonBox_accepted();

private:
    Ui::FunnelStringDialog *ui;
    funnel_dlg_cb_t dialog_cb_;
    void *dialog_cb_data_;
    funnel_dlg_cb_data_free_t dialog_cb_data_free_;
    QList<QLineEdit *> field_edits_;
};

class FunnelStringDialogHelper : public QObject
{
    Q_OBJECT

public slots:
    void emitCloseDialogs();

signals:
    void closeDialogs();
};

extern "C" {
    void string_dialog_new(const gchar* title, const gchar** field_names, const gchar** field_values, funnel_dlg_cb_t dialog_cb, void* dialog_cb_data, funnel_dlg_cb_data_free_t dialog_cb_data_free);
    void string_dialogs_close(void);
}

#endif // FUNNEL_STRING_DIALOG_H
