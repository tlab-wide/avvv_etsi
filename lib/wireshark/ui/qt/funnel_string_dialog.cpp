/* funnel_string_dialog.cpp
 *
 * Wireshark - Network traffic analyzer
 * By Gerald Combs <gerald@wireshark.org>
 * Copyright 1998 Gerald Combs
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "funnel_string_dialog.h"
#include <ui_funnel_string_dialog.h>

#include <QLabel>
#include <QLineEdit>

#include <ui/qt/utils/qt_ui_utils.h>
#include "wireshark_application.h"

// Helper object used for sending close signal to open dialogs from a C function
static FunnelStringDialogHelper dialog_helper_;

const int min_edit_width_ = 20; // em widths
FunnelStringDialog::FunnelStringDialog(const QString title, const QList<QPair<QString, QString>> field_list, funnel_dlg_cb_t dialog_cb, void* dialog_cb_data, funnel_dlg_cb_data_free_t dialog_data_free_cb) :
    QDialog(NULL),
    ui(new Ui::FunnelStringDialog),
    dialog_cb_(dialog_cb),
    dialog_cb_data_(dialog_cb_data),
    dialog_cb_data_free_(dialog_data_free_cb)
{
    ui->setupUi(this);
    setWindowTitle(wsApp->windowTitleString(title));
    int one_em = fontMetrics().height();

    int row = 0;
    QPair<QString, QString> field;
    foreach(field, field_list) {
        QLabel* field_label = new QLabel(field.first, this);
        ui->stringGridLayout->addWidget(field_label, row, 0);
        QLineEdit* field_edit = new QLineEdit(this);
        field_edit->setText(field.second);
        field_edit->setMinimumWidth(one_em * min_edit_width_);
        field_edits_ << field_edit;
        ui->stringGridLayout->addWidget(field_edit, row, 1);
        row++;
    }
}

FunnelStringDialog::~FunnelStringDialog()
{
    if (dialog_cb_data_free_) {
        dialog_cb_data_free_(dialog_cb_data_);
    }

    delete ui;
}

void FunnelStringDialog::accept()
{
    QDialog::accept();

    disconnect();
    deleteLater();
}

void FunnelStringDialog::reject()
{
    QDialog::reject();

    disconnect();
    deleteLater();
}

void FunnelStringDialog::on_buttonBox_accepted()
{
    if (!dialog_cb_) return;

    GPtrArray* returns = g_ptr_array_new();

    foreach (QLineEdit *field_edit, field_edits_) {
        g_ptr_array_add(returns, qstring_strdup(field_edit->text()));
    }
    g_ptr_array_add(returns, NULL);

    gchar **user_input = (gchar **)g_ptr_array_free(returns, FALSE);
    dialog_cb_(user_input, dialog_cb_data_);
}

void FunnelStringDialog::stringDialogNew(const QString title, QList<QPair<QString, QString>> field_list, funnel_dlg_cb_t dialog_cb, void* dialog_cb_data, funnel_dlg_cb_data_free_t dialog_cb_data_free)
{
    FunnelStringDialog* fsd = new FunnelStringDialog(title, field_list, dialog_cb, dialog_cb_data, dialog_cb_data_free);
    connect(&dialog_helper_, &FunnelStringDialogHelper::closeDialogs, fsd, &FunnelStringDialog::close);
    fsd->show();
}

void FunnelStringDialogHelper::emitCloseDialogs()
{
    emit closeDialogs();
}

void string_dialog_new(const gchar* title, const gchar** field_names, const gchar** field_values, funnel_dlg_cb_t dialog_cb, void* dialog_cb_data, funnel_dlg_cb_data_free_t dialog_cb_data_free)
{
    QList<QPair<QString, QString>> field_list;
    for (int i = 0; field_names[i]; i++) {
        QPair<QString, QString> field = QPair<QString, QString>(QString(field_names[i]), QString(""));
        if (field_values != NULL && field_values[i])
        {
            field.second = QString(field_values[i]);
        }

        field_list << field;
    }
    FunnelStringDialog::stringDialogNew(title, field_list, dialog_cb, dialog_cb_data, dialog_cb_data_free);
}

void string_dialogs_close(void)
{
    dialog_helper_.emitCloseDialogs();
}
