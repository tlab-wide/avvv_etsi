/* import_text_dialog.h
 *
 * Wireshark - Network traffic analyzer
 * By Gerald Combs <gerald@wireshark.org>
 * Copyright 1998 Gerald Combs
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef IMPORT_TEXT_DIALOG_H
#define IMPORT_TEXT_DIALOG_H

#include <config.h>

#include <stdio.h>

#include <glib.h>

#include "ui/text_import.h"

#include <ui/qt/widgets/syntax_line_edit.h>

#include <QDialog>
#include <QPushButton>
#include <QRadioButton>

namespace Ui {
class ImportTextDialog;
}

class ImportTextDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ImportTextDialog(QWidget *parent = 0);
    ~ImportTextDialog();
    QString &capfileName();

private:
    void enableHeaderWidgets(bool enable_ethernet_buttons = true, bool enable_export_pdu_buttons = true);

    /* regex fields */
    void enableFieldWidgets(bool enable_direction_input = true, bool enable_time_input = true);

    void check_line_edit(SyntaxLineEdit *le, bool &ok_enable, const QString &num_str, int base, guint max_val, bool is_short, guint *val_ptr);
    bool checkDateTimeFormat(const QString &time_format);

    void loadSettingsFile();
    void saveSettingsFile();
    void applyDialogSettings();
    void storeDialogSettings();

    void updateImportButtonState();

    Ui::ImportTextDialog *ti_ui_;
    QVariantMap settings;

    QPushButton *import_button_;
    QList<QRadioButton *>encap_buttons_;
    text_import_info_t import_info_;
    QString capfile_name_;
    bool file_ok_;
    bool timestamp_format_ok_;

    /* Regex input */
    bool regex_ok_;
    bool re_has_dir_;
    bool in_indication_ok_;
    bool out_indication_ok_;
    bool re_has_time_;

    bool ether_type_ok_;
    bool proto_ok_;
    bool source_port_ok_;
    bool dest_port_ok_;
    bool tag_ok_;
    bool ppi_ok_;
    bool payload_ok_;
    bool max_len_ok_;

public slots:
    int exec();

private slots:
    void on_textFileBrowseButton_clicked();
    void on_textFileLineEdit_textChanged(const QString &arg1);
    void on_modeTabWidget_currentChanged(int index);
    void on_timestampFormatLineEdit_textChanged(const QString &arg1);

    /* Hex Dump input */
    void on_noOffsetButton_toggled(bool checked);
    void on_directionIndicationCheckBox_toggled(bool checked);

    /* Regex input */
    void on_regexTextEdit_textChanged();
    void on_dataEncodingComboBox_currentIndexChanged(int index);
    void on_dirInIndicationLineEdit_textChanged(const QString &arg1);
    void on_dirOutIndicationLineEdit_textChanged(const QString &arg1);

    /* Encapsulation input */
    void on_encapComboBox_currentIndexChanged(int index);
    void on_noDummyButton_toggled(bool checked);
    void on_ethernetButton_toggled(bool checked);
    void on_ipv4Button_toggled(bool checked);
    void on_udpButton_toggled(bool checked);
    void on_tcpButton_toggled(bool checked);
    void on_sctpButton_toggled(bool checked);
    void on_sctpDataButton_toggled(bool checked);
    void on_exportPduButton_toggled(bool checked);
    void on_ethertypeLineEdit_textChanged(const QString &ethertype_str);
    void on_protocolLineEdit_textChanged(const QString &protocol_str);
    void on_sourcePortLineEdit_textChanged(const QString &source_port_str);
    void on_destinationPortLineEdit_textChanged(const QString &destination_port_str);
    void on_tagLineEdit_textChanged(const QString &tag_str);
    void on_ppiLineEdit_textChanged(const QString &ppi_str);

    void on_maxLengthLineEdit_textChanged(const QString &max_frame_len_str);
    void on_buttonBox_helpRequested();
};


#endif // IMPORT_TEXT_DIALOG_H
