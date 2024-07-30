/* display_filter_combo.h
 *
 * Wireshark - Network traffic analyzer
 * By Gerald Combs <gerald@wireshark.org>
 * Copyright 1998 Gerald Combs
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef DISPLAY_FILTER_COMBO_H
#define DISPLAY_FILTER_COMBO_H

#include <QComboBox>
#include <QList>

class DisplayFilterCombo : public QComboBox
{
    Q_OBJECT
public:
    explicit DisplayFilterCombo(QWidget *parent = 0);
    bool addRecentCapture(const char *filter);
    void writeRecent(FILE *rf);

protected:
    virtual bool event(QEvent *event);

private:
    void updateStyleSheet();

public slots:
    bool checkDisplayFilter();
    void applyDisplayFilter();
    void setDisplayFilter(QString filter);

private slots:
    void updateMaxCount();
};

#endif // DISPLAY_FILTER_COMBO_H
