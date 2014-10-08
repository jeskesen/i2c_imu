////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef COMPASSCALDLG_H
#define COMPASSCALDLG_H

#include <QDialog>
#include <qdialogbuttonbox.h>
#include <qmessagebox.h>

#include "RTMath.h"

class CompassCalDlg : public QDialog
{
    Q_OBJECT

public:
    CompassCalDlg(QWidget *parent = 0);
    ~CompassCalDlg();

    const RTVector3& getCompassMin() { return m_compassMin; }
    const RTVector3& getCompassMax() { return m_compassMax; }

public slots:
    void onOk();
    void onCancel();
    void newCalData(const RTVector3& compass);

protected:
    void timerEvent(QTimerEvent *event);

private:
    void layoutWindow();

    QDialogButtonBox *m_buttons;

    QLabel *m_compassMinX;
    QLabel *m_compassMinY;
    QLabel *m_compassMinZ;

    QLabel *m_compassMaxX;
    QLabel *m_compassMaxY;
    QLabel *m_compassMaxZ;

    RTVector3 m_compassMin;
    RTVector3 m_compassMax;

    int m_timer;

};

#endif // COMPASSCALDLG_H
