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

#include "CompassCalDlg.h"
#include <QBoxLayout>
#include <QFormLayout>
#include <QLabel>

CompassCalDlg::CompassCalDlg(QWidget *parent)
    : QDialog(parent, Qt::WindowCloseButtonHint | Qt::WindowTitleHint)
{
    layoutWindow();
    setWindowTitle("Compass calibration");
    connect(m_buttons, SIGNAL(accepted()), this, SLOT(onOk()));
    connect(m_buttons, SIGNAL(rejected()), this, SLOT(onCancel()));

    m_timer = startTimer(100);

    m_compassMin = RTVector3(10000, 10000, 10000);
    m_compassMax = RTVector3(-10000, -10000, -10000);
}

CompassCalDlg::~CompassCalDlg()
{
}

void CompassCalDlg::newCalData(const RTVector3& compass)
{
    if (compass.x() < m_compassMin.x())
        m_compassMin.setX(compass.x());

    if (compass.x() > m_compassMax.x())
        m_compassMax.setX(compass.x());

    if (compass.y() < m_compassMin.y())
        m_compassMin.setY(compass.y());

    if (compass.y() > m_compassMax.y())
        m_compassMax.setY(compass.y());

    if (compass.z() < m_compassMin.z())
        m_compassMin.setZ(compass.z());

    if (compass.z() > m_compassMax.z())
        m_compassMax.setZ(compass.z());

}

void CompassCalDlg::timerEvent(QTimerEvent *)
{
    m_compassMinX->setText(QString::number(m_compassMin.x()));
    m_compassMinY->setText(QString::number(m_compassMin.y()));
    m_compassMinZ->setText(QString::number(m_compassMin.z()));
    m_compassMaxX->setText(QString::number(m_compassMax.x()));
    m_compassMaxY->setText(QString::number(m_compassMax.y()));
    m_compassMaxZ->setText(QString::number(m_compassMax.z()));
}

void CompassCalDlg::onOk()
{
    QMessageBox msgBox;

    killTimer(m_timer);

    msgBox.setText("Saving new compass calibration settings");
    msgBox.setIcon(QMessageBox::Information);
    msgBox.exec();

    accept();
}

void CompassCalDlg::onCancel()
{
    killTimer(m_timer);
    reject();
}

void CompassCalDlg::layoutWindow()
{
    QHBoxLayout *dataLayout;

    setModal(true);

    QVBoxLayout *centralLayout = new QVBoxLayout(this);
    centralLayout->setSpacing(20);
    centralLayout->setContentsMargins(11, 11, 11, 11);

    centralLayout->addWidget(new QLabel("Compass min values (uT): "));

    m_compassMinX = new QLabel("0");
    m_compassMinX->setFrameStyle(QFrame::Panel);
    m_compassMinY = new QLabel("0");
    m_compassMinY->setFrameStyle(QFrame::Panel);
    m_compassMinZ = new QLabel("0");
    m_compassMinZ->setFrameStyle(QFrame::Panel);
    dataLayout = new QHBoxLayout();
    dataLayout->addSpacing(30);
    dataLayout->addWidget(m_compassMinX);
    dataLayout->addWidget(m_compassMinY);
    dataLayout->addWidget(m_compassMinZ);
    centralLayout->addLayout(dataLayout);

    centralLayout->addSpacing(10);
    centralLayout->addWidget(new QLabel("Compass max values (uT): "));

    m_compassMaxX = new QLabel("0");
    m_compassMaxX->setFrameStyle(QFrame::Panel);
    m_compassMaxY = new QLabel("0");
    m_compassMaxY->setFrameStyle(QFrame::Panel);
    m_compassMaxZ = new QLabel("0");
    m_compassMaxZ->setFrameStyle(QFrame::Panel);
    dataLayout = new QHBoxLayout();
    dataLayout->addSpacing(30);
    dataLayout->addWidget(m_compassMaxX);
    dataLayout->addWidget(m_compassMaxY);
    dataLayout->addWidget(m_compassMaxZ);
    centralLayout->addLayout(dataLayout);

    m_buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal, this);
    m_buttons->setCenterButtons(true);

    centralLayout->addWidget(m_buttons);
    setMinimumWidth(400);
}

