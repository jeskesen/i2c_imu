//
//  Copyright (c) 2014 richards-tech
//
//  This file is part of RTIMULib
//
//  RTIMULib is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  RTIMULib is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with RTIMULib.  If not, see <http://www.gnu.org/licenses/>.
//

#include <QMessageBox>
#include <qboxlayout.h>
#include <QSettings>

#include "RTIMULibDemo.h"
#include "CompassCalDlg.h"
#include "IMUThread.h"

#include "RTIMUMPU9150.h"
#include "RTKalman.h"

#define RATE_TIMER_INTERVAL 2

RTIMULibDemo::RTIMULibDemo()
    : QMainWindow()
{
    //  This is some normal Qt GUI stuff

	ui.setupUi(this);

    layoutWindow();
	layoutStatusBar();

    //  This code connects up signals from the GUI

	connect(ui.actionExit, SIGNAL(triggered()), this, SLOT(close()));
    connect(ui.actionCalibrateCompass, SIGNAL(triggered()), this, SLOT(onCalibrateCompass()));
    connect(m_enableGyro, SIGNAL(stateChanged(int)), this, SLOT(onEnableGyro(int)));
    connect(m_enableAccel, SIGNAL(stateChanged(int)), this, SLOT(onEnableAccel(int)));
    connect(m_enableCompass, SIGNAL(stateChanged(int)), this, SLOT(onEnableCompass(int)));
    connect(m_enableDebug, SIGNAL(stateChanged(int)), this, SLOT(onEnableDebug(int)));

    //  create the imu thread and connect up the signal

    m_imuThread = new IMUThread();

    connect(m_imuThread,
            SIGNAL(newIMUData(const RTVector3&, const RTQuaternion&, const RTVector3&, const RTVector3&, const RTVector3&)),
            this,
            SLOT(newIMUData(const RTVector3&, const RTQuaternion&, const RTVector3&, const RTVector3&, const RTVector3&)),
            Qt::DirectConnection);

    m_imuThread->resumeThread();
	
    //  This value allows a sample rate to be calculated

    m_sampleCount = 0;

    //  start some timers to get things going

    m_rateTimer = startTimer(RATE_TIMER_INTERVAL * 1000);

    //  Only update the display 10 times per second to keep CPU reasonable

    m_displayTimer = startTimer(100);
}

RTIMULibDemo::~RTIMULibDemo()
{
}

void RTIMULibDemo::onCalibrateCompass()
{
    CompassCalDlg dlg(this);

    m_imuThread->setCalibrationMode(true);
    connect(m_imuThread, SIGNAL(newCalData(const RTVector3&)), &dlg,
            SLOT(newCalData(const RTVector3&)), Qt::DirectConnection);

    if (dlg.exec() == QDialog::Accepted) {
        m_imuThread->newCompassCalData(dlg.getCompassMin(), dlg.getCompassMax());
        m_imuThread->setCalibrationMode(false);
    }
    disconnect(m_imuThread, SIGNAL(newCalData(const RTVector3&)), &dlg, SLOT(newCalData(const RTVector3&)));
}

void RTIMULibDemo::newIMUData(const RTVector3& kalmanPose, const RTQuaternion& kalmanQPose,
                             const RTVector3& gyro, const RTVector3& accel, const RTVector3& compass)
{
    m_kalmanPose = kalmanPose;
    m_kalmanQPose = kalmanQPose;
    m_gyro = gyro;
    m_accel = accel;
    m_compass = compass;
    m_sampleCount++;
}

void RTIMULibDemo::onEnableGyro(int state)
{
    m_imuThread->getIMU()->setGyroEnable(state == Qt::Checked);
}

void RTIMULibDemo::onEnableAccel(int state)
{
    m_imuThread->getIMU()->setAccelEnable(state == Qt::Checked);
}

void RTIMULibDemo::onEnableCompass(int state)
{
    m_imuThread->getIMU()->setCompassEnable(state == Qt::Checked);
}

void RTIMULibDemo::onEnableDebug(int state)
{
    m_imuThread->getIMU()->setDebugEnable(state == Qt::Checked);
}

void RTIMULibDemo::closeEvent(QCloseEvent *)
{
    killTimer(m_displayTimer);
    killTimer(m_rateTimer);
    m_imuThread->exitThread();
}

void RTIMULibDemo::timerEvent(QTimerEvent *event)
{
    if (event->timerId() == m_displayTimer) {
        //  Update the GUI

        m_gyroX->setText(QString::number(m_gyro.x(), 'g', 4));
        m_gyroY->setText(QString::number(m_gyro.y(), 'g', 4));
        m_gyroZ->setText(QString::number(m_gyro.z(), 'g', 4));

        m_accelX->setText(QString::number(m_accel.x(), 'g', 4));
        m_accelY->setText(QString::number(m_accel.y(), 'g', 4));
        m_accelZ->setText(QString::number(m_accel.z(), 'g', 4));

        m_compassX->setText(QString::number(m_compass.x(), 'g', 4));
        m_compassY->setText(QString::number(m_compass.y(), 'g', 4));
        m_compassZ->setText(QString::number(m_compass.z(), 'g', 4));

        m_kalmanPoseX->setText(QString::number(m_kalmanPose.x() * RTMATH_RAD_TO_DEGREE, 'g', 4));
        m_kalmanPoseY->setText(QString::number(m_kalmanPose.y() * RTMATH_RAD_TO_DEGREE, 'g', 4));
        m_kalmanPoseZ->setText(QString::number(m_kalmanPose.z() * RTMATH_RAD_TO_DEGREE, 'g', 4));

        m_kalmanQPoseScalar->setText(QString::number(m_kalmanQPose.scalar(), 'g', 4));
        m_kalmanQPoseX->setText(QString::number(m_kalmanQPose.x(), 'g', 4));
        m_kalmanQPoseY->setText(QString::number(m_kalmanQPose.y(), 'g', 4));
        m_kalmanQPoseZ->setText(QString::number(m_kalmanQPose.z(), 'g', 4));
    } else {

        //  Update the sample rate

        float rate = (float)m_sampleCount / (float(RATE_TIMER_INTERVAL));
        m_sampleCount = 0;
        m_rateStatus->setText(QString("Sample rate: %1 per second").arg(rate));

        if (m_imuThread->getIMU()->getCalibrationValid())
            m_calStatus->setText("Calibration in use");
        else
            m_calStatus->setText("Uncalibrated");
    }
}

void RTIMULibDemo::layoutWindow()
{
    QVBoxLayout *vLayout = new QVBoxLayout();
    vLayout->setContentsMargins(3, 3, 3, 3);
    vLayout->setSpacing(3);

    vLayout->addWidget(new QLabel("Kalman state (quaternion): "));

    QHBoxLayout *dataLayout = new QHBoxLayout();
    dataLayout->addSpacing(30);
    m_kalmanQPoseScalar = new QLabel("1");
    m_kalmanQPoseScalar->setFrameStyle(QFrame::Panel);
    m_kalmanQPoseX = new QLabel("0");
    m_kalmanQPoseX->setFrameStyle(QFrame::Panel);
    m_kalmanQPoseY = new QLabel("0");
    m_kalmanQPoseY->setFrameStyle(QFrame::Panel);
    m_kalmanQPoseZ = new QLabel("0");
    m_kalmanQPoseZ->setFrameStyle(QFrame::Panel);
    dataLayout->addWidget(m_kalmanQPoseScalar);
    dataLayout->addWidget(m_kalmanQPoseX);
    dataLayout->addWidget(m_kalmanQPoseY);
    dataLayout->addWidget(m_kalmanQPoseZ);
    dataLayout->addSpacing(30);
    vLayout->addLayout(dataLayout);

    vLayout->addSpacing(10);
    vLayout->addWidget(new QLabel("Pose - roll, pitch, yaw (degrees): "));

    m_kalmanPoseX = new QLabel("0");
    m_kalmanPoseX->setFrameStyle(QFrame::Panel);
    m_kalmanPoseY = new QLabel("0");
    m_kalmanPoseY->setFrameStyle(QFrame::Panel);
    m_kalmanPoseZ = new QLabel("0");
    m_kalmanPoseZ->setFrameStyle(QFrame::Panel);
    dataLayout = new QHBoxLayout();
    dataLayout->addSpacing(30);
    dataLayout->addWidget(m_kalmanPoseX);
    dataLayout->addWidget(m_kalmanPoseY);
    dataLayout->addWidget(m_kalmanPoseZ);
    dataLayout->addSpacing(137);
    vLayout->addLayout(dataLayout);

    vLayout->addSpacing(10);
    vLayout->addWidget(new QLabel("Gyros (radians/s): "));

    m_gyroX = new QLabel("0");
    m_gyroX->setFrameStyle(QFrame::Panel);
    m_gyroY = new QLabel("0");
    m_gyroY->setFrameStyle(QFrame::Panel);
    m_gyroZ = new QLabel("0");
    m_gyroZ->setFrameStyle(QFrame::Panel);
    dataLayout = new QHBoxLayout();
    dataLayout->addSpacing(30);
    dataLayout->addWidget(m_gyroX);
    dataLayout->addWidget(m_gyroY);
    dataLayout->addWidget(m_gyroZ);
    dataLayout->addSpacing(137);
    vLayout->addLayout(dataLayout);

    vLayout->addSpacing(10);
    vLayout->addWidget(new QLabel("Accelerometers (g): "));

    m_accelX = new QLabel("0");
    m_accelX->setFrameStyle(QFrame::Panel);
    m_accelY = new QLabel("0");
    m_accelY->setFrameStyle(QFrame::Panel);
    m_accelZ = new QLabel("0");
    m_accelZ->setFrameStyle(QFrame::Panel);
    dataLayout = new QHBoxLayout();
    dataLayout->addSpacing(30);
    dataLayout->addWidget(m_accelX);
    dataLayout->addWidget(m_accelY);
    dataLayout->addWidget(m_accelZ);
    dataLayout->addSpacing(137);
    vLayout->addLayout(dataLayout);

    vLayout->addSpacing(10);
    vLayout->addWidget(new QLabel("Magnetometers (uT): "));

    m_compassX = new QLabel("0");
    m_compassX->setFrameStyle(QFrame::Panel);
    m_compassY = new QLabel("0");
    m_compassY->setFrameStyle(QFrame::Panel);
    m_compassZ = new QLabel("0");
    m_compassZ->setFrameStyle(QFrame::Panel);
    dataLayout = new QHBoxLayout();
    dataLayout->addSpacing(30);
    dataLayout->addWidget(m_compassX);
    dataLayout->addWidget(m_compassY);
    dataLayout->addWidget(m_compassZ);
    dataLayout->addSpacing(137);
    vLayout->addLayout(dataLayout);

    vLayout->addSpacing(10);
    vLayout->addWidget(new QLabel("Kalman controls: "));

    m_enableGyro = new QCheckBox("Enable gyros");
    m_enableGyro->setChecked(true);
    vLayout->addWidget(m_enableGyro);

    m_enableAccel = new QCheckBox("Enable accels");
    m_enableAccel->setChecked(true);
    vLayout->addWidget(m_enableAccel);

    m_enableCompass = new QCheckBox("Enable compass");
    m_enableCompass->setChecked(true);
    vLayout->addWidget(m_enableCompass);

    m_enableDebug = new QCheckBox("Enable debug messages");
    m_enableDebug->setChecked(false);
    vLayout->addWidget(m_enableDebug);

    vLayout->addStretch(1);

    centralWidget()->setLayout(vLayout);
    setFixedSize(500, 450);

}


void RTIMULibDemo::layoutStatusBar()
{
    m_rateStatus = new QLabel(this);
    m_rateStatus->setAlignment(Qt::AlignLeft);
    ui.statusBar->addWidget(m_rateStatus, 1);

    m_calStatus = new QLabel(this);
    m_calStatus->setAlignment(Qt::AlignLeft);
    ui.statusBar->addWidget(m_calStatus, 0);
}
