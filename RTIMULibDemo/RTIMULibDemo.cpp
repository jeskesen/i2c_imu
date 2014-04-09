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
#include "SelectIMUDlg.h"
#include "IMUThread.h"

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
    connect(ui.actionSelectIMU, SIGNAL(triggered()), this, SLOT(onSelectIMU()));
    connect(m_enableGyro, SIGNAL(stateChanged(int)), this, SLOT(onEnableGyro(int)));
    connect(m_enableAccel, SIGNAL(stateChanged(int)), this, SLOT(onEnableAccel(int)));
    connect(m_enableCompass, SIGNAL(stateChanged(int)), this, SLOT(onEnableCompass(int)));
    connect(m_enableDebug, SIGNAL(stateChanged(int)), this, SLOT(onEnableDebug(int)));

    //  create the imu thread and connect up the signal

    m_imuThread = new IMUThread();

    connect(m_imuThread, SIGNAL(newIMUData(const RTIMU_DATA&)),
            this, SLOT(newIMUData(const RTIMU_DATA&)), Qt::DirectConnection);

    connect(this, SIGNAL(newIMU()), m_imuThread, SLOT(newIMU()));

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

void RTIMULibDemo::onSelectIMU()
{
    SelectIMUDlg dlg(m_imuThread->getSettings(), this);

    if (dlg.exec() == QDialog::Accepted) {
        emit newIMU();
    }
}


void RTIMULibDemo::onCalibrateCompass()
{
    CompassCalDlg dlg(this);

    if (m_imuThread->getIMU() == NULL)
        return;
    m_imuThread->setCalibrationMode(true);
    connect(m_imuThread, SIGNAL(newCalData(const RTVector3&)), &dlg,
            SLOT(newCalData(const RTVector3&)), Qt::DirectConnection);

    if (dlg.exec() == QDialog::Accepted) {
        m_imuThread->newCompassCalData(dlg.getCompassMin(), dlg.getCompassMax());
        m_imuThread->setCalibrationMode(false);
    }
    disconnect(m_imuThread, SIGNAL(newCalData(const RTVector3&)), &dlg, SLOT(newCalData(const RTVector3&)));
}

void RTIMULibDemo::newIMUData(const RTIMU_DATA& data)
{
    m_imuData = data;
    m_sampleCount++;
}

void RTIMULibDemo::onEnableGyro(int state)
{
    if (m_imuThread->getIMU() != NULL)
        m_imuThread->getIMU()->setGyroEnable(state == Qt::Checked);
}

void RTIMULibDemo::onEnableAccel(int state)
{
    if (m_imuThread->getIMU() != NULL)
        m_imuThread->getIMU()->setAccelEnable(state == Qt::Checked);
}

void RTIMULibDemo::onEnableCompass(int state)
{
    if (m_imuThread->getIMU() != NULL)
        m_imuThread->getIMU()->setCompassEnable(state == Qt::Checked);
}

void RTIMULibDemo::onEnableDebug(int state)
{
    if (m_imuThread->getIMU() != NULL)
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

        m_gyroX->setText(QString::number(m_imuData.gyro.x(), 'g', 6));
        m_gyroY->setText(QString::number(m_imuData.gyro.y(), 'g', 6));
        m_gyroZ->setText(QString::number(m_imuData.gyro.z(), 'g', 6));

        m_accelX->setText(QString::number(m_imuData.accel.x(), 'g', 6));
        m_accelY->setText(QString::number(m_imuData.accel.y(), 'g', 6));
        m_accelZ->setText(QString::number(m_imuData.accel.z(), 'g', 6));

        m_compassX->setText(QString::number(m_imuData.compass.x(), 'g', 6));
        m_compassY->setText(QString::number(m_imuData.compass.y(), 'g', 6));
        m_compassZ->setText(QString::number(m_imuData.compass.z(), 'g', 6));

        m_fusionPoseX->setText(QString::number(m_imuData.fusionPose.x() * RTMATH_RAD_TO_DEGREE, 'g', 6));
        m_fusionPoseY->setText(QString::number(m_imuData.fusionPose.y() * RTMATH_RAD_TO_DEGREE, 'g', 6));
        m_fusionPoseZ->setText(QString::number(m_imuData.fusionPose.z() * RTMATH_RAD_TO_DEGREE, 'g', 6));

        m_fusionQPoseScalar->setText(QString::number(m_imuData.fusionQPose.scalar(), 'g', 6));
        m_fusionQPoseX->setText(QString::number(m_imuData.fusionQPose.x(), 'g', 6));
        m_fusionQPoseY->setText(QString::number(m_imuData.fusionQPose.y(), 'g', 6));
        m_fusionQPoseZ->setText(QString::number(m_imuData.fusionQPose.z(), 'g', 6));
    } else {

        //  Update the sample rate

        float rate = (float)m_sampleCount / (float(RATE_TIMER_INTERVAL));
        m_sampleCount = 0;
        m_rateStatus->setText(QString("Sample rate: %1 per second").arg(rate));

        if (m_imuThread->getIMU() == NULL) {
            m_calStatus->setText("No IMU found");
        } else {
            if (m_imuThread->getIMU()->getCalibrationValid())
                m_calStatus->setText("Calibration in use");
            else
                m_calStatus->setText("Uncalibrated");
        }

        if (m_imuThread->getIMU() != NULL) {
            m_imuType->setText(m_imuThread->getIMU()->IMUName());

            if (!m_imuThread->getIMU()->IMUGyroBiasValid())
                m_biasStatus->setText("Gyro bias being calculated - keep IMU still!");
            else
                m_biasStatus->setText("Gyro bias valid");
        }
    }
}

void RTIMULibDemo::layoutWindow()
{
    QVBoxLayout *vLayout = new QVBoxLayout();
    vLayout->setContentsMargins(3, 3, 3, 3);
    vLayout->setSpacing(3);

    QHBoxLayout *imuLayout = new QHBoxLayout();
    vLayout->addLayout(imuLayout);
    imuLayout->addWidget(new QLabel("IMU type: "));
    m_imuType = new QLabel();
    imuLayout->addWidget(m_imuType);
    imuLayout->setStretch(1, 1);

    vLayout->addSpacing(10);

    QHBoxLayout *biasLayout = new QHBoxLayout();
    vLayout->addLayout(biasLayout);
    biasLayout->addWidget(new QLabel("Gyro bias status: "));
    m_biasStatus = new QLabel();
    biasLayout->addWidget(m_biasStatus);
    biasLayout->setStretch(1, 1);

    vLayout->addSpacing(10);

    vLayout->addWidget(new QLabel("Fusion state (quaternion): "));

    QHBoxLayout *dataLayout = new QHBoxLayout();
    dataLayout->addSpacing(30);
    m_fusionQPoseScalar = new QLabel("1");
    m_fusionQPoseScalar->setFrameStyle(QFrame::Panel);
    m_fusionQPoseX = new QLabel("0");
    m_fusionQPoseX->setFrameStyle(QFrame::Panel);
    m_fusionQPoseY = new QLabel("0");
    m_fusionQPoseY->setFrameStyle(QFrame::Panel);
    m_fusionQPoseZ = new QLabel("0");
    m_fusionQPoseZ->setFrameStyle(QFrame::Panel);
    dataLayout->addWidget(m_fusionQPoseScalar);
    dataLayout->addWidget(m_fusionQPoseX);
    dataLayout->addWidget(m_fusionQPoseY);
    dataLayout->addWidget(m_fusionQPoseZ);
    dataLayout->addSpacing(30);
    vLayout->addLayout(dataLayout);

    vLayout->addSpacing(10);
    vLayout->addWidget(new QLabel("Pose - roll, pitch, yaw (degrees): "));

    m_fusionPoseX = new QLabel("0");
    m_fusionPoseX->setFrameStyle(QFrame::Panel);
    m_fusionPoseY = new QLabel("0");
    m_fusionPoseY->setFrameStyle(QFrame::Panel);
    m_fusionPoseZ = new QLabel("0");
    m_fusionPoseZ->setFrameStyle(QFrame::Panel);
    dataLayout = new QHBoxLayout();
    dataLayout->addSpacing(30);
    dataLayout->addWidget(m_fusionPoseX);
    dataLayout->addWidget(m_fusionPoseY);
    dataLayout->addWidget(m_fusionPoseZ);
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
    vLayout->addWidget(new QLabel("Fusion controls: "));

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
    setFixedSize(500, 460);

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
