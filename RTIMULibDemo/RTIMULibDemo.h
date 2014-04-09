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

#ifndef _RTIMULIBDEMO_H
#define _RTIMULIBDEMO_H

#include <QMainWindow>
#include <QLabel>
#include <QCheckBox>

#include "ui_RTIMULibDemo.h"

#include "RTIMULib.h"

class IMUThread;

class RTIMULibDemo : public QMainWindow
{
	Q_OBJECT

public:
    RTIMULibDemo();
    ~RTIMULibDemo();

public slots:
    void onCalibrateCompass();
    void onSelectIMU();
    void onEnableGyro(int);
    void onEnableAccel(int);
    void onEnableCompass(int);
    void onEnableDebug(int);
    void newIMUData(const RTIMU_DATA&);

signals:
    void newIMU();

protected:
	void timerEvent(QTimerEvent *event);
	void closeEvent(QCloseEvent *event);

private:
    void layoutStatusBar();
    void layoutWindow();

    IMUThread *m_imuThread;                                 // the thread that operates the imu

    RTIMU_DATA m_imuData;                                   // this holds the IMU information and funsion output

    //  Qt GUI stuff

    Ui::RTIMULibDemoClass ui;

    QLabel *m_fusionQPoseScalar;
    QLabel *m_fusionQPoseX;
    QLabel *m_fusionQPoseY;
    QLabel *m_fusionQPoseZ;

    QLabel *m_fusionPoseX;
    QLabel *m_fusionPoseY;
    QLabel *m_fusionPoseZ;

    QLabel *m_gyroX;
    QLabel *m_gyroY;
    QLabel *m_gyroZ;

    QLabel *m_accelX;
    QLabel *m_accelY;
    QLabel *m_accelZ;

    QLabel *m_compassX;
    QLabel *m_compassY;
    QLabel *m_compassZ;

    QCheckBox *m_enableGyro;
    QCheckBox *m_enableAccel;
    QCheckBox *m_enableCompass;
    QCheckBox *m_enableDebug;

    QLabel *m_imuType;
    QLabel *m_biasStatus;
    QLabel *m_rateStatus;
    QLabel *m_calStatus;

    int m_rateTimer;
    int m_displayTimer;


    int m_sampleCount;
};

#endif // _RTIMULIBDEMO_H

