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

#include "RTMath.h"

#define PRODUCT_TYPE "RTIMULib"

class IMUThread;

class RTIMULibDemo : public QMainWindow
{
	Q_OBJECT

public:
    RTIMULibDemo();
    ~RTIMULibDemo();

public slots:
    void onCalibrateCompass();
    void onEnableGyro(int);
    void onEnableAccel(int);
    void onEnableCompass(int);
    void onEnableDebug(int);
    void newIMUData(const RTVector3& kalmanPose, const RTQuaternion& kalmanQPose, const RTVector3& gyro,
                    const RTVector3& accel, const RTVector3& compass);

protected:
	void timerEvent(QTimerEvent *event);
	void closeEvent(QCloseEvent *event);

private:
    void layoutStatusBar();
    void layoutWindow();

    IMUThread *m_imuThread;                                 // the thread that operates the imu

    //  These are the (possibly) calibrated sensor outputs
    //  They are always in standard units no matter what the IMU actually is:
    //
    //  gyro is in radians per second
    //  accel is in g
    //  compass is in uT

    RTVector3 m_gyro;
    RTVector3 m_accel;
    RTVector3 m_compass;

    //  These are the outputs from the kalman filter as Euler angles and a quaternion

    RTVector3 m_kalmanPose;
    RTQuaternion m_kalmanQPose;

    //  Qt GUI stuff

    Ui::RTIMULibDemoClass ui;

    QLabel *m_kalmanQPoseScalar;
    QLabel *m_kalmanQPoseX;
    QLabel *m_kalmanQPoseY;
    QLabel *m_kalmanQPoseZ;

    QLabel *m_kalmanPoseX;
    QLabel *m_kalmanPoseY;
    QLabel *m_kalmanPoseZ;

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

    QLabel *m_rateStatus;
    QLabel *m_calStatus;

    int m_rateTimer;
    int m_displayTimer;


    int m_sampleCount;
};

#endif // _RTIMULIBDEMO_H

