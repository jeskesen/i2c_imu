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

#ifndef _IMUTHREAD_H
#define	_IMUTHREAD_H

#include <QThread>

#include "RTMath.h"

class RTIMU;
class RTIMUSettings;

class IMUThread : public QObject
{
    Q_OBJECT

public:
    IMUThread();
    virtual ~IMUThread();

    //  resumeThread() is called when init is complete

    void resumeThread();

    //  exitThread is called to terminate and delete the thread

    void exitThread() { emit internalEndThread(); }

    RTIMUSettings *getSettings() { return m_settings; }
    void setCalibrationMode(bool enable);
    void newCompassCalData(const RTVector3& compassCalMin, const RTVector3& compassCalMax);

    RTIMU *getIMU() { return m_imu; }

public slots:
    void internalRunLoop() { initThread(); emit running();}
    void cleanup() {finishThread(); emit internalKillThread(); }

signals:
    void running();											// emitted when everything set up and thread active
    void internalEndThread();								// this to end thread
    void internalKillThread();								// tells the QThread to quit

    void newCalData(const RTVector3& compass);              // this is uncalibrated compass data emitted in cal mode
    void newIMUData(const RTVector3& kalmanPose, const RTQuaternion& kalmanQPose, const RTVector3& gyro,   // this is the normal data
                    const RTVector3& accel, const RTVector3& compass);

protected:
    void initThread();
    void finishThread();
    void timerEvent(QTimerEvent *event);

private:
    int m_timer;
    RTIMUSettings *m_settings;

    RTIMU *m_imu;
    bool m_calibrationMode;

    QThread *m_thread;
};

#endif // _IMUTHREAD_H
