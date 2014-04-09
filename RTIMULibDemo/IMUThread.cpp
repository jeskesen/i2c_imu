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

#include "IMUThread.h"
#include <QDebug>

IMUThread::IMUThread() : QObject()
{
    m_imu = NULL;
    m_calibrationMode = false;
    m_settings = new RTIMUSettings();                       // just use default name (RTIMULib.ini) for settings file
    m_timer = -1;
}

IMUThread::~IMUThread()
{

}

void IMUThread::newIMU()
{
    if (m_imu != NULL) {
        delete m_imu;
        m_imu = NULL;
    }

    if (m_timer != -1) {
        killTimer(m_timer);
        m_timer = -1;
    }

    m_imu = RTIMU::createIMU(m_settings);

    if (m_imu == NULL)
        return;

    //  set up IMU

    m_imu->IMUInit();

    m_timer = startTimer(m_imu->IMUGetPollInterval());
}



void IMUThread::setCalibrationMode(bool enable)
{
    m_imu->setCalibrationMode(enable);
    m_calibrationMode = enable;
}

void IMUThread::newCompassCalData(const RTVector3& compassCalMin, const RTVector3& compassCalMax)
{
    m_settings->m_compassCalValid = true;
    m_settings->m_compassCalMin = compassCalMin;
    m_settings->m_compassCalMax = compassCalMax;
    m_settings->saveSettings();
    m_imu->setCalibrationData(true, compassCalMin, compassCalMax);
}


void IMUThread::initThread()
{
    //  create IMU. There's a special function call for this
    //  as it makes sure that the required one is created as specified in the settings.

    m_imu = RTIMU::createIMU(m_settings);

    if (m_imu == NULL) {
        qDebug() << "No IMU found.";
        return;
    }

    //  set up IMU

    m_imu->IMUInit();

    //  poll at the rate suggested bu the IMU

    m_timer = startTimer(m_imu->IMUGetPollInterval());

    //  up the priority in case it's helpful

    m_thread->setPriority(QThread::TimeCriticalPriority);
}

void IMUThread::finishThread()
{
    if (m_timer != -1)
        killTimer(m_timer);

    m_timer = -1;

    if (m_imu != NULL)
        delete m_imu;

    m_imu = NULL;

    delete m_settings;
}

void IMUThread::timerEvent(QTimerEvent * /* event */)
{
    //  check for valid IMU

    if (m_imu == NULL)
        return;

    if (m_imu->IMUType() == RTIMU_TYPE_NULL)
        return;

    //  loop here to clear all samples just in case things aren't keeping up

    while (m_imu->IMURead()) {
        if (m_calibrationMode) {
            emit newCalData(m_imu->getCompass());
        } else {
            emit newIMUData(m_imu->getIMUData());
        }
    }
}

//----------------------------------------------------------
//
//  The following is some Qt threading stuff

void IMUThread::resumeThread()
{
    m_thread = new QThread();
    moveToThread(m_thread);
    connect(m_thread, SIGNAL(started()), this, SLOT(internalRunLoop()));
    connect(this, SIGNAL(internalEndThread()), this, SLOT(cleanup()));
    connect(this, SIGNAL(internalKillThread()), m_thread, SLOT(quit()));
    connect(m_thread, SIGNAL(finished()), m_thread, SLOT(deleteLater()));
    connect(m_thread, SIGNAL(finished()), this, SLOT(deleteLater()));
    m_thread->start();
}

