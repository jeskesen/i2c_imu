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

