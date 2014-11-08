#////////////////////////////////////////////////////////////////////////////
#//
#//  This file is part of RTIMULib
#//
#//  Copyright (c) 2014, richards-tech
#//
#//  Permission is hereby granted, free of charge, to any person obtaining a copy of
#//  this software and associated documentation files (the "Software"), to deal in
#//  the Software without restriction, including without limitation the rights to use,
#//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
#//  Software, and to permit persons to whom the Software is furnished to do so,
#//  subject to the following conditions:
#//
#//  The above copyright notice and this permission notice shall be included in all
#//  copies or substantial portions of the Software.
#//
#//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
#//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
#//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
#//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

INCLUDEPATH += $$PWD
DEPENDPATH += $$PWD

HEADERS += $$PWD/RTIMULib.h \
    $$PWD/RTIMULibDefs.h \
    $$PWD/RTMath.h \
    $$PWD/RTIMUHal.h \
    $$PWD/RTIMU.h \
    $$PWD/RTIMUMPU9150.h \
    $$PWD/RTIMUMPU9250.h \
    $$PWD/RTIMUGD20HM303D.h \
    $$PWD/RTIMUGD20M303DLHC.h \
    $$PWD/RTIMULSM9DS0.h \
    $$PWD/RTIMUNull.h \
    $$PWD/RTFusion.h \
    $$PWD/RTFusionKalman4.h \
    $$PWD/RTFusionRTQF.h \
    $$PWD/RTIMUSettings.h \
    $$PWD/RTIMUMagCal.h \
    $$PWD/RTIMUAccelCal.h \
    $$PWD/RTIMUCalDefs.h \

SOURCES += $$PWD/RTMath.cpp \
    $$PWD/RTIMUHal.cpp \
    $$PWD/RTIMU.cpp \
    $$PWD/RTIMUMPU9150.cpp \
    $$PWD/RTIMUMPU9250.cpp \
    $$PWD/RTIMUGD20HM303D.cpp \
    $$PWD/RTIMUGD20M303DLHC.cpp \
    $$PWD/RTIMULSM9DS0.cpp \
    $$PWD/RTIMUNull.cpp \
    $$PWD/RTFusion.cpp \
    $$PWD/RTFusionKalman4.cpp \
    $$PWD/RTFusionRTQF.cpp \
    $$PWD/RTIMUSettings.cpp \
    $$PWD/RTIMUMagCal.cpp \
    $$PWD/RTIMUAccelCal.cpp \

