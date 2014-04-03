#//
#//  Copyright (c) 2014 richards-tech
#//
#//  This file is part of RTIMULib
#//
#//  RTIMULib is free software: you can redistribute it and/or modify
#//  it under the terms of the GNU General Public License as published by
#//  the Free Software Foundation, either version 3 of the License, or
#//  (at your option) any later version.
#//
#//  RTIMULib is distributed in the hope that it will be useful,
#//  but WITHOUT ANY WARRANTY; without even the implied warranty of
#//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#//  GNU General Public License for more details.
#//
#//  You should have received a copy of the GNU General Public License
#//  along with RTIMULib.  If not, see <http://www.gnu.org/licenses/>.
#//

INCLUDEPATH += $$PWD
DEPENDPATH += $$PWD

HEADERS += $$PWD/RTMath.h \
    $$PWD/RTIMUHal.h \
    $$PWD/RTIMU.h \
    $$PWD/RTIMUMPU9150.h \
    $$PWD/RTIMUNull.h \
    $$PWD/RTKalman.h \
    $$PWD/RTKalman4.h \
    $$PWD/RTIMUSettings.h
 
SOURCES += $$PWD/RTMath.cpp \
    $$PWD/RTIMUHal.cpp \
    $$PWD/RTIMU.cpp \
    $$PWD/RTIMUMPU9150.cpp \
    $$PWD/RTIMUNull.cpp \
    $$PWD/RTKalman.cpp \
    $$PWD/RTKalman4.cpp \
    $$PWD/RTIMUSettings.cpp

