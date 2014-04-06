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

#include "RTIMUNull.h"
#include "RTIMUSettings.h"

RTIMUNull::RTIMUNull(RTIMUSettings *settings) : RTIMU(settings)
{
}

RTIMUNull::~RTIMUNull()
{
}

bool RTIMUNull::IMUInit()
{
    return true;
}

int RTIMUNull::IMUGetPollInterval()
{
    return (100);                                           // just a dummy value really
}

bool RTIMUNull::IMURead()
{
    updateFusion();
    return true;
}

void RTIMUNull::setIMUData(const RTIMU_DATA& data)
{
    m_imuData = data;
}
