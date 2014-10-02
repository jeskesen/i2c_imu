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

#include "RTMath.h"

//  Strings are put here. So the display functions are no re-entrant!

char RTMath::m_string[1000];

uint64_t RTMath::currentUSecsSinceEpoch()
{
    struct timeval tv;

    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000 + (uint64_t)tv.tv_usec;
}

const char *RTMath::displayRadians(const char *label, RTVector3& vec)
{
    sprintf(m_string, "%s: x:%f, y:%f, z:%f\n", label, vec.x(), vec.y(), vec.z());
    return m_string;
}

const char *RTMath::displayDegrees(const char *label, RTVector3& vec)
{
    sprintf(m_string, "%s: roll:%f, pitch:%f, yaw:%f", label, vec.x() * RTMATH_RAD_TO_DEGREE,
            vec.y() * RTMATH_RAD_TO_DEGREE, vec.z() * RTMATH_RAD_TO_DEGREE);
    return m_string;
}

const char *RTMath::display(const char *label, RTQuaternion& quat)
{
    sprintf(m_string, "%s: scalar: %f, x:%f, y:%f, z:%f\n", label, quat.scalar(), quat.x(), quat.y(), quat.z());
    return m_string;
}

const char *RTMath::display(const char *label, RTMatrix4x4& mat)
{
    sprintf(m_string, "%s(0): %f %f %f %f\n%s(1): %f %f %f %f\n%s(2): %f %f %f %f\n%s(3): %f %f %f %f\n",
            label, mat.val(0,0), mat.val(0,1), mat.val(0,2), mat.val(0,3),
            label, mat.val(1,0), mat.val(1,1), mat.val(1,2), mat.val(1,3),
            label, mat.val(2,0), mat.val(2,1), mat.val(2,2), mat.val(2,3),
            label, mat.val(3,0), mat.val(3,1), mat.val(3,2), mat.val(3,3));
    return m_string;
}

RTVector3 RTMath::poseFromAccelMag(const RTVector3& accel, const RTVector3& mag)
{
    RTVector3 result;
    RTQuaternion m;
    RTQuaternion q;

    accel.accelToEuler(result);

//  q.fromEuler(result);
//  since result.z() is always 0, this can be optimized a little

    RTFLOAT cosX2 = cos(result.x() / 2.0f);
    RTFLOAT sinX2 = sin(result.x() / 2.0f);
    RTFLOAT cosY2 = cos(result.y() / 2.0f);
    RTFLOAT sinY2 = sin(result.y() / 2.0f);

    q.setScalar(cosX2 * cosY2);
    q.setX(sinX2 * cosY2);
    q.setY(cosX2 * sinY2);
    q.setZ(-sinX2 * sinY2);
//    q.normalize();

    m.setScalar(0);
    m.setX(mag.x());
    m.setY(mag.y());
    m.setZ(mag.z());

    m = q * m * q.conjugate();
    result.setZ(-atan2(m.y(), m.x()));
    return result;
}

void RTMath::convertToVector(unsigned char *rawData, RTVector3& vec, RTFLOAT scale, bool bigEndian)
{
    if (bigEndian) {
        vec.setX((RTFLOAT)((int16_t)(((uint16_t)rawData[0] << 8) | (uint16_t)rawData[1])) * scale);
        vec.setY((RTFLOAT)((int16_t)(((uint16_t)rawData[2] << 8) | (uint16_t)rawData[3])) * scale);
        vec.setZ((RTFLOAT)((int16_t)(((uint16_t)rawData[4] << 8) | (uint16_t)rawData[5])) * scale);
    } else {
        vec.setX((RTFLOAT)((int16_t)(((uint16_t)rawData[1] << 8) | (uint16_t)rawData[0])) * scale);
        vec.setY((RTFLOAT)((int16_t)(((uint16_t)rawData[3] << 8) | (uint16_t)rawData[2])) * scale);
        vec.setZ((RTFLOAT)((int16_t)(((uint16_t)rawData[5] << 8) | (uint16_t)rawData[4])) * scale);
     }
}



//----------------------------------------------------------
//
//  The RTVector3 class

RTVector3::RTVector3()
{
    zero();
}

RTVector3::RTVector3(RTFLOAT x, RTFLOAT y, RTFLOAT z)
{
    m_data[0] = x;
    m_data[1] = y;
    m_data[2] = z;
}

RTVector3& RTVector3::operator =(const RTVector3& vec)
{
    if (this == &vec)
        return *this;

    m_data[0] = vec.m_data[0];
    m_data[1] = vec.m_data[1];
    m_data[2] = vec.m_data[2];

    return *this;
}


const RTVector3& RTVector3::operator +=(RTVector3& vec)
{
    for (int i = 0; i < 3; i++)
        m_data[i] += vec.m_data[i];
    return *this;
}

const RTVector3& RTVector3::operator -=(RTVector3& vec)
{
    for (int i = 0; i < 3; i++)
        m_data[i] -= vec.m_data[i];
    return *this;
}

void RTVector3::zero()
{
    for (int i = 0; i < 3; i++)
        m_data[i] = 0;
}


RTFLOAT RTVector3::dotProduct(const RTVector3& a, const RTVector3& b)
{
    return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
}

void RTVector3::crossProduct(const RTVector3& a, const RTVector3& b, RTVector3& d)
{
    d.setX(a.y() * b.z() - a.z() * b.y());
    d.setY(a.z() * b.x() - a.x() * b.z());
    d.setZ(a.x() * b.y() - a.y() * b.x());
}


void RTVector3::accelToEuler(RTVector3& rollPitchYaw) const
{
    RTVector3 normAccel = *this;

    normAccel.normalize();

    rollPitchYaw.setX(atan2(normAccel.y(), normAccel.z()));
    rollPitchYaw.setY(-atan2(normAccel.x(), sqrt(normAccel.y() * normAccel.y() + normAccel.z() * normAccel.z())));
    rollPitchYaw.setZ(0);
}


void RTVector3::accelToQuaternion(RTQuaternion& qPose) const
{
    RTVector3 normAccel = *this;
    RTVector3 vec;
    RTVector3 z(0, 0, 1.0);

    normAccel.normalize();

    RTFLOAT angle = acos(RTVector3::dotProduct(z, normAccel));
    RTVector3::crossProduct(normAccel, z, vec);
    vec.normalize();

    qPose.fromAngleVector(angle, vec);
}


void RTVector3::normalize()
{
    RTFLOAT length = sqrt(m_data[0] * m_data[0] + m_data[1] * m_data[1] +
            m_data[2] * m_data[2]);

    if (length == 0)
        return;

    m_data[0] /= length;
    m_data[1] /= length;
    m_data[2] /= length;
}

RTFLOAT RTVector3::length()
{
    return sqrt(m_data[0] * m_data[0] + m_data[1] * m_data[1] +
            m_data[2] * m_data[2]);
}

//----------------------------------------------------------
//
//  The RTQuaternion class

RTQuaternion::RTQuaternion()
{
    zero();
}

RTQuaternion::RTQuaternion(RTFLOAT scalar, RTFLOAT x, RTFLOAT y, RTFLOAT z)
{
    m_data[0] = scalar;
    m_data[1] = x;
    m_data[2] = y;
    m_data[3] = z;
}

RTQuaternion& RTQuaternion::operator =(const RTQuaternion& quat)
{
    if (this == &quat)
        return *this;

    m_data[0] = quat.m_data[0];
    m_data[1] = quat.m_data[1];
    m_data[2] = quat.m_data[2];
    m_data[3] = quat.m_data[3];

    return *this;
}



RTQuaternion& RTQuaternion::operator +=(const RTQuaternion& quat)
{
    for (int i = 0; i < 4; i++)
        m_data[i] += quat.m_data[i];
    return *this;
}

RTQuaternion& RTQuaternion::operator -=(const RTQuaternion& quat)
{
    for (int i = 0; i < 4; i++)
        m_data[i] -= quat.m_data[i];
    return *this;
}

RTQuaternion& RTQuaternion::operator -=(const RTFLOAT val)
{
    for (int i = 0; i < 4; i++)
        m_data[i] -= val;
    return *this;
}

RTQuaternion& RTQuaternion::operator *=(const RTQuaternion& qb)
{
    RTVector3 va;
    RTVector3 vb;
    RTFLOAT dotAB;
    RTVector3 crossAB;

    va.setX(m_data[1]);
    va.setY(m_data[2]);
    va.setZ(m_data[3]);

    vb.setX(qb.x());
    vb.setY(qb.y());
    vb.setZ(qb.z());

    dotAB = RTVector3::dotProduct(va, vb);
    RTVector3::crossProduct(va, vb, crossAB);
    RTFLOAT myScalar = m_data[0];

    m_data[0] = myScalar * qb.scalar() - dotAB;
    m_data[1] = myScalar * vb.x() + qb.scalar() * va.x() + crossAB.x();
    m_data[2] = myScalar * vb.y() + qb.scalar() * va.y() + crossAB.y();
    m_data[3] = myScalar * vb.z() + qb.scalar() * va.z() + crossAB.z();

    return *this;
}


RTQuaternion& RTQuaternion::operator *=(const RTFLOAT val)
{
    m_data[0] *= val;
    m_data[1] *= val;
    m_data[2] *= val;
    m_data[3] *= val;

    return *this;
}


const RTQuaternion RTQuaternion::operator *(const RTQuaternion& qb) const
{
    RTQuaternion result = *this;
    result *= qb;
    return result;
}

const RTQuaternion RTQuaternion::operator *(const RTFLOAT val) const
{
    RTQuaternion result = *this;
    result *= val;
    return result;
}


const RTQuaternion RTQuaternion::operator -(const RTQuaternion& qb) const
{
    RTQuaternion result = *this;
    result -= qb;
    return result;
}

const RTQuaternion RTQuaternion::operator -(const RTFLOAT val) const
{
    RTQuaternion result = *this;
    result -= val;
    return result;
}


void RTQuaternion::zero()
{
    for (int i = 0; i < 4; i++)
        m_data[i] = 0;
}

void RTQuaternion::normalize()
{
    RTFLOAT length = sqrt(m_data[0] * m_data[0] + m_data[1] * m_data[1] +
            m_data[2] * m_data[2] + m_data[3] * m_data[3]);

    if ((length == 0) || (length == 1))
        return;

    m_data[0] /= length;
    m_data[1] /= length;
    m_data[2] /= length;
    m_data[3] /= length;
}

void RTQuaternion::toEuler(RTVector3& vec)
{
    vec.setX(atan2(2.0 * (m_data[2] * m_data[3] + m_data[0] * m_data[1]),
            1 - 2.0 * (m_data[1] * m_data[1] + m_data[2] * m_data[2])));

    vec.setY(asin(2.0 * (m_data[0] * m_data[2] - m_data[1] * m_data[3])));

    vec.setZ(atan2(2.0 * (m_data[1] * m_data[2] + m_data[0] * m_data[3]),
            1 - 2.0 * (m_data[2] * m_data[2] + m_data[3] * m_data[3])));
}

void RTQuaternion::fromEuler(RTVector3& vec)
{
    RTFLOAT cosX2 = cos(vec.x() / 2.0f);
    RTFLOAT sinX2 = sin(vec.x() / 2.0f);
    RTFLOAT cosY2 = cos(vec.y() / 2.0f);
    RTFLOAT sinY2 = sin(vec.y() / 2.0f);
    RTFLOAT cosZ2 = cos(vec.z() / 2.0f);
    RTFLOAT sinZ2 = sin(vec.z() / 2.0f);

    m_data[0] = cosX2 * cosY2 * cosZ2 + sinX2 * sinY2 * sinZ2;
    m_data[1] = sinX2 * cosY2 * cosZ2 - cosX2 * sinY2 * sinZ2;
    m_data[2] = cosX2 * sinY2 * cosZ2 + sinX2 * cosY2 * sinZ2;
    m_data[3] = cosX2 * cosY2 * sinZ2 - sinX2 * sinY2 * cosZ2;
    normalize();
}

RTQuaternion RTQuaternion::conjugate() const
{
    RTQuaternion q;
    q.setScalar(m_data[0]);
    q.setX(-m_data[1]);
    q.setY(-m_data[2]);
    q.setZ(-m_data[3]);
    return q;
}

void RTQuaternion::toAngleVector(RTFLOAT& angle, RTVector3& vec)
{
    RTFLOAT halfTheta;
    RTFLOAT sinHalfTheta;

    halfTheta = acos(m_data[0]);
    sinHalfTheta = sin(halfTheta);

    if (sinHalfTheta == 0) {
        vec.setX(1.0);
        vec.setY(0);
        vec.setZ(0);
    } else {
        vec.setX(m_data[1] / sinHalfTheta);
        vec.setY(m_data[1] / sinHalfTheta);
        vec.setZ(m_data[1] / sinHalfTheta);
    }
    angle = 2.0 * halfTheta;
}

void RTQuaternion::fromAngleVector(const RTFLOAT& angle, const RTVector3& vec)
{
    RTFLOAT sinHalfTheta = sin(angle / 2.0);
    m_data[0] = cos(angle / 2.0);
    m_data[1] = vec.x() * sinHalfTheta;
    m_data[2] = vec.y() * sinHalfTheta;
    m_data[3] = vec.z() * sinHalfTheta;
}



//----------------------------------------------------------
//
//  The RTMatrix4x4 class

RTMatrix4x4::RTMatrix4x4()
{
    fill(0);
}

RTMatrix4x4& RTMatrix4x4::operator =(const RTMatrix4x4& mat)
{
    if (this == &mat)
        return *this;

    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            m_data[row][col] = mat.m_data[row][col];

    return *this;
}


void RTMatrix4x4::fill(RTFLOAT val)
{
    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            m_data[row][col] = val;
}


RTMatrix4x4& RTMatrix4x4::operator +=(const RTMatrix4x4& mat)
{
    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            m_data[row][col] += mat.m_data[row][col];

    return *this;
}

RTMatrix4x4& RTMatrix4x4::operator -=(const RTMatrix4x4& mat)
{
    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            m_data[row][col] -= mat.m_data[row][col];

    return *this;
}

RTMatrix4x4& RTMatrix4x4::operator *=(const RTFLOAT val)
{
    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            m_data[row][col] *= val;

    return *this;
}

const RTMatrix4x4 RTMatrix4x4::operator +(const RTMatrix4x4& mat) const
{
    RTMatrix4x4 result = *this;
    result += mat;
    return result;
}

const RTMatrix4x4 RTMatrix4x4::operator *(const RTFLOAT val) const
{
    RTMatrix4x4 result = *this;
    result *= val;
    return result;
}


const RTMatrix4x4 RTMatrix4x4::operator *(const RTMatrix4x4& mat) const
{
    RTMatrix4x4 res;

    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            res.m_data[row][col] =
                    m_data[row][0] * mat.m_data[0][col] +
                    m_data[row][1] * mat.m_data[1][col] +
                    m_data[row][2] * mat.m_data[2][col] +
                    m_data[row][3] * mat.m_data[3][col];

    return res;
}


const RTQuaternion RTMatrix4x4::operator *(const RTQuaternion& q) const
{
    RTQuaternion res;

    res.setScalar(m_data[0][0] * q.scalar() + m_data[0][1] * q.x() + m_data[0][2] * q.y() + m_data[0][3] * q.z());
    res.setX(m_data[1][0] * q.scalar() + m_data[1][1] * q.x() + m_data[1][2] * q.y() + m_data[1][3] * q.z());
    res.setY(m_data[2][0] * q.scalar() + m_data[2][1] * q.x() + m_data[2][2] * q.y() + m_data[2][3] * q.z());
    res.setZ(m_data[3][0] * q.scalar() + m_data[3][1] * q.x() + m_data[3][2] * q.y() + m_data[3][3] * q.z());

    return res;
}

void RTMatrix4x4::setToIdentity()
{
    fill(0);
    m_data[0][0] = 1;
    m_data[1][1] = 1;
    m_data[2][2] = 1;
    m_data[3][3] = 1;
}

RTMatrix4x4 RTMatrix4x4::transposed()
{
    RTMatrix4x4 res;

    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            res.m_data[col][row] = m_data[row][col];
    return res;
}

//  Note:
//  The matrix inversion code here was strongly influenced by some old code I found
//  but I have no idea where it came from. Apologies to whoever wrote it originally!
//  If it's you, please let me know at info@richards-tech.com so I can credit it correctly.

RTMatrix4x4 RTMatrix4x4::inverted()
{
    RTMatrix4x4 res;

    RTFLOAT det = matDet();

    if (det == 0) {
        res.setToIdentity();
        return res;
    }

    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            if ((row + col) & 1)
                res.m_data[col][row] = -matMinor(row, col) / det;
            else
                res.m_data[col][row] = matMinor(row, col) / det;
        }
    }

    return res;
}

RTFLOAT RTMatrix4x4::matDet()
{
    RTFLOAT det = 0;

    det += m_data[0][0] * matMinor(0, 0);
    det -= m_data[0][1] * matMinor(0, 1);
    det += m_data[0][2] * matMinor(0, 2);
    det -= m_data[0][3] * matMinor(0, 3);
    return det;
}

RTFLOAT RTMatrix4x4::matMinor(const int row, const int col)
{
    static int map[] = {1, 2, 3, 0, 2, 3, 0, 1, 3, 0, 1, 2};

    int *rc;
    int *cc;
    RTFLOAT res = 0;

    rc = map + row * 3;
    cc = map + col * 3;

    res += m_data[rc[0]][cc[0]] * m_data[rc[1]][cc[1]] * m_data[rc[2]][cc[2]];
    res -= m_data[rc[0]][cc[0]] * m_data[rc[1]][cc[2]] * m_data[rc[2]][cc[1]];
    res -= m_data[rc[0]][cc[1]] * m_data[rc[1]][cc[0]] * m_data[rc[2]][cc[2]];
    res += m_data[rc[0]][cc[1]] * m_data[rc[1]][cc[2]] * m_data[rc[2]][cc[0]];
    res += m_data[rc[0]][cc[2]] * m_data[rc[1]][cc[0]] * m_data[rc[2]][cc[1]];
    res -= m_data[rc[0]][cc[2]] * m_data[rc[1]][cc[1]] * m_data[rc[2]][cc[0]];
    return res;
}

