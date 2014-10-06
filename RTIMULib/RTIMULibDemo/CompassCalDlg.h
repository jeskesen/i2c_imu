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

#ifndef COMPASSCALDLG_H
#define COMPASSCALDLG_H

#include <QDialog>
#include <qdialogbuttonbox.h>
#include <qmessagebox.h>

#include "RTMath.h"

class CompassCalDlg : public QDialog
{
	Q_OBJECT

public:
    CompassCalDlg(QWidget *parent = 0);
    ~CompassCalDlg();

    const RTVector3& getCompassMin() { return m_compassMin; }
    const RTVector3& getCompassMax() { return m_compassMax; }

public slots:
	void onOk();
    void onCancel();
    void newCalData(const RTVector3& compass);

protected:
    void timerEvent(QTimerEvent *event);

private:
	void layoutWindow();

	QDialogButtonBox *m_buttons;

    QLabel *m_compassMinX;
    QLabel *m_compassMinY;
    QLabel *m_compassMinZ;

    QLabel *m_compassMaxX;
    QLabel *m_compassMaxY;
    QLabel *m_compassMaxZ;

    RTVector3 m_compassMin;
    RTVector3 m_compassMax;

    int m_timer;

};

#endif // COMPASSCALDLG_H
