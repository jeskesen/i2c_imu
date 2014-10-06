//
//  Copyright (c) 2014 richards-tech.
//
//  This file is part of SyntroNet
//
//  SyntroNet is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  SyntroNet is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with SyntroNet.  If not, see <http://www.gnu.org/licenses/>.
//

#ifndef SELECTIMUDLG_H
#define SELECTIMUDLG_H

#include <QDialog>
#include <QDialogButtonBox>
#include <QComboBox>

class RTIMUSettings;

class SelectIMUDlg : public QDialog
{
	Q_OBJECT

public:
    SelectIMUDlg(RTIMUSettings *settings, QWidget *parent = 0);
    ~SelectIMUDlg();

public slots:
	void onOk();
    void onCancel();
    void setSelectAddress(int imuType);

private:
	void layoutWindow();
    void setSelectAddress(int imuType, int slaveAddress);

    RTIMUSettings *m_settings;

	QDialogButtonBox *m_buttons;
    QComboBox *m_selectIMU;
    QComboBox *m_selectAddress;

};

#endif // SELECTIMUDLG_H
