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

#include "SelectFusionDlg.h"
#include "RTIMUSettings.h"
#include "RTFusion.h"

#include <QFormLayout>
#include <QLabel>

SelectFusionDlg::SelectFusionDlg(RTIMUSettings *settings, QWidget *parent)
	: QDialog(parent, Qt::WindowCloseButtonHint | Qt::WindowTitleHint)
{
    m_settings = settings;
	layoutWindow();
    setWindowTitle("Select Fusion algorithm");
	connect(m_buttons, SIGNAL(accepted()), this, SLOT(onOk()));
    connect(m_buttons, SIGNAL(rejected()), this, SLOT(onCancel()));
 }

SelectFusionDlg::~SelectFusionDlg()
{
}

void SelectFusionDlg::onOk()
{
    m_settings->m_fusionType = m_selectFusion->currentIndex();
    m_settings->saveSettings();

    accept();
}

void SelectFusionDlg::onCancel()
{
    reject();
}

void SelectFusionDlg::layoutWindow()
{
    QVBoxLayout *mainLayout;
    QFormLayout *form;

    setModal(true);

    mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(20);
    mainLayout->setContentsMargins(11, 11, 11, 11);

    form = new QFormLayout();
    mainLayout->addLayout(form);
	
    m_selectFusion = new QComboBox();

    for (int i = 0; i < RTFUSION_TYPE_COUNT; i++)
        m_selectFusion->addItem(RTFusion::fusionName(i));

    m_selectFusion->setCurrentIndex(m_settings->m_fusionType);

    form->addRow("Select Fusion algorithm type: ", m_selectFusion);

    m_buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal, this);
	m_buttons->setCenterButtons(true);

    mainLayout->addWidget(m_buttons);
}


