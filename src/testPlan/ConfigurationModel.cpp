//
// Copyright (c) 2009, Markus Rickert
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Technische Universitaet Muenchen nor the names of
//   its contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <rl/math/Unit.h>

#include "ConfigurationModel.h"
#include "MainWindow.h"
#include "Thread.h"
#include "Viewer.h"

ConfigurationModel::ConfigurationModel(QObject* parent) :
	QAbstractTableModel(parent)
{
}

ConfigurationModel::~ConfigurationModel()
{
}

int
ConfigurationModel::columnCount(const QModelIndex& parent) const
{
	return 1;
}

QVariant
ConfigurationModel::data(const QModelIndex& index, int role) const
{
	if (NULL == MainWindow::instance()->model)
	{
		return QVariant();
	}
	
	if (!index.isValid())
	{
		return QVariant();
	}
	
	switch (role)
	{
	case Qt::DisplayRole:
		switch (MainWindow::instance()->model->kinematics->getType(index.row()))
		{
		case rl::kin::Kinematics::TYPE_PRISMATIC:
			return (*MainWindow::instance()->q)(index.row());
			break;
		case rl::kin::Kinematics::TYPE_REVOLUTE:
			return (*MainWindow::instance()->q)(index.row()) * rl::math::RAD2DEG;
			break;
		default:
			break;
		}
	case Qt::TextAlignmentRole:
		return Qt::AlignRight;
		break;
	default:
		break;
	}
	
	return QVariant();
}

Qt::ItemFlags
ConfigurationModel::flags(const QModelIndex &index) const
{
	if (!index.isValid())
	{
		return Qt::ItemIsEnabled;
	}
	
	return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
}

QVariant
ConfigurationModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (Qt::DisplayRole == role && Qt::Vertical == orientation)
	{
		return QString::number(section);
	}
	
	return QVariant();
}

void
ConfigurationModel::invalidate()
{
	this->reset();
}

int
ConfigurationModel::rowCount(const QModelIndex& parent) const
{
	if (NULL == MainWindow::instance()->model)
	{
		return 0;
	}
	
	return MainWindow::instance()->model->getDof();
}

bool
ConfigurationModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
	if (NULL == MainWindow::instance()->model)
	{
		return false;
	}
	
	if (MainWindow::instance()->thread->isRunning())
	{
		return false;
	}
	
	if (index.isValid() && Qt::EditRole == role)
	{
		switch (MainWindow::instance()->model->kinematics->getType(index.row()))
		{
		case rl::kin::Kinematics::TYPE_PRISMATIC:
			(*MainWindow::instance()->q)(index.row()) = value.toDouble();
			break;
		case rl::kin::Kinematics::TYPE_REVOLUTE:
			(*MainWindow::instance()->q)(index.row()) = value.toDouble() * rl::math::DEG2RAD;
			break;
		default:
			break;
		}
		
		MainWindow::instance()->viewer->drawConfiguration(*MainWindow::instance()->q);
		
		emit dataChanged(index, index);
		
		return true;
	}
	
	return false;
}
