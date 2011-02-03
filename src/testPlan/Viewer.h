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

#ifndef _VIEWER_H_
#define _VIEWER_H_

#include <QWidget>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/VRMLnodes/SoVRMLColor.h>
#include <Inventor/VRMLnodes/SoVRMLCoordinate.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedLineSet.h>
#include <Inventor/VRMLnodes/SoVRMLPointSet.h>
#include <Inventor/VRMLnodes/SoVRMLSwitch.h>
#include <Inventor/VRMLnodes/SoVRMLTransform.h>
#include <rl/plan/Model.h>
#include <rl/plan/VectorList.h>
#include <rl/plan/Viewer.h>

class Viewer : public QWidget, public rl::plan::Viewer
{
	Q_OBJECT
	
public:
	Viewer(QWidget* parent = NULL, Qt::WindowFlags f = 0);
	
	virtual ~Viewer();
	
	rl::math::Real delta;
	
	rl::plan::Model* model;
	
	SoVRMLTransform* root;
	
	SoQtExaminerViewer* viewer;
	
public slots:
	void drawConfiguration(const rl::math::Vector& q);
	
	void drawConfigurationEdge(const rl::math::Vector& u, const rl::math::Vector& v, const bool& free = true);
	
	void drawConfigurationPath(const rl::plan::VectorList& path);
	
	void drawConfigurationVertex(const rl::math::Vector& q, const bool& free = true);
	
	void drawLine(const rl::math::Vector& xyz0, const rl::math::Vector& xyz1);
	
	void drawSphere(const rl::math::Vector& center, const rl::math::Real& radius);
	
	void drawSweptVolume(const rl::plan::VectorList& path);
	
	void drawWork(const rl::math::Transform& t);
	
	void drawWorkEdge(const rl::math::Vector& u, const rl::math::Vector& v);
	
	void drawWorkPath(const rl::plan::VectorList& path);
	
	void drawWorkVertex(const rl::math::Vector& q);
	
	void reset();
	
	void resetEdges();
	
	void resetLines();
	
	void resetVertices();
	
	void save(const QString& filename);
	
	void toggleConfigurationEdges(const bool& doOn);
	
	void toggleConfigurationVertices(const bool& doOn);
	
protected:
	
private:
	SoVRMLSwitch* edges;
	
	SoVRMLColor* edgesColor;
	
	SoVRMLCoordinate* edgesCoordinate;
	
	SoVRMLIndexedLineSet* edgesIndexedLineSet;
	
	SoVRMLCoordinate* linesCoordinate;
	
	SoVRMLIndexedLineSet* linesIndexedLineSet;
	
	SoVRMLSwitch* path;
	
	SoVRMLCoordinate* pathCoordinate;
	
	SoDrawStyle* pathDrawStyle;
	
	SoVRMLIndexedLineSet* pathIndexedLineSet;
	
	SoVRMLSwitch* path3;
	
	SoVRMLCoordinate* path3Coordinate;
	
	SoDrawStyle* path3DrawStyle;
	
	SoVRMLIndexedLineSet* path3IndexedLineSet;
	
	SoVRMLSwitch* spheres;
	
	SoVRMLSwitch* swept;
	
	SoVRMLSwitch* vertices;
	
	SoVRMLColor* verticesColor;
	
	SoVRMLCoordinate* verticesCoordinate;
	
	SoDrawStyle* verticesDrawStyle;
	
	SoVRMLPointSet* verticesPointSet;
	
	SoVRMLTransform* work;
};

#endif // _VIEWER_H_
