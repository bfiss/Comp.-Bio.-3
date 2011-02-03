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

#include <QGLWidget>
#include <QMessageBox>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/VRMLnodes/SoVRMLAppearance.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedFaceSet.h>
#include <Inventor/VRMLnodes/SoVRMLMaterial.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#include <Inventor/VRMLnodes/SoVRMLSphere.h>
#include <rl/math/Unit.h>
#include <rl/sg/Body.h>
#include <rl/sg/so/Body.h>
#include <rl/sg/so/Model.h>
#include <rl/sg/so/Shape.h>

#include "MainWindow.h"
#include "Viewer.h"

Viewer::Viewer(QWidget* parent, Qt::WindowFlags f) :
	QWidget(parent, f),
	delta(1.0f),
	model(NULL),
	root(new SoVRMLTransform()),
	viewer(new SoQtExaminerViewer(this, NULL, true, SoQtFullViewer::BUILD_POPUP)),
	edges(new SoVRMLSwitch()),
	edgesColor(new SoVRMLColor()),
	edgesCoordinate(new SoVRMLCoordinate()),
	edgesIndexedLineSet(new SoVRMLIndexedLineSet()),
	linesCoordinate(new SoVRMLCoordinate()),
	linesIndexedLineSet(new SoVRMLIndexedLineSet()),
	path(new SoVRMLSwitch()),
	pathCoordinate(new SoVRMLCoordinate()),
	pathDrawStyle(new SoDrawStyle()),
	pathIndexedLineSet(new SoVRMLIndexedLineSet()),
	path3(new SoVRMLSwitch()),
	path3Coordinate(new SoVRMLCoordinate()),
	path3DrawStyle(new SoDrawStyle()),
	path3IndexedLineSet(new SoVRMLIndexedLineSet()),
	spheres(new SoVRMLSwitch()),
	swept(new SoVRMLSwitch()),
	vertices(new SoVRMLSwitch()),
	verticesColor(new SoVRMLColor()),
	verticesCoordinate(new SoVRMLCoordinate()),
	verticesDrawStyle(new SoDrawStyle()),
	verticesPointSet(new SoVRMLPointSet()),
	work(new SoVRMLTransform())
{
	this->root->ref();
	
	this->viewer->setSceneGraph(this->root);
	this->viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
	
	// spheres
	
	this->spheres->whichChoice = SO_SWITCH_ALL;
	
	this->root->addChild(this->spheres);
	
	// edges
	
	this->edges->whichChoice = SO_SWITCH_ALL;
	
	SoVRMLShape* edgesShape = new SoVRMLShape();
	edgesShape->geometry = this->edgesIndexedLineSet;
	this->edgesIndexedLineSet->color = this->edgesColor;
	this->edgesIndexedLineSet->coord = this->edgesCoordinate;
	
	this->edges->addChild(edgesShape);
	
	this->root->addChild(this->edges);
	
	// swept
	
	this->swept->whichChoice = SO_SWITCH_ALL;
	
	this->root->addChild(this->swept);
	
	// work
	
	this->root->addChild(this->work);
	
	// lines
	
	SoVRMLShape* linesShape = new SoVRMLShape();
	SoVRMLAppearance* linesAppearance = new SoVRMLAppearance();
	SoVRMLMaterial* linesMaterial = new SoVRMLMaterial();
	linesMaterial->diffuseColor.setValue(1.0f, 0.0f, 0.0f);
	linesAppearance->material = linesMaterial;
	linesShape->appearance = linesAppearance;
	linesShape->geometry = this->linesIndexedLineSet;
	this->linesIndexedLineSet->coord = this->linesCoordinate;
	
	this->root->addChild(linesShape);
	
	// vertices
	
	this->vertices->whichChoice = SO_SWITCH_NONE;
	
	this->verticesDrawStyle->lineWidth = 0.0f;
	this->verticesDrawStyle->pointSize = 8.0f;
	this->vertices->addChild(this->verticesDrawStyle);
	
	SoVRMLShape* verticesShape = new SoVRMLShape();
	verticesShape->geometry = this->verticesPointSet;
	this->verticesPointSet->color = this->verticesColor;
	this->verticesPointSet->coord = this->verticesCoordinate;
	
	this->vertices->addChild(verticesShape);
	
	this->root->addChild(this->vertices);
	
	// path
	
	this->path->whichChoice = SO_SWITCH_ALL;
	
	this->pathDrawStyle->lineWidth = 4.0f;
	this->pathDrawStyle->pointSize = 0.0f;
	this->path->addChild(this->pathDrawStyle);
	
	SoVRMLShape* pathShape = new SoVRMLShape();
	SoVRMLAppearance* pathAppearance = new SoVRMLAppearance();
	SoVRMLMaterial* pathMaterial = new SoVRMLMaterial();
	pathMaterial->diffuseColor.setValue(0.0f, 1.0f, 0.0f);
	pathAppearance->material = pathMaterial;
	pathShape->appearance = pathAppearance;
	pathShape->geometry = this->pathIndexedLineSet;
	this->pathIndexedLineSet->coord = this->pathCoordinate;
	
	this->path->addChild(pathShape);
	
	this->root->addChild(this->path);
	
	// path3
	
	this->path3->whichChoice = SO_SWITCH_ALL;
	
	this->path3DrawStyle->lineWidth = 4.0f;
	this->path3DrawStyle->pointSize = 0.0f;
	this->path3->addChild(this->path3DrawStyle);
	
	SoVRMLShape* path3Shape = new SoVRMLShape();
	SoVRMLAppearance* path3Appearance = new SoVRMLAppearance();
	SoVRMLMaterial* path3Material = new SoVRMLMaterial();
	path3Material->diffuseColor.setValue(0.0f, 1.0f, 0.0f);
	path3Appearance->material = path3Material;
	path3Shape->appearance = path3Appearance;
	path3Shape->geometry = this->path3IndexedLineSet;
	this->path3IndexedLineSet->coord = this->path3Coordinate;
	
	this->path3->addChild(path3Shape);
	
	this->root->addChild(this->path3);
}

Viewer::~Viewer()
{
	this->root->unref();
}

void
Viewer::drawConfiguration(const rl::math::Vector& q)
{
	this->model->setPosition(q);
	this->model->updateFrames();
}

void
Viewer::drawConfigurationEdge(const rl::math::Vector& u, const rl::math::Vector& v, const bool& free)
{
	this->edges->enableNotify(false);
	
	rl::math::Vector inter(this->model->getDof());
	
	rl::math::Real steps = std::ceil(this->model->distance(u, v) / this->delta);
	
	for (std::size_t l = 0; l < this->model->kinematics->getOperationalDof(); ++l)
	{
		for (std::size_t i = 0; i < steps + 1; ++i)
		{
			this->model->interpolate(u, v, i / steps, inter);
			
			this->model->kinematics->setPosition(inter);
			this->model->kinematics->updateFrames();
			
			this->edgesCoordinate->point.set1Value(
				this->edgesCoordinate->point.getNum(),
				this->model->forwardPosition(l)(0, 3),
				this->model->forwardPosition(l)(1, 3),
				this->model->forwardPosition(l)(2, 3)
			);
			
			this->edgesIndexedLineSet->coordIndex.set1Value(
				this->edgesIndexedLineSet->coordIndex.getNum(),
				this->edgesCoordinate->point.getNum() - 1
			);
			
			if (free)
			{
				this->edgesColor->color.set1Value(
					this->edgesColor->color.getNum(),
					0.0f, 0.5f, 0.0f
				);
			}
			else
			{
				this->edgesColor->color.set1Value(
					this->edgesColor->color.getNum(),
					0.5f, 0.0f, 0.0f
				);
			}
			
			this->edgesIndexedLineSet->colorIndex.set1Value(
				this->edgesIndexedLineSet->coordIndex.getNum(),
				this->edgesCoordinate->point.getNum() - 1
			);
		}
		
		this->edgesIndexedLineSet->coordIndex.set1Value(
			this->edgesIndexedLineSet->coordIndex.getNum(),
			SO_END_FACE_INDEX
		);
	}
	
	this->edges->enableNotify(true);
	
	this->edges->touch();
}

void
Viewer::drawConfigurationVertex(const rl::math::Vector& q, const bool& free)
{
	this->vertices->enableNotify(false);
	
	this->model->kinematics->setPosition(q);
	this->model->kinematics->updateFrames();
	
	for (std::size_t l = 0; l < this->model->kinematics->getOperationalDof(); ++l)
	{
		this->verticesCoordinate->point.set1Value(
			this->verticesCoordinate->point.getNum(),
			this->model->forwardPosition(l)(0, 3),
			this->model->forwardPosition(l)(1, 3),
			this->model->forwardPosition(l)(2, 3)
		);
		
		this->verticesColor->color.set1Value(
			this->verticesColor->color.getNum(),
			0.0f, 0.5f, 0.0f
		);
	}
	
	this->vertices->enableNotify(true);
	
	this->vertices->touch();
}

void
Viewer::drawConfigurationPath(const rl::plan::VectorList& path)
{
	this->path->enableNotify(false);
	
	this->pathCoordinate->point.setNum(0);
	this->pathIndexedLineSet->coordIndex.setNum(0);
	
	rl::math::Vector inter(this->model->getDof());
	
	for (std::size_t l = 0; l < this->model->kinematics->getOperationalDof(); ++l)
	{
		rl::plan::VectorList::const_iterator i = path.begin();
		rl::plan::VectorList::const_iterator j = ++path.begin();
		
		if (i != path.end() && j != path.end())
		{
			this->model->setPosition(*i);
			this->model->updateFrames();
			
			this->pathCoordinate->point.set1Value(
				this->pathCoordinate->point.getNum(),
				this->model->forwardPosition(l)(0, 3),
				this->model->forwardPosition(l)(1, 3),
				this->model->forwardPosition(l)(2, 3)
			);
			
			this->pathIndexedLineSet->coordIndex.set1Value(
				this->pathIndexedLineSet->coordIndex.getNum(),
				this->pathCoordinate->point.getNum() - 1
			);
		}
		
		for (; i != path.end() && j != path.end(); ++i, ++j)
		{
			rl::math::Real steps = std::ceil(this->model->distance(*i, *j) / this->delta);
			
			for (std::size_t k = 1; k < steps + 1; ++k)
			{
				this->model->interpolate(*i, *j, k / steps, inter);
				
				this->model->kinematics->setPosition(inter);
				this->model->kinematics->updateFrames();
				
				this->pathCoordinate->point.set1Value(
					this->pathCoordinate->point.getNum(),
					this->model->forwardPosition(l)(0, 3),
					this->model->forwardPosition(l)(1, 3),
					this->model->forwardPosition(l)(2, 3)
				);
				
				this->pathIndexedLineSet->coordIndex.set1Value(
					this->pathIndexedLineSet->coordIndex.getNum(),
					this->pathCoordinate->point.getNum() - 1
				);
			}
		}
		
		this->pathIndexedLineSet->coordIndex.set1Value(
			this->pathIndexedLineSet->coordIndex.getNum(),
			SO_END_FACE_INDEX
		);
	}
	
	this->path->enableNotify(true);
	
	this->path->touch();
}

void
Viewer::drawLine(const rl::math::Vector& xyz0, const rl::math::Vector& xyz1)
{
	this->linesCoordinate->point.set1Value(
		this->linesCoordinate->point.getNum(),
		xyz0(0),
		xyz0(1),
		xyz0(2)
	);
	
	this->linesIndexedLineSet->coordIndex.set1Value(
		this->linesIndexedLineSet->coordIndex.getNum(),
		this->linesCoordinate->point.getNum() - 1
	);
	
	this->linesCoordinate->point.set1Value(
		this->linesCoordinate->point.getNum(),
		xyz1(0),
		xyz1(1),
		xyz1(2)
	);
	
	this->linesIndexedLineSet->coordIndex.set1Value(
		this->linesIndexedLineSet->coordIndex.getNum(),
		this->linesCoordinate->point.getNum() - 1
	);
	
	this->linesIndexedLineSet->coordIndex.set1Value(
		this->linesIndexedLineSet->coordIndex.getNum(),
		SO_END_FACE_INDEX
	);
}

void
Viewer::drawSphere(const rl::math::Vector& center, const rl::math::Real& radius)
{
	SoVRMLTransform* transform = new SoVRMLTransform();
	transform->translation.setValue(center(0), center(1), center(2));
	
	SoVRMLShape* shape = new SoVRMLShape();
	
	SoVRMLAppearance* appearance = new SoVRMLAppearance();
	SoVRMLMaterial* material = new SoVRMLMaterial();
	material->diffuseColor.setValue(0.0f, 1.0f, 0.0f);
	material->transparency.setValue(0.9f);
	appearance->material = material;
	
	shape->appearance = appearance;
	
	SoVRMLSphere* sphere = new SoVRMLSphere();
	sphere->radius = radius;
	
	shape->geometry = sphere;
	
	transform->addChild(shape);
	
	this->spheres->addChild(transform);
}

void
Viewer::drawSweptVolume(const rl::plan::VectorList& path)
{
	this->swept->enableNotify(false);
	
	this->swept->removeAllChildren();
	
	rl::math::Vector inter(this->model->getDof());
	
	rl::plan::VectorList::const_iterator i = path.begin();
	rl::plan::VectorList::const_iterator j = ++path.begin();
	
	if (i != path.end() && j != path.end())
	{
		this->model->setPosition(*i);
		this->model->updateFrames();
		
		SoVRMLGroup* model = new SoVRMLGroup();
		
		for (std::size_t i = 0; i < this->model->model->getNumBodies(); ++i)
		{
			SoVRMLTransform* frame = new SoVRMLTransform();
			frame->copyFieldValues(static_cast< rl::sg::so::Body* >(this->model->model->getBody(i))->root);
			
			for (std::size_t j = 0; j < this->model->model->getBody(i)->getNumShapes(); ++j)
			{
				SoVRMLTransform* transform = new SoVRMLTransform();
				transform->copyFieldValues(static_cast< rl::sg::so::Shape* >(this->model->model->getBody(i)->getShape(j))->root);
				transform->addChild(static_cast< rl::sg::so::Shape* >(this->model->model->getBody(i)->getShape(j))->shape);
				frame->addChild(transform);
			}
			
			model->addChild(frame);
		}
		
		this->swept->addChild(model);
	}
	
	for (; i != path.end() && j != path.end(); ++i, ++j)
	{
		rl::math::Real steps = std::ceil(this->model->distance(*i, *j) / this->delta);
		
		for (std::size_t k = 1; k < steps + 1; ++k)
		{
			this->model->interpolate(*i, *j, k / steps, inter);
			
			this->model->setPosition(inter);
			this->model->updateFrames();
			
			SoVRMLGroup* model = new SoVRMLGroup();
			
			for (std::size_t i = 0; i < this->model->model->getNumBodies(); ++i)
			{
				SoVRMLTransform* frame = new SoVRMLTransform();
				frame->copyFieldValues(static_cast< rl::sg::so::Body* >(this->model->model->getBody(i))->root);
				
				for (std::size_t j = 0; j < this->model->model->getBody(i)->getNumShapes(); ++j)
				{
					SoVRMLTransform* transform = new SoVRMLTransform();
					transform->copyFieldValues(static_cast< rl::sg::so::Shape* >(this->model->model->getBody(i)->getShape(j))->root);
					transform->addChild(static_cast< rl::sg::so::Shape* >(this->model->model->getBody(i)->getShape(j))->shape);
					frame->addChild(transform);
				}
				
				model->addChild(frame);
			}
			
			this->swept->addChild(model);
		}
	}
	
	this->swept->enableNotify(true);
	
	this->swept->touch();
}

void
Viewer::drawWork(const rl::math::Transform& t)
{
	SbMatrix matrix;
	
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			matrix[i][j] = static_cast< float >(t(j, i));
		}
	}
	
	this->work->setMatrix(matrix);
}

void
Viewer::drawWorkEdge(const rl::math::Vector& u, const rl::math::Vector& v)
{
	this->edgesCoordinate->point.set1Value(
		this->edgesCoordinate->point.getNum(),
		u(0),
		u(1),
		u(2)
	);
	
	this->edgesIndexedLineSet->coordIndex.set1Value(
		this->edgesIndexedLineSet->coordIndex.getNum(),
		this->edgesCoordinate->point.getNum() - 1
	);
	
	this->edgesCoordinate->point.set1Value(
		this->edgesCoordinate->point.getNum(),
		v(0),
		v(1),
		v(2)
	);
	
	this->edgesIndexedLineSet->coordIndex.set1Value(
		this->edgesIndexedLineSet->coordIndex.getNum(),
		this->edgesCoordinate->point.getNum() - 1
	);
	
	this->edgesIndexedLineSet->coordIndex.set1Value(
		this->edgesIndexedLineSet->coordIndex.getNum(),
		SO_END_FACE_INDEX
	);
}

void
Viewer::drawWorkPath(const rl::plan::VectorList& path)
{
	this->path3->enableNotify(false);
	
	this->path3Coordinate->point.setNum(0);
	this->path3IndexedLineSet->coordIndex.setNum(0);
	
	for (rl::plan::VectorList::const_iterator i = path.begin(); i != path.end(); ++i)
	{
		this->path3Coordinate->point.set1Value(
			this->path3Coordinate->point.getNum(),
			(*i)(0),
			(*i)(1),
			(*i)(2)
		);
		
		this->path3IndexedLineSet->coordIndex.set1Value(
			this->path3IndexedLineSet->coordIndex.getNum(),
			this->path3Coordinate->point.getNum() - 1
		);
	}
	
	this->path3IndexedLineSet->coordIndex.set1Value(
		this->path3IndexedLineSet->coordIndex.getNum(),
		SO_END_FACE_INDEX
	);
	
	this->path3->enableNotify(true);
	
	this->path3->touch();
}

void
Viewer::drawWorkVertex(const rl::math::Vector& q)
{
}

void
Viewer::reset()
{
	this->resetEdges();
	this->resetLines();
	this->resetVertices();
	this->pathCoordinate->point.setNum(0);
	this->pathIndexedLineSet->coordIndex.setNum(0);
	this->path3Coordinate->point.setNum(0);
	this->path3IndexedLineSet->coordIndex.setNum(0);
	this->spheres->removeAllChildren();
	this->swept->removeAllChildren();
	this->work->setMatrix(SbMatrix::identity());
}

void
Viewer::resetEdges()
{
	this->edgesCoordinate->point.setNum(0);
	this->edgesIndexedLineSet->coordIndex.setNum(0);
}

void
Viewer::resetLines()
{
	this->linesCoordinate->point.setNum(0);
	this->linesIndexedLineSet->coordIndex.setNum(0);
}

void
Viewer::resetVertices()
{
	this->verticesCoordinate->point.setNum(0);
	this->verticesColor->color.setNum(0);
}

void
Viewer::save(const QString& filename)
{
	QImage image = static_cast< QGLWidget* >(this->viewer->getGLWidget())->grabFrameBuffer(true);
	
	QString format = filename.right(filename.length() - filename.lastIndexOf('.') - 1).toUpper();
	
	if (("JFIF" == format) || ("JPE" == format) || ("JPG" == format))
	{
		format = "JPEG";
	}

	if (!image.save(filename, format.toStdString().c_str()))
	{
		QMessageBox::critical(this, this->windowTitle(), "Error writing " + filename + ".");
	}
}

void
Viewer::toggleConfigurationEdges(const bool& doOn)
{
	if (doOn)
	{
		this->edges->whichChoice = SO_SWITCH_ALL;
	}
	else
	{
		this->edges->whichChoice = SO_SWITCH_NONE;
	}
}

void
Viewer::toggleConfigurationVertices(const bool& doOn)
{
	if (doOn)
	{
		this->vertices->whichChoice = SO_SWITCH_ALL;
	}
	else
	{
		this->vertices->whichChoice = SO_SWITCH_NONE;
	}
}
