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

#include <QApplication>
#include <QDateTime>
#include <QDockWidget>
#include <QFileDialog>
#include <QGLWidget>
#include <QGraphicsView>
#include <QHeaderView>
#include <QLayout>
#include <QMenuBar>
#include <QMutexLocker>
#include <QPainter>
#include <QPrinter>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/nodes/SoCamera.h>
#include <Inventor/nodes/SoOrthographicCamera.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/Qt/SoQt.h>
#include <rl/math/Rotation.h>
#include <rl/math/Unit.h>
#include <rl/plan/AddRrtConCon.h>
#include <rl/plan/AdvancedOptimizer.h>
#include <rl/plan/BridgeSampler.h>
#include <rl/plan/DistanceModel.h>
#include <rl/plan/GaussianSampler.h>
#include <rl/plan/Prm.h>
#include <rl/plan/RecursiveVerifier.h>
#include <rl/plan/Rrt.h>
#include <rl/plan/RrtCon.h>
#include <rl/plan/RrtConCon.h>
#include <rl/plan/RrtDual.h>
#include <rl/plan/RrtExtCon.h>
#include <rl/plan/RrtExtExt.h>
#include <rl/plan/RrtGoalBias.h>
#include <rl/plan/SimpleModel.h>
#include <rl/plan/SimpleOptimizer.h>
#include <rl/plan/UniformSampler.h>
#include <rl/plan/WorkspaceSphereExplorer.h>
#include <rl/sg/Body.h>
#include <rl/xml/Attribute.h>
#include <rl/xml/Document.h>
#include <rl/xml/DomParser.h>
#include <rl/xml/Node.h>
#include <rl/xml/Object.h>
#include <rl/xml/Path.h>

#include "ConfigurationDelegate.h"
#include "ConfigurationModel.h"
#include "ConfigurationSpaceScene.h"
#include "MainWindow.h"
#include "Thread.h"
#include "Viewer.h"

MainWindow::MainWindow(QWidget* parent, Qt::WFlags f) :
	QMainWindow(parent, f),
	configurationModel(new ConfigurationModel(this)),
	epsilon(),
	explorerGoals(),
	explorers(),
	explorerStarts(),
	goal(),
	kinematics(),
	kinematics2(),
	model(),
	model2(),
	mutex(),
	planner(),
	q(),
	sampler(),
	sampler2(),
	sigma(),
	scene(),
	scene2(),
	sceneModel(NULL),
	sceneModel2(NULL),
	start(),
	thread(new Thread(this)),
	verifier(),
	verifier2(),
	viewer(NULL),
	configurationDelegate(new ConfigurationDelegate(this)),
	configurationDockWidget(new QDockWidget(this)),
	configurationSpaceDockWidget(new QDockWidget(this)),
	configurationSpaceScene(new ConfigurationSpaceScene(this)),
	configurationSpaceView(new QGraphicsView(this)),
	configurationView(new QTableView(this)),
	evalAction(new QAction(this)),
	exitAction(new QAction(this)),
	fileName(),
	getGoalConfigurationAction(new QAction(this)),
	getRandomConfigurationAction(new QAction(this)),
	getRandomFreeConfigurationAction(new QAction(this)),
	getStartConfigurationAction(new QAction(this)),
	openAction(new QAction(this)),
	resetAction(new QAction(this)),
	saveImageAction(new QAction(this)),
	savePdfAction(new QAction(this)),
	saveSceneAction(new QAction(this)),
	setGoalConfigurationAction(new QAction(this)),
	setStartConfigurationAction(new QAction(this)),
	startThreadAction(new QAction(this)),
	toggleCameraAction(new QAction(this)),
	toggleConfigurationAction(new QAction(this)),
	toggleConfigurationEdgesAction(new QAction(this)),
	toggleConfigurationSpaceAction(new QAction(this)),
	toggleConfigurationVerticesAction(new QAction(this)),
	toggleViewAction(new QAction(this))
{
	MainWindow::singleton = this;
	
	SoQt::init(this);
	SoDB::init();
	
	this->viewer = new Viewer(this);
	this->setCentralWidget(this->viewer);
	
	this->configurationSpaceView->setEnabled(false);
	this->configurationSpaceView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	this->configurationSpaceView->setScene(this->configurationSpaceScene);
	this->configurationSpaceView->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	this->configurationSpaceView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	
	this->configurationView->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
	this->configurationView->horizontalHeader()->hide();
	this->configurationView->setAlternatingRowColors(true);
	this->configurationView->setItemDelegate(this->configurationDelegate);
	this->configurationView->setModel(this->configurationModel);
	this->configurationView->setViewport(new QGLWidget(this->configurationView));
	
	this->configurationDockWidget->hide();
	this->configurationDockWidget->resize(120, 320);
	this->configurationDockWidget->setFloating(true);
	this->configurationDockWidget->setWidget(this->configurationView);
	
	this->configurationSpaceDockWidget->hide();
	this->configurationSpaceDockWidget->setFloating(true);
	this->configurationSpaceDockWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	this->configurationSpaceDockWidget->setWidget(this->configurationSpaceView);
	
	this->init();
	
	if (QApplication::arguments().size() < 2)
	{
		this->open();
	}
	else
	{
		this->load(QApplication::arguments()[1]);
	}
}

MainWindow::~MainWindow()
{
	this->thread->stop();
	
	MainWindow::singleton = NULL;
}

void
MainWindow::connect(const QObject* sender, const QObject* receiver)
{
	QObject::connect(
		sender,
		SIGNAL(configurationRequested(const rl::math::Vector&)),
		receiver,
		SLOT(drawConfiguration(const rl::math::Vector&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(configurationEdgeRequested(const rl::math::Vector&, const rl::math::Vector&, const bool&)),
		receiver,
		SLOT(drawConfigurationEdge(const rl::math::Vector&, const rl::math::Vector&, const bool&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(configurationPathRequested(const rl::plan::VectorList&)),
		receiver,
		SLOT(drawConfigurationPath(const rl::plan::VectorList&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(configurationVertexRequested(const rl::math::Vector&, const bool&)),
		receiver,
		SLOT(drawConfigurationVertex(const rl::math::Vector&, const bool&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(edgeResetRequested()),
		receiver,
		SLOT(resetEdges())
	);
	
	QObject::connect(
		sender,
		SIGNAL(lineRequested(const rl::math::Vector&, const rl::math::Vector&)),
		receiver,
		SLOT(drawLine(const rl::math::Vector&, const rl::math::Vector&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(lineResetRequested()),
		receiver,
		SLOT(resetLines())
	);
	
	QObject::connect(
		sender,
		SIGNAL(resetRequested()),
		receiver,
		SLOT(reset())
	);
	
	QObject::connect(
		sender,
		SIGNAL(sphereRequested(const rl::math::Vector&, const rl::math::Real&)),
		receiver,
		SLOT(drawSphere(const rl::math::Vector&, const rl::math::Real&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(sweptVolumeRequested(const rl::plan::VectorList&)),
		receiver,
		SLOT(drawSweptVolume(const rl::plan::VectorList&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(vertexResetRequested()),
		receiver,
		SLOT(resetVertices())
	);
	
	QObject::connect(
		sender,
		SIGNAL(workRequested(const rl::math::Transform&)),
		receiver,
		SLOT(drawWork(const rl::math::Transform&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(workEdgeRequested(const rl::math::Vector&, const rl::math::Vector&)),
		receiver,
		SLOT(drawWorkEdge(const rl::math::Vector&, const rl::math::Vector&))
	);
	
	QObject::connect(
		sender,
		SIGNAL(workPathRequested(const rl::plan::VectorList&)),
		receiver,
		SLOT(drawWorkPath(const rl::plan::VectorList&))
	);
}

void
MainWindow::disconnect(const QObject* sender, const QObject* receiver)
{
	QObject::disconnect(sender, NULL, receiver, NULL);
}

void
MainWindow::eval()
{
	this->configurationSpaceScene->eval();
}

void
MainWindow::getGoalConfiguration()
{
	boost::numeric::bindings::ipps::copy(*this->goal, *this->q);
	this->configurationModel->invalidate();
	this->viewer->drawConfiguration(*this->q);
}

void
MainWindow::getRandomConfiguration()
{
	this->sampler2->generate(*this->q);
	this->configurationModel->invalidate();
	this->viewer->drawConfiguration(*this->q);
}

void
MainWindow::getRandomFreeConfiguration()
{
	this->sampler2->generateCollisionFree(*this->q);
	this->configurationModel->invalidate();
	this->viewer->drawConfiguration(*this->q);
}

void
MainWindow::getStartConfiguration()
{
	boost::numeric::bindings::ipps::copy(*this->start, *this->q);
	this->configurationModel->invalidate();
	this->viewer->drawConfiguration(*this->q);
}

void
MainWindow::init()
{
	QMenu* fileMenu = this->menuBar()->addMenu("File");
	
	this->openAction->setText("Open...");
	this->openAction->setShortcut(QKeySequence::Open);
	QObject::connect(this->openAction, SIGNAL(triggered()), this, SLOT(open()));
	this->addAction(this->openAction);
	fileMenu->addAction(this->openAction);
	
	fileMenu->addSeparator();
	
	this->saveImageAction->setText("Save as PNG");
	this->saveImageAction->setShortcut(QKeySequence("Return"));
	QObject::connect(this->saveImageAction, SIGNAL(triggered()), this, SLOT(saveImage()));
	this->addAction(this->saveImageAction);
	fileMenu->addAction(this->saveImageAction);
	
	this->saveSceneAction->setText("Save as VRML");
	this->saveSceneAction->setShortcut(QKeySequence("Ctrl+Return"));
	QObject::connect(this->saveSceneAction, SIGNAL(triggered()), this, SLOT(saveScene()));
	this->addAction(this->saveSceneAction);
	fileMenu->addAction(this->saveSceneAction);
	
	this->savePdfAction->setText("Save as PDF");
	this->savePdfAction->setShortcut(QKeySequence("Alt+Return"));
	QObject::connect(this->savePdfAction, SIGNAL(triggered()), this, SLOT(savePdf()));
	this->addAction(this->savePdfAction);
	fileMenu->addAction(this->savePdfAction);
	
	fileMenu->addSeparator();
	
	this->exitAction->setText("Exit");
	QObject::connect(this->exitAction, SIGNAL(triggered()), qApp, SLOT(quit()));
	this->addAction(this->exitAction);
	fileMenu->addAction(this->exitAction);
	
	QMenu* configurationMenu = this->menuBar()->addMenu("Configuration");
	
	this->toggleConfigurationAction->setText("Show/Hide");
	this->toggleConfigurationAction->setShortcut(QKeySequence("F5"));
	QObject::connect(this->toggleConfigurationAction, SIGNAL(triggered()), this, SLOT(toggleConfiguration()));
	this->addAction(this->toggleConfigurationAction);
	configurationMenu->addAction(this->toggleConfigurationAction);
	
	this->getRandomConfigurationAction->setText("Random");
	this->getRandomConfigurationAction->setShortcut(QKeySequence("F3"));
	QObject::connect(this->getRandomConfigurationAction, SIGNAL(triggered()), this, SLOT(getRandomConfiguration()));
	this->addAction(this->getRandomConfigurationAction);
	configurationMenu->addAction(this->getRandomConfigurationAction);
	
	this->getRandomFreeConfigurationAction->setText("Random (Collision-Free)");
	this->getRandomFreeConfigurationAction->setShortcut(QKeySequence("F4"));
	QObject::connect(this->getRandomFreeConfigurationAction, SIGNAL(triggered()), this, SLOT(getRandomFreeConfiguration()));
	this->addAction(this->getRandomFreeConfigurationAction);
	configurationMenu->addAction(this->getRandomFreeConfigurationAction);
	
	QMenu* cSpaceMenu = this->menuBar()->addMenu("C-Space");
	
	this->toggleConfigurationSpaceAction->setText("Show/Hide");
	this->toggleConfigurationSpaceAction->setShortcut(QKeySequence("F6"));
	QObject::connect(this->toggleConfigurationSpaceAction, SIGNAL(triggered()), this, SLOT(toggleConfigurationSpace()));
	this->addAction(this->toggleConfigurationSpaceAction);
	cSpaceMenu->addAction(this->toggleConfigurationSpaceAction);
	
	this->evalAction->setText("Evaluate");
	this->evalAction->setShortcut(QKeySequence("F11"));
	QObject::connect(this->evalAction, SIGNAL(triggered()), this, SLOT(eval()));
	this->addAction(this->evalAction);
	cSpaceMenu->addAction(this->evalAction);
	
	QMenu* plannerMenu = this->menuBar()->addMenu("Planner");
	
	this->getStartConfigurationAction->setText("Get Start Configuration");
	this->getStartConfigurationAction->setShortcut(QKeySequence("F1"));
	QObject::connect(this->getStartConfigurationAction, SIGNAL(triggered()), this, SLOT(getStartConfiguration()));
	this->addAction(this->getStartConfigurationAction);
	plannerMenu->addAction(this->getStartConfigurationAction);
	
	this->setStartConfigurationAction->setText("Set Start Configuration");
	this->setStartConfigurationAction->setShortcut(QKeySequence("CTRL+F1"));
	QObject::connect(this->setStartConfigurationAction, SIGNAL(triggered()), this, SLOT(setStartConfiguration()));
	this->addAction(this->setStartConfigurationAction);
	plannerMenu->addAction(this->setStartConfigurationAction);
	
	plannerMenu->addSeparator();
	
	this->getGoalConfigurationAction->setText("Get Goal Configuration");
	this->getGoalConfigurationAction->setShortcut(QKeySequence("F2"));
	QObject::connect(this->getGoalConfigurationAction, SIGNAL(triggered()), this, SLOT(getGoalConfiguration()));
	this->addAction(this->getGoalConfigurationAction);
	plannerMenu->addAction(this->getGoalConfigurationAction);
	
	this->setGoalConfigurationAction->setText("Set Goal Configuration");
	this->setGoalConfigurationAction->setShortcut(QKeySequence("CTRL+F2"));
	QObject::connect(this->setGoalConfigurationAction, SIGNAL(triggered()), this, SLOT(setGoalConfiguration()));
	this->addAction(this->setGoalConfigurationAction);
	plannerMenu->addAction(this->setGoalConfigurationAction);
	
	plannerMenu->addSeparator();
	
	this->startThreadAction->setText("Start");
	this->startThreadAction->setShortcut(QKeySequence("Space"));
	QObject::connect(this->startThreadAction, SIGNAL(triggered()), this, SLOT(startThread()));
	this->addAction(this->startThreadAction);
	plannerMenu->addAction(this->startThreadAction);
	
	this->resetAction->setText("Reset");
	this->resetAction->setShortcut(QKeySequence("F12"));
	QObject::connect(this->resetAction, SIGNAL(triggered()), this, SLOT(reset()));
	this->addAction(this->resetAction);
	plannerMenu->addAction(this->resetAction);
	
	QMenu* viewMenu = this->menuBar()->addMenu("View");
	
	this->toggleViewAction->setCheckable(true);
	this->toggleViewAction->setText("Active");
	this->toggleViewAction->setShortcut(QKeySequence("F9"));
	QObject::connect(this->toggleViewAction, SIGNAL(toggled(bool)), this, SLOT(toggleView(bool)));
	this->addAction(this->toggleViewAction);
	viewMenu->addAction(this->toggleViewAction);
	
	viewMenu->addSeparator();
	
	this->toggleConfigurationEdgesAction->setCheckable(true);
	this->toggleConfigurationEdgesAction->setChecked(true);
	this->toggleConfigurationEdgesAction->setText("Configuration Edges");
	QObject::connect(this->toggleConfigurationEdgesAction, SIGNAL(toggled(bool)), this->viewer, SLOT(toggleConfigurationEdges(bool)));
	this->addAction(this->toggleConfigurationEdgesAction);
	viewMenu->addAction(this->toggleConfigurationEdgesAction);
	
	this->toggleConfigurationVerticesAction->setCheckable(true);
	this->toggleConfigurationVerticesAction->setChecked(false);
	this->toggleConfigurationVerticesAction->setText("Configuration Vertices");
	QObject::connect(this->toggleConfigurationVerticesAction, SIGNAL(toggled(bool)), this->viewer, SLOT(toggleConfigurationVertices(bool)));
	this->addAction(this->toggleConfigurationVerticesAction);
	viewMenu->addAction(this->toggleConfigurationVerticesAction);
	
	viewMenu->addSeparator();
	
	this->toggleCameraAction->setText("Perspective/Orthographic");
	this->toggleCameraAction->setShortcut(QKeySequence("F9"));
	QObject::connect(this->toggleCameraAction, SIGNAL(triggered()), this, SLOT(toggleCamera()));
	this->addAction(this->toggleCameraAction);
	viewMenu->addAction(this->toggleCameraAction);
}

MainWindow*
MainWindow::instance()
{
	if (NULL == MainWindow::singleton)
	{
		new MainWindow();
	}
	
	return MainWindow::singleton;
}

void
MainWindow::load(const QString& fileName)
{
	QMutexLocker lock(&this->mutex);
	
	rl::xml::DomParser parser;
	
	rl::xml::Document doc = parser.readFile(fileName.toStdString(), "", XML_PARSE_NOENT | XML_PARSE_XINCLUDE);
	
	this->fileName = fileName;
	this->setWindowTitle(fileName + " - testPlan");
	
	doc.substitute(XML_PARSE_NOENT | XML_PARSE_XINCLUDE);
	
	rl::xml::Path path(doc);
	
	this->viewer->setMinimumSize(
		static_cast< int >(path.eval("number(//viewer/size/width)").getFloatval(640)),
		static_cast< int >(path.eval("number(//viewer/size/height)").getFloatval(480))
	);
	
	this->scene.reset(new rl::sg::solid::Scene());
	
	rl::xml::Object modelScene = path.eval("//model/scene");
	this->scene->load(modelScene.getNodeTab(0).getUri(modelScene.getNodeTab(0).getAttribute("href").getValue()));
	this->sceneModel = static_cast< rl::sg::solid::Model* >(this->scene->getModel(
		static_cast< int >(path.eval("number(//model/model)").getFloatval())
	));
	
	rl::xml::Object kinematics = path.eval("//model/kinematics");
	this->kinematics.reset(rl::kin::Kinematics::create(
		kinematics.getNodeTab(0).getUri(kinematics.getNodeTab(0).getAttribute("href").getValue())
	));
	
	if (path.eval("count(//model/kinematics/world) > 0").getBoolval())
	{
		rl::math::rotation::fromXyz(
			path.eval("number(//model/kinematics/world/rotation/x)").getFloatval(0.0f) * rl::math::DEG2RAD,
			path.eval("number(//model/kinematics/world/rotation/y)").getFloatval(0.0f) * rl::math::DEG2RAD,
			path.eval("number(//model/kinematics/world/rotation/z)").getFloatval(0.0f) * rl::math::DEG2RAD,
			this->kinematics->world()
		);
		
		rl::math::transform::setTranslation(
			path.eval("number(//model/kinematics/world/translation/x)").getFloatval(0.0f),
			path.eval("number(//model/kinematics/world/translation/y)").getFloatval(0.0f),
			path.eval("number(//model/kinematics/world/translation/z)").getFloatval(0.0f),
			this->kinematics->world()
		);
	}
	
	this->model.reset(new rl::plan::DistanceModel());
	this->model->kinematics = this->kinematics.get();
	this->model->model = this->sceneModel;
	this->model->scene = this->scene.get();
	
	this->q.reset(new rl::math::Vector(this->kinematics->getDof()));
	
	this->scene2.reset(new rl::sg::so::Scene());
	
	rl::xml::Object viewerScene = path.eval("//viewer/model/scene");
	this->scene2->load(viewerScene.getNodeTab(0).getUri(viewerScene.getNodeTab(0).getAttribute("href").getValue()));
	this->sceneModel2 = static_cast< rl::sg::so::Model* >(this->scene2->getModel(
		static_cast< int >(path.eval("number(//viewer/model/model)").getFloatval())
	));
	
	rl::xml::Object kinematics2 = path.eval("//viewer/model/kinematics");
	this->kinematics2.reset(rl::kin::Kinematics::create(
		kinematics2.getNodeTab(0).getUri(kinematics2.getNodeTab(0).getAttribute("href").getValue())
	));
	
	if (path.eval("count(//viewer/model/kinematics/world) > 0").getBoolval())
	{
		rl::math::rotation::fromXyz(
			path.eval("number(//viewer/model/kinematics/world/rotation/x)").getFloatval(0.0f) * rl::math::DEG2RAD,
			path.eval("number(//viewer/model/kinematics/world/rotation/y)").getFloatval(0.0f) * rl::math::DEG2RAD,
			path.eval("number(//viewer/model/kinematics/world/rotation/z)").getFloatval(0.0f) * rl::math::DEG2RAD,
			this->kinematics2->world()
		);
		
		rl::math::transform::setTranslation(
			path.eval("number(//viewer/model/kinematics/world/translation/x)").getFloatval(0.0f),
			path.eval("number(//viewer/model/kinematics/world/translation/y)").getFloatval(0.0f),
			path.eval("number(//viewer/model/kinematics/world/translation/z)").getFloatval(0.0f),
			this->kinematics2->world()
		);
	}
	
	this->model2.reset(new rl::plan::Model());
	this->model2->kinematics = this->kinematics2.get();
	this->model2->model = this->sceneModel2;
	this->model2->scene = this->scene2.get();
	
	rl::xml::Object start = path.eval("//start//q");
	this->start.reset(new rl::math::Vector(start.getNodeNr()));
	
	for (int i = 0; i < start.getNodeNr(); ++i)
	{
		(*this->start)(i) = std::atof(start.getNodeTab(i).getContent().c_str());
		
		if (start.getNodeTab(i).hasAttribute("unit"))
		{
			if ("deg" == start.getNodeTab(i).getAttribute("unit").getValue())
			{
				(*this->start)(i) *= rl::math::DEG2RAD;
			}
		}
	}
	
	boost::numeric::bindings::ipps::copy(*this->start, *this->q);
	
	rl::xml::Object goal = path.eval("//goal//q");
	this->goal.reset(new rl::math::Vector(goal.getNodeNr()));
	
	for (int i = 0; i < goal.getNodeNr(); ++i)
	{
		(*this->goal)(i) = std::atof(goal.getNodeTab(i).getContent().c_str());
		
		if (goal.getNodeTab(i).hasAttribute("unit"))
		{
			if ("deg" == goal.getNodeTab(i).getAttribute("unit").getValue())
			{
				(*this->goal)(i) *= rl::math::DEG2RAD;
			}
		}
	}
	
	rl::xml::Object epsilon = path.eval("//epsilon//q");
	this->epsilon.reset(new rl::math::Vector(epsilon.getNodeNr()));
	
	for (int i = 0; i < epsilon.getNodeNr(); ++i)
	{
		(*this->epsilon)(i) = std::atof(epsilon.getNodeTab(i).getContent().c_str());
		
		if (epsilon.getNodeTab(i).hasAttribute("unit"))
		{
			if ("deg" == epsilon.getNodeTab(i).getAttribute("unit").getValue())
			{
				(*this->epsilon)(i) *= rl::math::DEG2RAD;
			}
		}
	}
	
	rl::xml::Object sigma = path.eval("//sigma//q");
	this->sigma.reset(new rl::math::Vector(sigma.getNodeNr()));
	
	for (int i = 0; i < sigma.getNodeNr(); ++i)
	{
		(*this->sigma)(i) = std::atof(sigma.getNodeTab(i).getContent().c_str());
		
		if (sigma.getNodeTab(i).hasAttribute("unit"))
		{
			if ("deg" == sigma.getNodeTab(i).getAttribute("unit").getValue())
			{
				(*this->sigma)(i) *= rl::math::DEG2RAD;
			}
		}
	}
	
	if (path.eval("count(//uniformSampler) > 0").getBoolval())
	{
		this->sampler.reset(new rl::plan::UniformSampler());
		rl::plan::UniformSampler* uniformSampler = static_cast< rl::plan::UniformSampler* >(this->sampler.get());
		
		if (path.eval("count(//uniformSampler/seed) > 0").getBoolval())
		{
			uniformSampler->seed(
				static_cast< boost::mt19937::result_type >(path.eval("number(//uniformSampler/seed)").getFloatval(rl::util::Timer::now() * 1000000.0f))
			);
		}
	}
	else if (path.eval("count(//gaussianSampler) > 0").getBoolval())
	{
		this->sampler.reset(new rl::plan::GaussianSampler());
		rl::plan::GaussianSampler* gaussianSampler = static_cast< rl::plan::GaussianSampler* >(this->sampler.get());
		
		if (path.eval("count(//gaussianSampler/seed) > 0").getBoolval())
		{
			gaussianSampler->seed(
				static_cast< boost::mt19937::result_type >(path.eval("number(//gaussianSampler/seed)").getFloatval(rl::util::Timer::now() * 1000000.0f))
			);
		}
		
		gaussianSampler->sigma = this->sigma.get();
	}
	else if (path.eval("count(//bridgeSampler) > 0").getBoolval())
	{
		this->sampler.reset(new rl::plan::BridgeSampler());
		rl::plan::BridgeSampler* bridgeSampler = static_cast< rl::plan::BridgeSampler* >(this->sampler.get());
		bridgeSampler->ratio = path.eval("number(//bridgeSampler/ratio)").getFloatval(5.0f / 6.0f);
		
		if (path.eval("count(//bridgeSampler/seed) > 0").getBoolval())
		{
			bridgeSampler->seed(
				static_cast< boost::mt19937::result_type >(path.eval("number(//bridgeSampler/seed)").getFloatval(rl::util::Timer::now() * 1000000.0f))
			);
		}
		
		bridgeSampler->sigma = this->sigma.get();
	}
	
	if (NULL != this->sampler)
	{
		this->sampler->model = this->model.get();
	}
	
	this->sampler2.reset(new rl::plan::UniformSampler());
	this->sampler2->model = this->model.get();
	
	if (path.eval("count(//recursiveVerifier) > 0").getBoolval())
	{
		this->verifier.reset(new rl::plan::RecursiveVerifier());
		this->verifier->delta = path.eval("number(//recursiveVerifier/delta)").getFloatval(1.0f);
		
		if ("deg" == path.eval("string(//recursiveVerifier/delta/@unit)").getStringval())
		{
			this->verifier->delta *= rl::math::DEG2RAD;
		}
	}
	
	if (NULL != this->verifier)
	{
		this->verifier->model = this->model.get();
	}
	
	if (path.eval("count(//simpleOptimizer/recursiveVerifier) > 0").getBoolval())
	{
		this->verifier2.reset(new rl::plan::RecursiveVerifier());
		this->verifier2->delta = path.eval("number(//simpleOptimizer/recursiveVerifier/delta)").getFloatval(1.0f);
		
		if ("deg" == path.eval("string(//simpleOptimizer/recursiveVerifier/delta/@unit)").getStringval())
		{
			this->verifier2->delta *= rl::math::DEG2RAD;
		}
	}
	else if (path.eval("count(//advancedOptimizer/recursiveVerifier) > 0").getBoolval())
	{
		this->verifier2.reset(new rl::plan::RecursiveVerifier());
		this->verifier2->delta = path.eval("number(//advancedOptimizer/recursiveVerifier/delta)").getFloatval(1.0f);
		
		if ("deg" == path.eval("string(//advancedOptimizer/recursiveVerifier/delta/@unit)").getStringval())
		{
			this->verifier2->delta *= rl::math::DEG2RAD;
		}
	}
	
	if (NULL != this->verifier2)
	{
		this->verifier2->model = this->model.get();
	}
	
	if (path.eval("count(//simpleOptimizer) > 0").getBoolval())
	{
		this->optimizer.reset(new rl::plan::SimpleOptimizer());
		rl::plan::SimpleOptimizer* simpleOptimizer = static_cast< rl::plan::SimpleOptimizer* >(this->optimizer.get());
	}
	else if (path.eval("count(//advancedOptimizer) > 0").getBoolval())
	{
		this->optimizer.reset(new rl::plan::AdvancedOptimizer());
		rl::plan::AdvancedOptimizer* advancedOptimizer = static_cast< rl::plan::AdvancedOptimizer* >(this->optimizer.get());
		advancedOptimizer->length = path.eval("number(//advancedOptimizer/length)").getFloatval(1.0f);
		
		if ("deg" == path.eval("string(//advancedOptimizer/length/@unit)").getStringval())
		{
			advancedOptimizer->length *= rl::math::DEG2RAD;
		}
		
		advancedOptimizer->ratio = path.eval("number(//advancedOptimizer/ratio)").getFloatval(0.1f);
	}
	
	if (NULL != this->optimizer)
	{
		this->optimizer->model = this->model.get();
		this->optimizer->verifier = this->verifier2.get();
	}
	
	rl::xml::Object planner = path.eval("//addRrtConCon|//prm|//rrt|//rrtCon|//rrtConCon|//rrtConExt|//rrtDual|//rrtGoalBias|//rrtExtCon|//rrtExtExt");
	
	if ("addRrtConCon" == planner.getNodeTab(0).getName())
	{
		this->planner.reset(new rl::plan::AddRrtConCon());
		rl::plan::AddRrtConCon* addRrtConCon = static_cast< rl::plan::AddRrtConCon* >(this->planner.get());
		addRrtConCon->alpha = path.eval("number(alpha)", planner.getNodeTab(0)).getFloatval(0.05f);
		addRrtConCon->delta = path.eval("number(delta)", planner.getNodeTab(0)).getFloatval(1.0f);
		
		if ("deg" == path.eval("string(delta/@unit)", planner.getNodeTab(0)).getStringval())
		{
			addRrtConCon->delta *= rl::math::DEG2RAD;
		}
		
		addRrtConCon->epsilon = this->epsilon.get();
		addRrtConCon->kd = path.eval("count(bruteForce) > 0", planner.getNodeTab(0)).getBoolval() ? false : true;
		addRrtConCon->lower = path.eval("number(lower)", planner.getNodeTab(0)).getFloatval(2.0f);
		
		if ("deg" == path.eval("string(lower/@unit)", planner.getNodeTab(0)).getStringval())
		{
			addRrtConCon->lower *= rl::math::DEG2RAD;
		}
		
		addRrtConCon->radius = path.eval("number(radius)", planner.getNodeTab(0)).getFloatval(20.0f);
		
		if ("deg" == path.eval("string(radius/@unit)", planner.getNodeTab(0)).getStringval())
		{
			addRrtConCon->radius *= rl::math::DEG2RAD;
		}
		
		addRrtConCon->sampler = this->sampler.get();
	}
	else if ("prm" == planner.getNodeTab(0).getName())
	{
		this->planner.reset(new rl::plan::Prm());
		rl::plan::Prm* prm = static_cast< rl::plan::Prm* >(this->planner.get());
		prm->degree = static_cast< int >(path.eval("number(degree)", planner.getNodeTab(0)).getFloatval(std::numeric_limits< std::size_t >::max()));
		prm->k = static_cast< int >(path.eval("number(k)", planner.getNodeTab(0)).getFloatval(30.0f));
		prm->kd = path.eval("count(bruteForce) > 0", planner.getNodeTab(0)).getBoolval() ? false : true;
		prm->radius = path.eval("number(radius)", planner.getNodeTab(0)).getFloatval(std::numeric_limits< rl::math::Real >::max());
		
		if ("deg" == path.eval("string(radius/@unit)", planner.getNodeTab(0)).getStringval())
		{
			prm->radius *= rl::math::DEG2RAD;
		}
		
		prm->sampler = this->sampler.get();
		prm->verifier = this->verifier.get();
	}
	else if ("rrt" == planner.getNodeTab(0).getName())
	{
		this->planner.reset(new rl::plan::Rrt());
		rl::plan::Rrt* rrt = static_cast< rl::plan::Rrt* >(this->planner.get());
		rrt->delta = path.eval("number(delta)", planner.getNodeTab(0)).getFloatval(1.0f);
		
		if ("deg" == path.eval("string(delta/@unit)", planner.getNodeTab(0)).getStringval())
		{
			rrt->delta *= rl::math::DEG2RAD;
		}
		
		rrt->epsilon = this->epsilon.get();
		rrt->kd = path.eval("count(bruteForce) > 0", planner.getNodeTab(0)).getBoolval() ? false : true;
		rrt->sampler = this->sampler.get();
	}
	else if ("rrtCon" == planner.getNodeTab(0).getName())
	{
		this->planner.reset(new rl::plan::RrtCon());
		rl::plan::RrtCon* rrtCon = static_cast< rl::plan::RrtCon* >(this->planner.get());
		rrtCon->delta = path.eval("number(delta)", planner.getNodeTab(0)).getFloatval(1.0f);
		
		if ("deg" == path.eval("string(delta/@unit)", planner.getNodeTab(0)).getStringval())
		{
			rrtCon->delta *= rl::math::DEG2RAD;
		}
		
		rrtCon->epsilon = this->epsilon.get();
		rrtCon->kd = path.eval("count(bruteForce) > 0", planner.getNodeTab(0)).getBoolval() ? false : true;
		rrtCon->probability = path.eval("number(probability)", planner.getNodeTab(0)).getFloatval(0.05f);
		rrtCon->sampler = this->sampler.get();
		
		if (path.eval("count(seed) > 0", planner.getNodeTab(0)).getBoolval())
		{
			rrtCon->seed(
				static_cast< boost::mt19937::result_type >(path.eval("number(seed)", planner.getNodeTab(0)).getFloatval(rl::util::Timer::now() * 1000000.0f))
			);
		}
	}
	else if ("rrtConCon" == planner.getNodeTab(0).getName())
	{
		this->planner.reset(new rl::plan::RrtConCon());
		rl::plan::RrtConCon* rrtConCon = static_cast< rl::plan::RrtConCon* >(this->planner.get());
		rrtConCon->delta = path.eval("number(delta)", planner.getNodeTab(0)).getFloatval(1.0f);
		
		if ("deg" == path.eval("string(delta/@unit)", planner.getNodeTab(0)).getStringval())
		{
			rrtConCon->delta *= rl::math::DEG2RAD;
		}
		
		rrtConCon->epsilon = this->epsilon.get();
		rrtConCon->kd = path.eval("count(bruteForce) > 0", planner.getNodeTab(0)).getBoolval() ? false : true;
		rrtConCon->sampler = this->sampler.get();
	}
	else if ("rrtDual" == planner.getNodeTab(0).getName())
	{
		this->planner.reset(new rl::plan::RrtDual());
		rl::plan::RrtDual* rrtDual = static_cast< rl::plan::RrtDual* >(this->planner.get());
		rrtDual->delta = path.eval("number(delta)", planner.getNodeTab(0)).getFloatval(1.0f);
		
		if ("deg" == path.eval("string(delta/@unit)", planner.getNodeTab(0)).getStringval())
		{
			rrtDual->delta *= rl::math::DEG2RAD;
		}
		
		rrtDual->epsilon = this->epsilon.get();
		rrtDual->kd = path.eval("count(bruteForce) > 0", planner.getNodeTab(0)).getBoolval() ? false : true;
		rrtDual->sampler = this->sampler.get();
	}
	else if ("rrtExtCon" == planner.getNodeTab(0).getName())
	{
		this->planner.reset(new rl::plan::RrtExtCon());
		rl::plan::RrtExtCon* rrtExtCon = static_cast< rl::plan::RrtExtCon* >(this->planner.get());
		rrtExtCon->delta = path.eval("number(delta)", planner.getNodeTab(0)).getFloatval(1.0f);
		
		if ("deg" == path.eval("string(delta/@unit)", planner.getNodeTab(0)).getStringval())
		{
			rrtExtCon->delta *= rl::math::DEG2RAD;
		}
		
		rrtExtCon->epsilon = this->epsilon.get();
		rrtExtCon->kd = path.eval("count(bruteForce) > 0", planner.getNodeTab(0)).getBoolval() ? false : true;
		rrtExtCon->sampler = this->sampler.get();
	}
	else if ("rrtExtExt" == planner.getNodeTab(0).getName())
	{
		this->planner.reset(new rl::plan::RrtExtExt());
		rl::plan::RrtExtExt* rrtExtExt = static_cast< rl::plan::RrtExtExt* >(this->planner.get());
		rrtExtExt->delta = path.eval("number(delta)", planner.getNodeTab(0)).getFloatval(1.0f);
		
		if ("deg" == path.eval("string(delta/@unit)", planner.getNodeTab(0)).getStringval())
		{
			rrtExtExt->delta *= rl::math::DEG2RAD;
		}
		
		rrtExtExt->epsilon = this->epsilon.get();
		rrtExtExt->kd = path.eval("count(bruteForce) > 0", planner.getNodeTab(0)).getBoolval() ? false : true;
		rrtExtExt->sampler = this->sampler.get();
	}
	else if ("rrtGoalBias" == planner.getNodeTab(0).getName())
	{
		this->planner.reset(new rl::plan::RrtGoalBias());
		rl::plan::RrtGoalBias* rrtGoalBias = static_cast< rl::plan::RrtGoalBias* >(this->planner.get());
		rrtGoalBias->delta = path.eval("number(delta)", planner.getNodeTab(0)).getFloatval(1.0f);
		
		if ("deg" == path.eval("string(delta/@unit)", planner.getNodeTab(0)).getStringval())
		{
			rrtGoalBias->delta *= rl::math::DEG2RAD;
		}
		
		rrtGoalBias->epsilon = this->epsilon.get();
		rrtGoalBias->kd = path.eval("count(bruteForce) > 0", planner.getNodeTab(0)).getBoolval() ? false : true;
		rrtGoalBias->probability = path.eval("number(probability)", planner.getNodeTab(0)).getFloatval(0.05f);
		rrtGoalBias->sampler = this->sampler.get();
		
		if (path.eval("count(seed) > 0", planner.getNodeTab(0)).getBoolval())
		{
			rrtGoalBias->seed(
				static_cast< boost::mt19937::result_type >(path.eval("number(seed)", planner.getNodeTab(0)).getFloatval(rl::util::Timer::now() * 1000000.0f))
			);
		}
	}
	
	this->planner->duration = path.eval("number(//duration)").getFloatval(std::numeric_limits< rl::math::Real >::max());
	this->planner->goal = this->goal.get();
	this->planner->model = this->model.get();
	this->planner->start = this->start.get();
	
	this->viewer->delta = path.eval("number(//viewer/delta)").getFloatval();
	
	if ("deg" == path.eval("string(//viewer/delta/@unit)").getStringval())
	{
		this->viewer->delta *= rl::math::DEG2RAD;
	}
	
	this->viewer->root->addChild(this->scene2->root);
	this->viewer->model = this->model2.get();
	
	this->configurationSpaceScene->model = this->model.get();
	
	if (path.eval("count(//viewer/disable) > 0").getBoolval())
	{
		this->toggleView(false);
		this->toggleViewAction->setChecked(false);
	}
	else
	{
		this->toggleView(true);
		this->toggleViewAction->setChecked(true);
	}
	
	if (path.eval("count(//viewer/swept) > 0").getBoolval())
	{
		this->thread->swept = true;
	}
	
	if (path.eval("count(//viewer/quit) > 0").getBoolval())
	{
		this->thread->quit = true;
	}
	
	this->viewer->viewer->setBackgroundColor(SbColor(
		path.eval("number(//viewer/background/r)").getFloatval(0.0f),
		path.eval("number(//viewer/background/g)").getFloatval(0.0f),
		path.eval("number(//viewer/background/b)").getFloatval(0.0f)
	));
	
	if (path.eval("count(//viewer/camera/orthographic) > 0").getBoolval())
	{
		this->viewer->viewer->setCameraType(SoOrthographicCamera::getClassTypeId());
	}
	else
	{
		this->viewer->viewer->setCameraType(SoPerspectiveCamera::getClassTypeId());
	}
	
	this->viewer->viewer->getCamera()->setToDefaults();
	
	this->viewer->viewer->viewAll();
	
	this->viewer->viewer->getCamera()->position.setValue(
		path.eval("number(//viewer/camera/position/x)").getFloatval(this->viewer->viewer->getCamera()->position.getValue()[0]),
		path.eval("number(//viewer/camera/position/y)").getFloatval(this->viewer->viewer->getCamera()->position.getValue()[1]),
		path.eval("number(//viewer/camera/position/z)").getFloatval(this->viewer->viewer->getCamera()->position.getValue()[2])
	);
	
	if (path.eval("count(//viewer/camera/target) > 0").getBoolval())
	{
		this->viewer->viewer->getCamera()->pointAt(
			SbVec3f(
				path.eval("number(//viewer/camera/target/x)").getFloatval(0.0f),
				path.eval("number(//viewer/camera/target/y)").getFloatval(0.0f),
				path.eval("number(//viewer/camera/target/z)").getFloatval(0.0f)
			),
			SbVec3f(
				path.eval("number(//viewer/camera/up/x)").getFloatval(0.0f),
				path.eval("number(//viewer/camera/up/y)").getFloatval(0.0f),
				path.eval("number(//viewer/camera/up/z)").getFloatval(1.0f)
			)
		);
	}
	
	this->viewer->viewer->getCamera()->scaleHeight(
		path.eval("number(//viewer/camera/scale)").getFloatval(1.0f)
	);
	
	if (path.eval("count(//viewer/cspace) > 0").getBoolval())
	{
		this->evalAction->setEnabled(true);
		this->savePdfAction->setEnabled(true);
		this->toggleConfigurationSpaceAction->setEnabled(true);
		
		this->configurationSpaceScene->delta = path.eval("number(//viewer/cspace/delta)").getFloatval(1.0f);
		
		if ("deg" == path.eval("string(//viewer/cspace/delta/@unit)").getStringval())
		{
			this->configurationSpaceScene->delta *= rl::math::DEG2RAD;
		}
		
		this->configurationSpaceScene->x = static_cast< std::size_t >(path.eval("number(//viewer/cspace/x)").getFloatval(0.0f));
		this->configurationSpaceScene->y = static_cast< std::size_t >(path.eval("number(//viewer/cspace/y)").getFloatval(1.0f));
		
		this->configurationSpaceScene->eval();
		
		qreal scale = static_cast< std::size_t >(path.eval("number(//viewer/cspace/scale)").getFloatval(1.0f));
		
		this->configurationSpaceView->setEnabled(true);
		
		this->configurationSpaceView->setSceneRect(
			this->model->getMinimum(this->configurationSpaceScene->x),
			-this->model->getMaximum(this->configurationSpaceScene->y),
			std::abs(this->model->getMaximum(this->configurationSpaceScene->x) - this->model->getMinimum(this->configurationSpaceScene->x)),
			std::abs(this->model->getMaximum(this->configurationSpaceScene->y) - this->model->getMinimum(this->configurationSpaceScene->y))
		);
		
		this->configurationSpaceView->resetMatrix();
		this->configurationSpaceView->scale(scale, scale);
		
		this->configurationSpaceView->adjustSize();
		this->configurationSpaceDockWidget->adjustSize();
		
		this->configurationSpaceDockWidget->setUpdatesEnabled(false);
		this->configurationSpaceDockWidget->setFloating(!this->configurationSpaceDockWidget->isFloating());
		this->configurationSpaceDockWidget->setFloating(!this->configurationSpaceDockWidget->isFloating());
		this->configurationSpaceDockWidget->setUpdatesEnabled(true);
	}
	else
	{
		this->evalAction->setEnabled(false);
		this->savePdfAction->setEnabled(false);
		this->toggleConfigurationSpaceAction->setEnabled(false);
		
		this->configurationSpaceScene->clear();
		this->configurationSpaceDockWidget->hide();
		this->configurationSpaceView->setEnabled(false);
		this->disconnect(this->thread, this->configurationSpaceScene);
	}
	
	this->viewer->drawConfiguration(*this->start);
	
	this->configurationModel->invalidate();
	
	if (path.eval("count(//viewer/wait) < 1").getBoolval())
	{
		this->startThread();
	}
}

void
MainWindow::open()
{
	this->reset();
	
	QString fileName = QFileDialog::getOpenFileName(this, "", this->fileName, "All Formats (*.xml)");
	
	if (!fileName.isEmpty())
	{
		this->load(fileName);
	}
}

void
MainWindow::reset()
{
	rl::math::Real duration = this->planner->duration;
	
	this->thread->blockSignals(true);
	QCoreApplication::processEvents();
	this->planner->duration = 0;
	this->thread->stop();
	this->planner->duration = duration;
	this->thread->blockSignals(false);
	
	this->planner->reset();
	this->viewer->reset();
	this->configurationSpaceScene->reset();
	
	this->configurationView->setEnabled(true);
	this->evalAction->setEnabled(true);
	this->getGoalConfigurationAction->setEnabled(true);
	this->getRandomConfigurationAction->setEnabled(true);
	this->getRandomFreeConfigurationAction->setEnabled(true);
	this->getStartConfigurationAction->setEnabled(true);
	this->openAction->setEnabled(true);
	this->setGoalConfigurationAction->setEnabled(true);
	this->setStartConfigurationAction->setEnabled(true);
	this->startThreadAction->setEnabled(true);
	this->toggleViewAction->setEnabled(true);
}

void
MainWindow::saveImage()
{
	this->viewer->save("rlplan-" + QDateTime::currentDateTime().toString("yyyyMMdd-HHmmsszzz") + ".png");
}

void
MainWindow::savePdf()
{
	QPrinter printer;
	printer.setOutputFileName("rlplan-" + QDateTime::currentDateTime().toString("yyyyMMdd-HHmmsszzz") + ".pdf");
	printer.setOutputFormat(QPrinter::PdfFormat);
	printer.setPageSize(QPrinter::A4);
	
	QPainter painter(&printer);
	
	this->configurationSpaceScene->render(&painter);
}

void
MainWindow::saveScene()
{
	SoOutput output;
	
	if (!output.openFile(QString("rlplan-" + QDateTime::currentDateTime().toString("yyyyMMdd-HHmmsszzz") + ".wrl").toStdString().c_str()))
	{
		return;
	}
	
	output.setHeaderString("#VRML V2.0 utf8");
	
	SoWriteAction writeAction(&output);
	writeAction.apply(this->viewer->root);
	
	output.closeFile();
}
void
MainWindow::setGoalConfiguration()
{
	boost::numeric::bindings::ipps::copy(*this->q, *this->goal);
}

void
MainWindow::setStartConfiguration()
{
	boost::numeric::bindings::ipps::copy(*this->q, *this->start);
}

void
MainWindow::startThread()
{
	this->configurationView->setEnabled(false);
	this->evalAction->setEnabled(false);
	this->getGoalConfigurationAction->setEnabled(false);
	this->getRandomConfigurationAction->setEnabled(false);
	this->getRandomFreeConfigurationAction->setEnabled(false);
	this->getStartConfigurationAction->setEnabled(false);
	this->openAction->setEnabled(false);
	this->setGoalConfigurationAction->setEnabled(false);
	this->setStartConfigurationAction->setEnabled(false);
	this->startThreadAction->setEnabled(false);
	this->toggleViewAction->setEnabled(false);
	
	this->thread->start();
}

void
MainWindow::toggleCamera()
{
	if (SoPerspectiveCamera::getClassTypeId() == this->viewer->viewer->getCameraType())
	{
		this->viewer->viewer->setCameraType(SoOrthographicCamera::getClassTypeId());
	}
	else
	{
		this->viewer->viewer->setCameraType(SoPerspectiveCamera::getClassTypeId());
	}
}

void
MainWindow::toggleConfiguration()
{
	if (this->configurationDockWidget->isVisible())
	{
		this->configurationDockWidget->hide();
	}
	else
	{
		this->configurationDockWidget->show();
	}
}

void
MainWindow::toggleConfigurationSpace()
{
	if (this->configurationSpaceView->isEnabled())
	{
		if (this->configurationSpaceDockWidget->isVisible())
		{
			this->configurationSpaceDockWidget->hide();
		}
		else
		{
			this->configurationSpaceDockWidget->show();
		}
	}
}

void
MainWindow::toggleView(const bool& doOn)
{
	if (doOn)
	{
		this->planner->viewer = this->thread;
		
		if (NULL != this->optimizer)
		{
			this->optimizer->viewer = this->thread;
		}
		
		for (std::vector< boost::shared_ptr< rl::plan::WorkspaceSphereExplorer > >::iterator i = this->explorers.begin(); i != this->explorers.end(); ++i)
		{
			(*i)->viewer = this->thread;
		}
		
		this->connect(this->thread, this->configurationSpaceScene);
		this->connect(this->thread, this->viewer);
	}
	else
	{
		this->disconnect(this->thread, this->configurationSpaceScene);
		this->disconnect(this->thread, this->viewer);
		
		this->planner->viewer = NULL;
		
		if (NULL != this->optimizer)
		{
			this->optimizer->viewer = NULL;
		}
		
		for (std::vector< boost::shared_ptr< rl::plan::WorkspaceSphereExplorer > >::iterator i = this->explorers.begin(); i != this->explorers.end(); ++i)
		{
			(*i)->viewer = NULL;
		}
	}
}
