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

#include <fstream>
#include <QApplication>
#include <QDateTime>
#include <QMutexLocker>
#include <rl/math/Quaternion.h>
#include <rl/math/Unit.h>
#include <rl/plan/Prm.h>
#include <rl/plan/Rrt.h>
#include <rl/util/Timer.h>

#include "MainWindow.h"
#include "Thread.h"
#include "Viewer.h"

Thread::Thread(QObject* parent) :
	QThread(parent),
	quit(false),
	swept(false),
	running(false)
{
}

Thread::~Thread()
{
}

void
Thread::drawConfiguration(const rl::math::Vector& q)
{
	emit configurationRequested(q);
}

void
Thread::drawConfigurationEdge(const rl::math::Vector& q0, const rl::math::Vector& q1, const bool& free)
{
	emit configurationEdgeRequested(q0, q1, free);
}

void
Thread::drawConfigurationPath(const rl::plan::VectorList& path)
{
	emit configurationPathRequested(path);
}

void
Thread::drawConfigurationVertex(const rl::math::Vector& q, const bool& free)
{
	emit configurationVertexRequested(q, free);
}

void
Thread::drawLine(const rl::math::Vector& xyz0, const rl::math::Vector& xyz1)
{
	emit lineRequested(xyz0, xyz1);
}

void
Thread::drawSphere(const rl::math::Vector& center, const rl::math::Real& radius)
{
	emit sphereRequested(center, radius);
}

void
Thread::drawSweptVolume(const rl::plan::VectorList& path)
{
	emit sweptVolumeRequested(path);
}

void
Thread::drawWork(const rl::math::Transform& t)
{
	emit workRequested(t);
}

void
Thread::drawWorkEdge(const rl::math::Vector& q0, const rl::math::Vector& q1)
{
//	emit workEdgeRequested(q0, q1);
}

void
Thread::drawWorkPath(const rl::plan::VectorList& path)
{
	emit workPathRequested(path);
}

void
Thread::drawWorkVertex(const rl::math::Vector& q)
{
//	emit workVertexRequested(q);
}

void
Thread::reset()
{
	emit resetRequested();
}

void
Thread::resetEdges()
{
	emit edgeResetRequested();
}

void
Thread::resetLines()
{
	emit lineResetRequested();
}

void
Thread::resetVertices()
{
	emit vertexResetRequested();
}

void
Thread::run()
{
	QMutexLocker lock(&MainWindow::instance()->mutex);
	
	this->running = true;
	
	rl::util::Timer timer;
	
	this->drawConfiguration(*MainWindow::instance()->planner->start);
	
	usleep(static_cast< std::size_t >(2.0f * 1000.0f * 1000.0f));
	
	if (!this->running) return;
	
	this->drawConfiguration(*MainWindow::instance()->planner->goal);
	
	usleep(static_cast< std::size_t >(2.0f * 1000.0f * 1000.0f));
	
	if (!this->running) return;
	
	if (!MainWindow::instance()->planner->verify())
	{
		std::cout << "start or goal invalid" << std::endl;
		return;
	}
	
	std::cout << "solve() ... " << std::endl;;
	timer.start();
	bool solved = MainWindow::instance()->planner->solve();
	timer.stop();
	std::cout << "solve() " << (solved ? "true" : "false") << " " << timer.elapsed() * 1000.0f << " ms" << std::endl;
	
	std::ofstream benchmark;
	
	benchmark.open("benchmark.csv", std::ios::app);
	
//	benchmark << "Time,Solved,Robot,Planner,Vertices,Edges,CD,Exploration Duration (s),Duration (s)" << std::endl;
	
	benchmark << QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz").toStdString();
	benchmark << ",";
	benchmark << (solved ? "true" : "false");
	benchmark << ",";
	benchmark << MainWindow::instance()->planner->getName();
	benchmark << ",";
	benchmark << MainWindow::instance()->kinematics->getManufacturer();
	benchmark << (!MainWindow::instance()->kinematics->getManufacturer().empty() && !MainWindow::instance()->kinematics->getName().empty() ? " " : "");
	benchmark << MainWindow::instance()->kinematics->getName();
	benchmark << ",";
	
	if (rl::plan::Prm* prm = dynamic_cast< rl::plan::Prm* >(MainWindow::instance()->planner.get()))
	{
		benchmark << prm->getNumVertices();
	}
	else if (rl::plan::Rrt* rrt = dynamic_cast< rl::plan::Rrt* >(MainWindow::instance()->planner.get()))
	{
		benchmark << rrt->getNumVertices();
	}
	
	benchmark << ",";
	
	if (rl::plan::Prm* prm = dynamic_cast< rl::plan::Prm* >(MainWindow::instance()->planner.get()))
	{
		benchmark << prm->getNumEdges();
	}
	else if (rl::plan::Rrt* rrt = dynamic_cast< rl::plan::Rrt* >(MainWindow::instance()->planner.get()))
	{
		benchmark << rrt->getNumEdges();
	}
	
	benchmark << ",";
	benchmark << MainWindow::instance()->model->getQueries();
	benchmark << ",";
	
	benchmark << 0.0f;
	
	benchmark << ",";
	benchmark << timer.elapsed() << std::endl;
	
	benchmark.close();
	
	if (!this->running) return;
	
	if (this->quit)
	{
		QApplication::quit();
		return;
	}
	
	if (solved)
	{
		rl::plan::VectorList path;
		MainWindow::instance()->planner->getPath(path);
		
		if (this->swept)
		{
			this->drawSweptVolume(path);
			return;
		}
		
		this->drawConfigurationPath(path);
		
		if (!this->running) return;
		
		if (NULL != MainWindow::instance()->optimizer)
		{
			usleep(static_cast< std::size_t >(2.0f * 1000.0f * 1000.0f));
			
			std::cout << "optimize() ... " << std::endl;;
			timer.start();
			MainWindow::instance()->optimizer->process(path);
			timer.stop();
			std::cout << "optimize() " << timer.elapsed() * 1000.0f << " ms" << std::endl;
			
			this->drawConfigurationPath(path);
		}
		
		rl::math::Vector diff(MainWindow::instance()->model->getDof());
		rl::math::Vector inter(MainWindow::instance()->model->getDof());
		
		while (true)
		{
			if (!this->running) break;
			
			rl::plan::VectorList::iterator i = path.begin();
			rl::plan::VectorList::iterator j = ++path.begin();
			
			if (i != path.end() && j != path.end())
			{
				this->drawConfiguration(*i);
				usleep(static_cast< std::size_t >(0.01f * 1000.0f * 1000.0f));
			}
			
			rl::math::Real delta = MainWindow::instance()->viewer->delta;
			
			for (; i != path.end() && j != path.end(); ++i, ++j)
			{
				boost::numeric::bindings::ippm::sub_vv(*j, *i, diff);
				
				rl::math::Real steps = std::ceil(MainWindow::instance()->model->distance(*i, *j) / delta);
				
				for (std::size_t k = 1; k < steps + 1; ++k)
				{
					if (!this->running) break;
					
					MainWindow::instance()->model->interpolate(*i, *j, k / steps, inter);
					this->drawConfiguration(inter);
					usleep(static_cast< std::size_t >(0.01f * 1000.0f * 1000.0f));
				}
			}
			
			if (!this->running) break;
			
			rl::plan::VectorList::reverse_iterator ri = path.rbegin();
			rl::plan::VectorList::reverse_iterator rj = ++path.rbegin();
			
			if (ri != path.rend() && rj != path.rend())
			{
				this->drawConfiguration(*ri);
				usleep(static_cast< std::size_t >(0.01f * 1000.0f * 1000.0f));
			}
			
			for (; ri != path.rend() && rj != path.rend(); ++ri, ++rj)
			{
				boost::numeric::bindings::ippm::sub_vv(*rj, *ri, diff);
				
				rl::math::Real steps = std::ceil(MainWindow::instance()->model->distance(*ri, *rj) / delta);
				
				for (std::size_t k = 1; k < steps + 1; ++k)
				{
					if (!this->running) break;
					
					MainWindow::instance()->model->interpolate(*ri, *rj, k / steps, inter);
					this->drawConfiguration(inter);
					usleep(static_cast< std::size_t >(0.01f * 1000.0f * 1000.0f));
				}
			}
		}
	}
}

void
Thread::stop()
{
	if (this->running)
	{
		this->running = false;
		
		while (!this->isFinished())
		{
			QThread::usleep(0);
		}
	}
}
