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

#include <rl/util/Timer.h>

#include "DistanceModel.h"
#include "Viewer.h"
#include "WorkspaceSphereExplorer.h"

namespace rl
{
	namespace plan
	{
		WorkspaceSphereExplorer::WorkspaceSphereExplorer() :
			goal(),
			greedy(GREEDY_SPACE),
			model(NULL),
			radius(0.0f),
			range(::std::numeric_limits< ::rl::math::Real >::max()),
			samples(10),
			start(),
			viewer(NULL),
			begin(NULL),
			end(NULL),
			graph(),
			queue(),
			rand(
				::boost::mt19937(static_cast< ::boost::mt19937::result_type >(::rl::util::Timer::now() * 1000000.0f)),
				::boost::uniform_on_sphere< ::rl::math::Real >(3)
			)
		{
		}
		
		WorkspaceSphereExplorer::~WorkspaceSphereExplorer()
		{
		}
		
		WorkspaceSphereExplorer::Edge
		WorkspaceSphereExplorer::addEdge(const Vertex& u, const Vertex& v)
		{
			Edge edge = ::boost::add_edge(u, v, this->graph).first;
			
			if (NULL != this->viewer)
			{
				this->viewer->drawWorkEdge(*this->graph[u].sphere.center, *this->graph[v].sphere.center);
			}
			
			return edge;
		}
		
		WorkspaceSphereExplorer::Vertex
		WorkspaceSphereExplorer::addVertex(const WorkspaceSphere& sphere)
		{
			Vertex vertex = ::boost::add_vertex(this->graph);
			this->graph[vertex].index = ::boost::num_vertices(this->graph) - 1;
			this->graph[vertex].sphere = sphere;
			
			if (NULL != this->viewer)
			{
				this->viewer->drawSphere(*this->graph[vertex].sphere.center, this->graph[vertex].sphere.radius);
				this->viewer->drawWorkVertex(*this->graph[vertex].sphere.center);
			}
			
			return vertex;
		}
		
		bool
		WorkspaceSphereExplorer::explore()
		{
			WorkspaceSphere start;
			start.center = VectorPtr(new ::rl::math::Vector(3));
			(*start.center)(0) = (*this->start)(0);
			(*start.center)(1) = (*this->start)(1);
			(*start.center)(2) = (*this->start)(2);
			start.radius = this->model->distance(*start.center);
			start.parent = NULL;
			start.priority = ::boost::numeric::bindings::ipps::normDiff_L2(*this->goal, *start.center) - start.radius;
			
			this->queue.insert(start);
			
			::std::vector< ::rl::math::Real > sample(3);
			
			while (!this->queue.empty())
			{
				WorkspaceSphere top = *this->queue.begin();
				
				this->queue.erase(this->queue.begin());
				
				if (top.radius >= this->radius)
				{
					Vertex vertex = this->addVertex(top);
					
					if (NULL != top.parent)
					{
						this->addEdge(top.parent, vertex);
					}
					else
					{
						this->begin = vertex;
					}
					
					if (::boost::numeric::bindings::ipps::normDiff_L2(*this->goal, *top.center) < top.radius)
					{
						this->end = vertex;
						
						return true;
					}
					
					::std::multiset< WorkspaceSphere >::iterator i = this->queue.begin();
					::std::multiset< WorkspaceSphere >::iterator j;
					
					while (i != this->queue.end())
					{
						if (::boost::numeric::bindings::ipps::normDiff_L2(*i->center, *top.center) < top.radius)
						{
							j = i;
							++i;
							this->queue.erase(j);
						}
						else
						{
							++i;
						}
					}
					
					for (::std::size_t i = 0; i < this->samples; ++i)
					{
						WorkspaceSphere sphere;
						
						sphere.parent = vertex;
						
						sample = this->rand();
						
						sphere.center = VectorPtr(new ::rl::math::Vector(3));
						::boost::numeric::bindings::ippm::saxpy(sample, top.radius, *top.center, *sphere.center);
						
						if (::boost::numeric::bindings::ipps::normDiff_L2(*this->start, *sphere.center) <= this->range)
						{
							if (!this->isCovered(*sphere.center))
							{
								sphere.radius = this->model->distance(*sphere.center);
								
								if (sphere.radius >= this->radius)
								{
									switch (this->greedy)
									{
									case GREEDY_DISTANCE:
										sphere.priority = ::boost::numeric::bindings::ipps::normDiff_L2(*this->goal, *sphere.center) - sphere.radius;
										break;
									case GREEDY_SPACE:
										sphere.priority = 1.0f / sphere.radius;
										break;
									default:
										break;
									}
									
									this->queue.insert(sphere);
								}
							}
						}
					}
				}
			}
			
			return false;
		}
		
		void
		WorkspaceSphereExplorer::getPath(WorkspaceSphereList& path)
		{
			Vertex i = this->end;
			
			while (i != this->begin)
			{
				path.push_front(this->graph[i].sphere);
				i = ::boost::source(*::boost::in_edges(i, this->graph).first, this->graph);
			}
			
			path.push_front(this->graph[i].sphere);
		}
		
		bool
		WorkspaceSphereExplorer::isCovered(const ::rl::math::Vector& point)
		{
			for (VertexIteratorPair i = ::boost::vertices(this->graph); i.first != i.second; ++i.first)
			{
				if (::boost::numeric::bindings::ipps::normDiff_L2(point, *this->graph[*i.first].sphere.center) < this->graph[*i.first].sphere.radius)
				{
					return true;
				}
			}
			
			return false;
		}
		
		void
		WorkspaceSphereExplorer::reset()
		{
			this->graph.clear();
			this->queue.clear();
			this->begin = NULL;
			this->end = NULL;
		}
		
		void
		WorkspaceSphereExplorer::seed(const ::boost::mt19937::result_type& value)
		{
			this->rand.engine().seed(value);
		}
	}
}
