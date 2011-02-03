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

#include <rl/sg/Body.h>
#include <rl/sg/SimpleScene.h>

#include "SimpleModel.h"

namespace rl
{
	namespace plan
	{
		SimpleModel::SimpleModel() :
			Model(),
			body(0),
			queries(0)
		{
		}
		
		SimpleModel::~SimpleModel()
		{
		}
		
		::std::size_t
		SimpleModel::getCollidingBody() const
		{
			return this->body;
		}
		
		::std::size_t
		SimpleModel::getQueries() const
		{
			return this->queries;
		}
		
		bool
		SimpleModel::isColliding()
		{
			++this->queries;
			
			for (::std::size_t i = 0; i < this->model->getNumBodies(); ++i)
			{
				if (this->kinematics->isColliding(i))
				{
					for (::rl::sg::Scene::Iterator j = this->scene->begin(); j != this->scene->end(); ++j)
					{
						if (this->model != *j)
						{
							for (::rl::sg::Model::Iterator k = (*j)->begin(); k != (*j)->end(); ++k)
							{
								if (dynamic_cast< ::rl::sg::SimpleScene* >(this->scene)->areColliding(this->model->getBody(i), *k))
								{
									this->body = i;
									return true;
								}
							}
						}
					}
				}
				
				for (::std::size_t j = 0; j < i; ++j)
				{
					if (this->kinematics->areColliding(i, j))
					{
						if (dynamic_cast< ::rl::sg::SimpleScene* >(this->scene)->areColliding(this->model->getBody(i), this->model->getBody(j)))
						{
							this->body = i;
							return true;
						}
					}
				}
			}
			
			this->body = this->getBodies();
			return false;
		}
	}
}
