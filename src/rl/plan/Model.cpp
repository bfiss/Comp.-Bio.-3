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

#include "Model.h"

namespace rl
{
	namespace plan
	{
		Model::Model() :
			kinematics(NULL),
			model(NULL),
			scene(NULL)
		{
		}
		
		Model::~Model()
		{
		}
		
		void
		Model::clip(::rl::math::Vector& q) const
		{
			this->kinematics->clip(q);
		}
		
		::rl::math::Real
		Model::distance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const
		{
			return this->kinematics->distance(q1, q2);
		}
		
		void
		Model::forwardForce(const ::rl::math::Vector& tau, ::rl::math::Vector& f) const
		{
			this->kinematics->forwardForce(tau, f);
		}
		
		const ::rl::math::Transform&
		Model::forwardPosition(const ::std::size_t& i) const
		{
			return this->kinematics->forwardPosition(i);
		}
		
		void
		Model::forwardVelocity(const ::rl::math::Vector& qdot, ::rl::math::Vector& xdot) const
		{
			this->kinematics->forwardVelocity(qdot, xdot);
		}
		
		::std::size_t
		Model::getBodies() const
		{
			return this->kinematics->getBodies();
		}
		
		::rl::sg::Body*
		Model::getBody(const ::std::size_t& i) const
		{
			return this->model->getBody(i);
		}
		
		::rl::math::Vector&
		Model::getCenter(const ::std::size_t& i) const
		{
			return this->model->getBody(i)->center;
		}
		
		::std::size_t
		Model::getDof() const
		{
			return this->kinematics->getDof();
		}
		
		const ::rl::math::Transform&
		Model::getFrame(const ::std::size_t& i) const
		{
			return this->kinematics->getFrame(i);
		}
		
		const ::rl::math::Matrix&
		Model::getJacobian() const
		{
			return this->kinematics->getJacobian();
		}
		
		::rl::math::Real
		Model::getMaximum(const ::std::size_t& i) const
		{
			return this->kinematics->getMaximum(i);
		}
		
		::rl::math::Real
		Model::getMinimum(const ::std::size_t& i) const
		{
			return this->kinematics->getMinimum(i);
		}
		
		::std::size_t
		Model::getOperationalDof() const
		{
			return this->kinematics->getOperationalDof();
		}
		
		void
		Model::inverseForce(const ::rl::math::Vector& f, ::rl::math::Vector& tau) const
		{
			this->kinematics->inverseForce(f, tau);
		}
		
		::rl::math::Real
		Model::inverseOfTransformedDistance(const ::rl::math::Real& d) const
		{
			return this->kinematics->inverseOfTransformedDistance(d);
		}
		
		void
		Model::inverseVelocity(const ::rl::math::Vector& xdot, ::rl::math::Vector& qdot) const
		{
			this->kinematics->inverseVelocity(xdot, qdot);
		}
		
		void
		Model::interpolate(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2, const ::rl::math::Real& alpha, ::rl::math::Vector& q) const
		{
			this->kinematics->interpolate(q1, q2, alpha, q);
		}
		
		bool
		Model::isSingular() const
		{
			return this->kinematics->isSingular();
		}
		
		bool
		Model::isValid(const ::rl::math::Vector& q) const
		{
			return this->kinematics->isValid(q);
		}
		
		::rl::math::Real
		Model::maxDistanceToRectangle(const ::rl::math::Vector& q, const ::rl::math::Vector& min, const ::rl::math::Vector& max) const
		{
			return this->kinematics->maxDistanceToRectangle(q, min, max);
		}
		
		::rl::math::Real
		Model::minDistanceToRectangle(const ::rl::math::Vector& q, const ::rl::math::Vector& min, const ::rl::math::Vector& max) const
		{
			return this->kinematics->minDistanceToRectangle(q, min, max);
		}
		
		::rl::math::Real
		Model::minDistanceToRectangle(const ::rl::math::Real& q, const ::rl::math::Real& min, const ::rl::math::Real& max, const ::std::size_t& cuttingDimension) const
		{
			return this->kinematics->minDistanceToRectangle(q, min, max, cuttingDimension);
		}
		
		::rl::math::Real
		Model::newDistance(const ::rl::math::Real& dist, const ::rl::math::Real& oldOff, const ::rl::math::Real& newOff, const int& cuttingDimension) const
		{
			return this->kinematics->newDistance(dist, oldOff, newOff, cuttingDimension);
		}
		
		void
		Model::setPosition(const ::rl::math::Vector& q)
		{
			assert(q.size() == this->kinematics->getDof());
			
			this->kinematics->setPosition(q);
		}
		
		void
		Model::step(const ::rl::math::Vector& q1, const ::rl::math::Vector& qdot, ::rl::math::Vector& q2) const
		{
			this->kinematics->step(q1, qdot, q2);
		}
		
		::rl::math::Real
		Model::transformedDistance(const ::rl::math::Real& d) const
		{
			return this->kinematics->transformedDistance(d);
		}
		
		::rl::math::Real
		Model::transformedDistance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const
		{
			return this->kinematics->transformedDistance(q1, q2);
		}
		
		void
		Model::updateFrames()
		{
			assert(this->model->getNumBodies() == this->kinematics->getBodies());
			
			this->kinematics->updateFrames();
			
			for (::std::size_t i = 0; i < this->model->getNumBodies(); ++i)
			{
				this->model->getBody(i)->setFrame(this->kinematics->getFrame(i));
			}
		}
		
		void
		Model::updateJacobian()
		{
			this->kinematics->updateJacobian();
		}
		
		void
		Model::updateJacobianInverse(const ::rl::math::Real& lambda)
		{
			this->kinematics->updateJacobianInverse(lambda);
		}
	}
}
