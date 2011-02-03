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

#ifndef _RL_PLAN_MODEL_H_
#define _RL_PLAN_MODEL_H_

#include <rl/kin/Kinematics.h>
#include <rl/math/Transform.h>
#include <rl/math/Vector.h>
#include <rl/sg/Model.h>
#include <rl/sg/Scene.h>

namespace rl
{
	namespace plan
	{
		class Model
		{
		public:
			Model();
			
			virtual ~Model();
			
			virtual void clip(::rl::math::Vector& q) const;
			
			virtual ::rl::math::Real distance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const;
			
			virtual void forwardForce(const ::rl::math::Vector& tau, ::rl::math::Vector& f) const;
			
			virtual const ::rl::math::Transform& forwardPosition(const ::std::size_t& i = 0) const;
			
			virtual void forwardVelocity(const ::rl::math::Vector& qdot, ::rl::math::Vector& xdot) const;
			
			virtual ::rl::sg::Body* getBody(const ::std::size_t& i) const;
			
			virtual ::std::size_t getBodies() const;
			
			virtual ::rl::math::Vector& getCenter(const ::std::size_t& i) const;
			
			virtual ::std::size_t getDof() const;
			
			virtual const ::rl::math::Transform& getFrame(const ::std::size_t& i) const;
			
			virtual const ::rl::math::Matrix& getJacobian() const;
			
			virtual ::rl::math::Real getMaximum(const ::std::size_t& i) const;
			
			virtual ::rl::math::Real getMinimum(const ::std::size_t& i) const;
			
			virtual ::std::size_t getOperationalDof() const;
			
			virtual void inverseForce(const ::rl::math::Vector& f, ::rl::math::Vector& tau) const;
			
			virtual ::rl::math::Real inverseOfTransformedDistance(const ::rl::math::Real& d) const;
			
			virtual void inverseVelocity(const ::rl::math::Vector& tdot, ::rl::math::Vector& qdot) const;
			
			virtual void interpolate(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2, const ::rl::math::Real& alpha, ::rl::math::Vector& q) const;
			
			virtual bool isSingular() const;
			
			virtual bool isValid(const ::rl::math::Vector& q) const;
			
			virtual ::rl::math::Real maxDistanceToRectangle(const ::rl::math::Vector& q, const ::rl::math::Vector& min, const ::rl::math::Vector& max) const;
			
			virtual ::rl::math::Real minDistanceToRectangle(const ::rl::math::Vector& q, const ::rl::math::Vector& min, const ::rl::math::Vector& max) const;
			
			virtual ::rl::math::Real minDistanceToRectangle(const ::rl::math::Real& q, const ::rl::math::Real& min, const ::rl::math::Real& max, const ::std::size_t& cuttingDimension) const;
			
			virtual ::rl::math::Real newDistance(const ::rl::math::Real& dist, const ::rl::math::Real& oldOff, const ::rl::math::Real& newOff, const int& cuttingDimension) const;
			
			virtual void setPosition(const ::rl::math::Vector& q);
			
			virtual void step(const ::rl::math::Vector& q1, const ::rl::math::Vector& qdot, ::rl::math::Vector& q2) const;
			
			virtual ::rl::math::Real transformedDistance(const ::rl::math::Real& d) const;
			
			virtual ::rl::math::Real transformedDistance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const;
			
			virtual void updateFrames();
			
			virtual void updateJacobian();
			
			virtual void updateJacobianInverse(const ::rl::math::Real& lambda = 0.0f);
			
			::rl::kin::Kinematics* kinematics;
			
			::rl::sg::Model* model;
			
			::rl::sg::Scene* scene;
			
		protected:
			
		private:
			
		};
	}
}

#endif // _RL_PLAN_MODEL_H_
