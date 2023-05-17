// Copyright (c) 2023, LAAS-CNRS
// Authors: Florent Lamiraux
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#include <../corba/bin-picking.impl.hh>
#include <../corba/bin-picking.hh>

#include <hpp/pinocchio/gripper.hh>
#include <hpp/pinocchio/liegroup-space.hh>

#include <hpp/corbaserver/conversions.hh>

#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/graph.hh>
#include <hpp/manipulation/problem-solver.hh>

#include <hpp/bin-picking/effector.hh>
#include <hpp/bin-picking/handle.hh>

namespace hpp {
namespace bin_picking {
namespace impl {

using corbaServer::floatSeqToConfig;
using corbaServer::floatSeqToVector;
using corbaServer::vectorToFloatSeq;

using manipulation::graph::GraphPtr_t;
typedef manipulation::graph::Edge Transition;
typedef manipulation::graph::EdgePtr_t TransitionPtr_t;

DevicePtr_t BinPicking::getRobotOrThrow()
{
  DevicePtr_t robot(problemSolver()->robot());
  if (!robot){
    throw std::runtime_error("No robot has been loaded.");
  }
  return robot;
}

EffectorPtr_t BinPicking::getEffectorOrThrow(const char* name)
{
  std::map <std::string, EffectorPtr_t>::iterator it(effectors_.find
                                                     (std::string(name)));
  if(it == effectors_.end()) {
    std::ostringstream os;
    os << "No effector with name " << name;
    throw std::logic_error(os.str().c_str());
  }
  return it->second;
}

BinPicking::BinPicking() : server_(0x0) {}

hpp::manipulation::ProblemSolverPtr_t BinPicking::problemSolver()
{
  return server_->problemSolver();
}

void BinPicking::createEffector(const char* name, const char* gripperName,
                                const floatSeq& q, CORBA::Long transitionId)
{
  try{
    DevicePtr_t robot(getRobotOrThrow());
    GripperPtr_t gripper(robot->grippers.get(std::string(gripperName)));

    GraphPtr_t graph(problemSolver()->constraintGraph());
    TransitionPtr_t transition(HPP_DYNAMIC_PTR_CAST(Transition,
        graph->get(transitionId).lock()));
    if(!transition){
      std::ostringstream os;
      os << "Index " << transitionId << " does not correspond to an edge in the"
        "constraint graph.";
      throw std::logic_error(os.str().c_str());
    }
    EffectorPtr_t effector(Effector::create(robot, gripper,
        corbaServer::floatSeqToVector(q), transition->relativeMotion()));
    effectors_[name] = effector;
  } catch (const std::exception& exc){
    throw Error(exc.what());
  }
}

void BinPicking::addObstacleToEffector(const char* effectorName,
    const char* obstacleName, double securityMargin)
{
  try{
    EffectorPtr_t effector(getEffectorOrThrow(effectorName));
    effector->addObstacle(std::string(obstacleName), securityMargin);
  } catch(const std::exception &exc){
    throw Error(exc.what());
  }
}

CORBA::Boolean BinPicking::collisionTest(const char* name,
      const Transform_ gripperPose, const ::hpp::floatSeq& q,
      CORBA::String_out report)
{
  try{
    SE3 gp(corbaServer::toTransform3f(gripperPose));
    EffectorPtr_t effector(getEffectorOrThrow(name));
    std::string collisionReport;
    bool res(
        effector->collisionTest(gp, corbaServer::floatSeqToVector(q),
                                collisionReport)
        );
    report = CORBA::string_dup(collisionReport.c_str());
    return res;
  } catch(const std::exception &exc){
    throw Error(exc.what());
  }
}

void BinPicking::discretizeHandle(const char* name, CORBA::Long nbHandles)
{
  try {
    DevicePtr_t robot(getRobotOrThrow());
    HandlePtr_t handle(robot->handles.get(std::string(name)));
    std::vector<HandlePtr_t> handles(::hpp::bin_picking::discretizeHandle(
                                     handle, nbHandles));
    for (HandlePtr_t h : handles) {
      robot->handles.add(h->name(), h);
    }
  } catch(const std::exception &exc) {
    throw Error(exc.what());
  }
}

} // namespace impl
} // namespace bin_picking
} // namespace hpp
