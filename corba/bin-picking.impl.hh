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

#ifndef HPP_BIN_PICKING_CORBA_BIN_PICKING_IMPL_HH
#define HPP_BIN_PICKING_CORBA_BIN_PICKING_IMPL_HH

#include <hpp/bin-picking/fwd.hh>
#include <corba/bin-picking-idl.hh>
#include <hpp/manipulation/fwd.hh>

namespace hpp {
namespace bin_picking {
class Server;
namespace impl {

class BinPicking : public virtual POA_hpp::corbaserver::bin_picking::BinPicking
{
public:
  BinPicking();
  void setServer(Server* server) { server_ = server; }

  virtual void createEffector(const char* name, const char* gripperName,
                              const floatSeq& q, CORBA::Long transitionId);
  virtual void addObstacleToEffector(const char* effectorName,
      const char* obstacleName, double securityMargin);
  virtual ::CORBA::Boolean collisionTest(const char* name,
      const Transform_ gripperPose, const ::hpp::floatSeq& q,
      CORBA::String_out report);
private:
  EffectorPtr_t getEffectorOrThrow(const char* name);
  manipulation::ProblemSolverPtr_t problemSolver();
  DevicePtr_t getRobotOrThrow();
  Server* server_;
  std::map <std::string, EffectorPtr_t> effectors_;
}; // class BinPicking
} // namespace impl
} // namespace bin_picking
} // namespace hpp

#endif // HPP_BIN_PICKING_CORBA_BIN_PICKING_IMPL_HH
