// Copyright (c) 2023 CNRS
// Author: Florent Lamiraux
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

#include <../corba/bin-picking.hh>
#include <../corba/bin-picking.impl.hh>

#include <hpp/corbaserver/server.hh>

#include <hpp/manipulation/problem-solver.hh>

namespace hpp{
namespace bin_picking{

typedef hpp::manipulation::ProblemSolver ProblemSolver;
typedef hpp::manipulation::ProblemSolverPtr_t ProblemSolverPtr_t;

Server::Server(corbaServer::Server* parent) : corbaServer::ServerPlugin(parent),
                                              binPickingImpl_(0x0)
{
}

Server::~Server()
{
  if (binPickingImpl_) delete binPickingImpl_;
}

std::string Server::name() const { return "bin_picking"; }

/// Start corba server
void Server::startCorbaServer(const std::string& contextId,
                              const std::string& contextKind) {
  initializeTplServer(binPickingImpl_, contextId, contextKind, name(), "bin_picking");

  binPickingImpl_->implementation().setServer(this);
}

ProblemSolverPtr_t Server::problemSolver() {
  ProblemSolverPtr_t ret(dynamic_cast<hpp::manipulation::ProblemSolverPtr_t>
                         (problemSolverMap_->selected()));
  if (!ret){
    std::ostringstream os;
    os << "Selected problem solver \"" << problemSolverMap_->selectedName()
       << "\" is not of type hpp::manipulation::ProblemSolver";
    throw std::logic_error(os.str().c_str());
  }
  return ret;
}

::CORBA::Object_ptr Server::servant(const std::string& name) const {
  if (name == "bin_picking") return binPickingImpl_->implementation()._this();
  throw std::invalid_argument("No servant " + name);
}

} // namespace bin_picking
} // name hpp

HPP_CORBASERVER_DEFINE_PLUGIN(hpp::bin_picking::Server)
