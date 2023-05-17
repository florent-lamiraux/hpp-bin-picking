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

#include <hpp/util/debug.hh>
#include <hpp/bin-picking/effector.hh>

#include <hpp/pinocchio/gripper.hh>
#include <hpp/pinocchio/joint-collection.hh>

#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/handle.hh>

namespace hpp{
namespace bin_picking{

EffectorPtr_t Effector::create(const DevicePtr_t& robot,
    const pinocchio::GripperPtr_t& gripper, const Configuration_t& q,
    const RelativeMotion::matrix_type& relativeMotion)
{
  return  EffectorPtr_t(new Effector(robot, gripper, q, relativeMotion));
}

Effector::Effector(const DevicePtr_t& robot,
    const pinocchio::GripperPtr_t& gripper, const Configuration_t& q,
    const RelativeMotion::matrix_type& relativeMotion) :
  robot_(robot),
  model_(robot->model()), geomModel_(robot->geomModel()), data_(model_),
  geomData_(geomModel_), gripper_(gripper)
{
  buildEffector(q, relativeMotion);
}

void Effector::updateData()
{
  data_ = pinocchio::Data(model_);
  geomData_ = ::pinocchio::GeometryData(geomModel_);
}

std::ostream& Effector::print(std::ostream& os) const
{
  return os;
}

void Effector::buildEffector(const Configuration_t& q,
                             const RelativeMotion::matrix_type& relativeMotion)
{
  JointPtr_t gripperJoint(gripper_.lock()->joint());
  // Compute forward kinematics
  ::pinocchio::forwardKinematics(model_, data_, q);
  ::pinocchio::updateFramePlacements(model_, data_);
  ::pinocchio::updateGeometryPlacements(model_, data_, geomModel_, geomData_);
  // Recover list of joints that move with the gripper
  assert(model_.existJointName(gripperJoint->name()));
  JointIndex j1(model_.getJointId(gripperJoint->name()));
  // Get frame pose corresponding to the gripper
  assert(model_.existFrame(gripper_.lock()->name()));
  FrameIndex gripperFrameId(model_.getFrameId(gripper_.lock()->name()));
  SE3 gripperPoseInv(data_.oMf[gripperFrameId].inverse());
  for (JointIndex j2=0; j2<(JointIndex)model_.njoints; ++j2){
    if (j1 == j2 || relativeMotion(j1, j2) == core::RelativeMotion::Constrained)
      {
      // Gather geometry objects attached to the joint
      for (auto gId : geomData_.innerObjects[j2]){
        CollisionGeometryPtr_t geometry
          (geomModel_.geometryObjects[gId].geometry);
        innerObjects_.push_back(std::make_pair(geometry,
            gripperPoseInv*geomData_.oMg[gId]));
        innerObjNames_.push_back(geomModel_.geometryObjects[gId].name);
      }
    }
  }
}

void Effector::addObstacle(const std::string& name, double securityMargin)
{
  if (!geomModel_.existGeometryName(name)){
    throw std::logic_error(std::string("No geometry with name ") + name);
  }
  GeomIndex gid(geomModel_.getGeometryId(name));
  obstacles_.push_back(std::make_pair(gid, securityMargin));
  // Add collision test for each inner object
  hpp::fcl::CollisionRequest request;
  request.security_margin = securityMargin;
  for (auto inner : innerObjects_){
    computeCollisions_.push_back(hpp::fcl::ComputeCollision(inner.first.get(),
        geomModel_.geometryObjects[gid].geometry.get()));
    collisionRequests_.push_back(request);
  }
}

bool Effector::collisionTest(const HandlePtr_t &handle,
    const Configuration_t& q, std::string& report)
{
  // Compute forward kinematics;
  ::pinocchio::forwardKinematics(model_, data_, q);
  ::pinocchio::updateFramePlacements(model_, data_);
  ::pinocchio::updateGeometryPlacements(model_, data_, geomModel_, geomData_);

  double clearance(gripper_.lock()->clearance() + handle->clearance());
  matrix3_t I3(matrix3_t::Identity());
  vector3_t trans; trans << -clearance, 0, 0;
  SE3 pgOffset(I3, trans);
  fcl::CollisionResult result;
  std::size_t i=0;
  report = std::string("collision between ");
  bool res(false);
  assert(model_.existFrame(handle->name()));
  FrameIndex hid(model_.getFrameId(handle->name()));
  SE3 gripperPose(data_.oMf[hid]);
  // Test grasp
  for(auto obstacle : obstacles_) {
    // recover pose of obstacle
    const SE3& obstaclePose(geomData_.oMg[obstacle.first]);
    const std::string& obstName(geomModel_.geometryObjects[obstacle.first].
                                name);
    std::size_t iObj=0;
    for(auto inner : innerObjects_) {
      SE3 innerPose(gripperPose*inner.second);
      std::size_t n(computeCollisions_[i](
          fcl::Transform3f(innerPose.rotation(), innerPose.translation()),
          fcl::Transform3f(obstaclePose.rotation(), obstaclePose.translation()),
          collisionRequests_[i], result)
          );
      if (n > 0){
        report += innerObjNames_[iObj];
        report += std::string(" and ");
        report += obstName;
        report += std::string(", ");
        res = true;
      }
      ++i; ++iObj;
    }
  }
  if (res) {
    report += std::string("in grasp");
    return true;
  }
  // Test pregrasp
  gripperPose = gripperPose * pgOffset;
  i = 0;
  for(auto obstacle : obstacles_) {
    // recover pose of obstacle
    const SE3& obstaclePose(geomData_.oMg[obstacle.first]);
    const std::string& obstName(geomModel_.geometryObjects[obstacle.first].
                                name);
    std::size_t iObj=0;
    for(auto inner : innerObjects_) {
      SE3 innerPose(gripperPose*inner.second);
      std::size_t n(computeCollisions_[i](
          fcl::Transform3f(innerPose.rotation(), innerPose.translation()),
          fcl::Transform3f(obstaclePose.rotation(), obstaclePose.translation()),
          collisionRequests_[i], result)
          );
      if (n > 0){
        report += innerObjNames_[iObj];
        report += std::string(" and ");
        report += obstName;
        report += std::string(", ");
        res = true;
      }
      ++i; ++iObj;
    }
  }
  if (res) {
    report += std::string("in pregrasp");
    return true;
  }
  report = std::string("");
  return false;
}

} // namespace bin_picking
} // namespace hpp
