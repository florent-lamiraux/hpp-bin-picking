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

#ifndef HPP_BIN_PICKING_EFFECTOR_IMPL_HH
#define HPP_BIN_PICKING_EFFECTOR_IMPL_HH

#include <hpp/bin-picking/fwd.hh>

#include <hpp/fcl/collision.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

#include <hpp/core/relative-motion.hh>

namespace hpp{
namespace bin_picking{

typedef core::RelativeMotion RelativeMotion;

/// Set of objects rigidly linked together that holds a \link hpp::pinocchio::Gripper gripper \endlink.
///
/// This class is particularly useful to test the collision of a grasp
/// without solving the nonlinear constraint.
class Effector{
public:
  /// Create instance and return share pointer
  /// \param robot the robot that owns the gripper,
  /// \param gripper the gripper hold by the effector
  /// \param q a configuration that satisfies the constraints of the edge
  ///        leading to the grasp. This configuration is used to compute
  ///        the relative pose of rigidly fixed objects between each other,
  /// \param relative motion between joints to detect geometries that
  ///        are rigidly linked,
  static EffectorPtr_t create(const DevicePtr_t& robot,
     const pinocchio::GripperPtr_t& gripper,
     const Configuration_t& q,
     const RelativeMotion::matrix_type& relativeMotion);
  /// Update data and geomData members after the model has been changed
  void updateData();
  /// Add an obstacle to the effector
  /// \param name name of the obstacle in pinocchio::GeometryModel,
  /// \param securityMargin security margin for collision tests.
  void addObstacle(const std::string& name, double securityMargin);
  /// Test collision for a given handle in a given configuration
  /// \param handle handle to align the gripper with
  /// \param q configuration used to compute the pose of all geometries in the
  ///          world frame.
  /// \param clearance offset applied along -x axis to test pregrasp
  ///        configurations,
  /// \retval report validation report with information in case of collision
  /// \return true if there is a collision.
  /// \note pregrasp pose of the effector is also tested.
  bool collisionTest(const HandlePtr_t &handle, const Configuration_t& q,
                     std::string& report);
  /// Print object in a stream;
  virtual std::ostream& print(std::ostream& os) const;
protected:
  /// Constructor
  /// \param robot the robot that owns the gripper,
  /// \param gripper the gripper hold by the effector
  /// \param q a configuration that satisfies the constraints of the edge
  ///        leading to the grasp. This configuration is used to compute
  ///        the relative pose of rigidly fixed objects between each other,
  /// \param relative motion between joints to detect geometries that
  ///        are rigidly linked,
  Effector(const DevicePtr_t& robot, const pinocchio::GripperPtr_t& gripper,
     const Configuration_t& q,
     const RelativeMotion::matrix_type& relativeMotion);

private:
  // Geometry and pose with respect to gripper frame
  typedef std::pair<fcl::CollisionGeometryPtr_t, SE3> CollisionObject;
  typedef std::vector<CollisionObject> CollisionObjects_t;
  // Geometry with security margin
  typedef std::pair<GeomIndex, double> Obstacle_t;
  typedef std::vector<Obstacle_t> Obstacles_t;
  typedef std::vector<hpp::fcl::ComputeCollision> ComputeCollisions_t;
  typedef std::vector<hpp::fcl::CollisionRequest> CollisionRequests_t;
  // Recover objects that are rigidly linked to the effector and compute
  // their pose with respect to the gripper frame.
  void buildEffector(const Configuration_t& q,
                     const RelativeMotion::matrix_type& relativeMotion);
  DevicePtr_t robot_;
  const pinocchio::Model& model_;
  const ::pinocchio::GeometryModel& geomModel_;
  pinocchio::Data data_;
  ::pinocchio::GeometryData geomData_;
  GripperWkPtr_t gripper_;
  // geometries composing the effector
  CollisionObjects_t innerObjects_;
  // names of the geometries composing the effector
  std::vector<std::string> innerObjNames_;
  Obstacles_t obstacles_;
  ComputeCollisions_t computeCollisions_;
  CollisionRequests_t collisionRequests_;
}; // class Effector

inline std::ostream& operator<<(std::ostream& os,
                                const Effector& instance) {
  return instance.print(os);
}

} // namespace bin_picking
} // namespace hpp
#endif //HPP_BIN_PICKING_EFFECTOR_IMPL_HH
