/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "multirobot_utils/StaticMotion.hpp"

#include "iostream"

namespace multirobot_utils
{
  namespace internal
  {

    //==============================================================================
    StaticMotion::StaticMotion(const Eigen::Isometry2d &tf)
    {
      const Eigen::Vector2d &p = tf.translation();
      Eigen::Rotation2Dd R{tf.rotation()};

#ifdef MULTIROBOT_UTILS__USING_FCL_0_6
      const auto x = fcl::Vector3d(p[0], p[1], 0.0);
      _tf =
          fcl::Translation3d(x) * fcl::AngleAxisd(R.angle(), fcl::Vector3d::UnitZ());
#else
      const auto x = fcl::Vec3f(p[0], p[1], 0.0);
      fcl::Quaternion3f q;
      q.fromAxisAngle(fcl::Vec3f(0.0, 0.0, 1.0), R.angle());

      _tf.setTransform(q, x);
#endif
    }

    //==============================================================================
    bool StaticMotion::integrate(double /*dt*/) const
    {
      // Do nothing
      return true;
    }

//==============================================================================
#ifdef MULTIROBOT_UTILS__USING_FCL_0_6
    double StaticMotion::computeMotionBound(
        const fcl::BVMotionBoundVisitor<double> &) const
#else
    double StaticMotion::computeMotionBound(
        const fcl::BVMotionBoundVisitor &) const
#endif
    {
      // TODO(MXG): Investigate the legitimacy of this implementation. Make sure
      // that this function truly should always return 0.
      return 0;
    }

//==============================================================================
#ifdef MULTIROBOT_UTILS__USING_FCL_0_6
    double StaticMotion::computeMotionBound(
        const fcl::TriangleMotionBoundVisitor<double> &) const
#else
    double StaticMotion::computeMotionBound(
        const fcl::TriangleMotionBoundVisitor &) const
#endif
    {
      std::cout << " ----- OH NO, WE'RE USING StaticMotion::computeMotionBound(TriangleMotionBoundVisitor)!! ----- "
                << std::endl;
      // *INDENT-OFF*
      throw std::runtime_error(
          "unimplemented function: StaticMotion::computeMotionBound(TriangleMotionBoundVisitor)");
      // *INDENT-ON*
    }

//==============================================================================
#ifdef MULTIROBOT_UTILS__USING_FCL_0_6
    void StaticMotion::getCurrentTransform(fcl::Transform3d &tf) const
#else
    void StaticMotion::getCurrentTransform(fcl::Transform3f &tf) const
#endif
    {
      tf = _tf;
    }

//==============================================================================
#ifdef MULTIROBOT_UTILS__USING_FCL_0_6
    void StaticMotion::getTaylorModel(
        fcl::TMatrix3<double> &,
        fcl::TVector3<double> &) const
#else
    void StaticMotion::getTaylorModel(fcl::TMatrix3 &, fcl::TVector3 &) const
#endif
    {
      std::cout << " ----- OH NO, WE'RE USING StaticMotion::getTaylorModel()!! ----- "
                << std::endl;
      // *INDENT-OFF*
      throw std::runtime_error(
          "unimplemented function: StaticMotion::getTaylorModel()");
      // *INDENT-ON*
    }

  } // namespace internal
} // namespace multirobot_utils
