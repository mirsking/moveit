/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Mirs King */

#include <moveit/collision_detection_physx/collision_robot_physx.h>

collision_detection::CollisionRobotPhysx::CollisionRobotPhysx(const robot_model::RobotModelConstPtr &model, double padding, double scale)
  : CollisionRobot(model, padding, scale)
{
}

collision_detection::CollisionRobotPhysx::CollisionRobotPhysx(const CollisionRobotPhysx &other) : CollisionRobot(other)
{
}

void collision_detection::CollisionRobotPhysx::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state) const
{
  checkSelfCollisionHelper(req, res, state, NULL);
}

void collision_detection::CollisionRobotPhysx::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                                                const AllowedCollisionMatrix &acm) const
{
  checkSelfCollisionHelper(req, res, state, &acm);
}

void collision_detection::CollisionRobotPhysx::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2) const
{
  logError("Physx continuous collision checking not yet implemented");
}

void collision_detection::CollisionRobotPhysx::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2, const AllowedCollisionMatrix &acm) const
{
  logError("Physx continuous collision checking not yet implemented");
}

void collision_detection::CollisionRobotPhysx::checkSelfCollisionHelper(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                                                      const AllowedCollisionMatrix *acm) const
{
    /// write code to check self collision
}

void collision_detection::CollisionRobotPhysx::checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                                                 const CollisionRobot &other_robot, const robot_state::RobotState &other_state) const
{
  checkOtherCollisionHelper(req, res, state, other_robot, other_state, NULL);
}

void collision_detection::CollisionRobotPhysx::checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                                                 const CollisionRobot &other_robot, const robot_state::RobotState &other_state,
                                                                 const AllowedCollisionMatrix &acm) const
{
  checkOtherCollisionHelper(req, res, state, other_robot, other_state, &acm);
}

void collision_detection::CollisionRobotPhysx::checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2,
                                                                 const CollisionRobot &other_robot, const robot_state::RobotState &other_state1, const robot_state::RobotState &other_state2) const
{
  logError("Physx continuous collision checking not yet implemented");
}

void collision_detection::CollisionRobotPhysx::checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2,
                                                                 const CollisionRobot &other_robot, const robot_state::RobotState &other_state1, const robot_state::RobotState &other_state2,
                                                                 const AllowedCollisionMatrix &acm) const
{
  logError("Physx continuous collision checking not yet implemented");
}

void collision_detection::CollisionRobotPhysx::checkOtherCollisionHelper(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                                                       const CollisionRobot &other_robot, const robot_state::RobotState &other_state,
                                                                       const AllowedCollisionMatrix *acm) const
{
    /// write code to check other collision
}

double collision_detection::CollisionRobotPhysx::distanceSelf(const robot_state::RobotState &state) const
{
  return distanceSelfHelper(state, NULL);
}

double collision_detection::CollisionRobotPhysx::distanceSelf(const robot_state::RobotState &state,
                                                            const AllowedCollisionMatrix &acm) const
{
  return distanceSelfHelper(state, &acm);
}

double collision_detection::CollisionRobotPhysx::distanceSelfHelper(const robot_state::RobotState &state,
                                                                  const AllowedCollisionMatrix *acm) const
{
    return 0.0;
}

double collision_detection::CollisionRobotPhysx::distanceOther(const robot_state::RobotState &state,
                                                             const CollisionRobot &other_robot,
                                                             const robot_state::RobotState &other_state) const
{
  return distanceOtherHelper(state, other_robot, other_state, NULL);
}

double collision_detection::CollisionRobotPhysx::distanceOther(const robot_state::RobotState &state,
                                                             const CollisionRobot &other_robot,
                                                             const robot_state::RobotState &other_state,
                                                             const AllowedCollisionMatrix &acm) const
{
  return distanceOtherHelper(state, other_robot, other_state, &acm);
}

double collision_detection::CollisionRobotPhysx::distanceOtherHelper(const robot_state::RobotState &state,
                                                                   const CollisionRobot &other_robot,
                                                                   const robot_state::RobotState &other_state,
                                                                   const AllowedCollisionMatrix *acm) const
{
    return 0.0;
}
