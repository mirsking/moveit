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

#ifndef MOVEIT_COLLISION_DETECTION_PHYSX_COLLISION_COMMON_
#define MOVEIT_COLLISION_DETECTION_PHYSX_COLLISION_COMMON_

#include <moveit/collision_detection/world.h>
#include <moveit/collision_detection/collision_world.h>
#include <moveit/macros/class_forward.h>
#include <set>

namespace collision_detection
{

MOVEIT_CLASS_FORWARD(CollisionGeometryData);

struct CollisionGeometryData
{
  CollisionGeometryData(const robot_model::LinkModel *link, int index)
    : type(BodyTypes::ROBOT_LINK)
    , shape_index(index)
  {
    ptr.link = link;
  }

  CollisionGeometryData(const robot_state::AttachedBody *ab, int index)
    : type(BodyTypes::ROBOT_ATTACHED)
    , shape_index(index)
  {
    ptr.ab = ab;
  }

  CollisionGeometryData(const World::Object *obj, int index)
    : type(BodyTypes::WORLD_OBJECT)
    , shape_index(index)
  {
    ptr.obj = obj;
  }

  const std::string& getID() const
  {
    switch (type)
    {
    case BodyTypes::ROBOT_LINK:
      return ptr.link->getName();
    case BodyTypes::ROBOT_ATTACHED:
      return ptr.ab->getName();
    default:
      break;
    }
    return ptr.obj->id_;
  }

  std::string getTypeString() const
  {
    switch (type)
    {
    case BodyTypes::ROBOT_LINK:
      return "Robot link";
    case BodyTypes::ROBOT_ATTACHED:
      return "Robot attached";
    default:
      break;
    }
    return "Object";
  }

  /** \brief Check if two CollisionGeometryData objects point to the same source object */
  bool sameObject(const CollisionGeometryData &other) const
  {
    return type == other.type && ptr.raw == other.ptr.raw;
  }

  BodyType type;
  int shape_index;
  union
  {
    const robot_model::LinkModel    *link;
    const robot_state::AttachedBody *ab;
    const World::Object             *obj;
    const void                      *raw;
  } ptr;
};

struct CollisionData
{
  CollisionData() : req_(NULL), active_components_only_(NULL), res_(NULL), acm_(NULL), done_(false)
  {
  }

  CollisionData(const CollisionRequest *req, CollisionResult *res,
                const AllowedCollisionMatrix *acm) : req_(req), active_components_only_(NULL), res_(res), acm_(acm), done_(false)
  {
  }

  ~CollisionData()
  {
  }

  /// Compute \e active_components_only_ based on \e req_
  // void enableGroup(const robot_model::RobotModelConstPtr &kmodel);

  /// The collision request passed by the user
  const CollisionRequest       *req_;

  /// If the collision request includes a group name, this set contains the pointers to the link models that are considered for collision;
  /// If the pointer is NULL, all collisions are considered.
  const std::set<const robot_model::LinkModel*>
                               *active_components_only_;

  /// The user specified response location
  CollisionResult              *res_;

  /// The user specified collision matrix (may be NULL)
  const AllowedCollisionMatrix *acm_;

  /// Flag indicating whether collision checking is complete
  bool                          done_;
};
}

#endif
