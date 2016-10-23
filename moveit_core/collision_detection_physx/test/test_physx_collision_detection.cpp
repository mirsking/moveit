/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection_physx/collision_world_physx.h>
#include <moveit/collision_detection_physx/collision_robot_physx.h>
#include <moveit_resources/config.h>

#include <urdf_parser/urdf_parser.h>
#include <geometric_shapes/shape_operations.h>

#include <gtest/gtest.h>
#include <sstream>
#include <algorithm>
#include <ctype.h>
#include <fstream>

#include <boost/filesystem.hpp>

typedef collision_detection::CollisionWorldPhysx DefaultCWorldType;
typedef collision_detection::CollisionRobotPhysx DefaultCRobotType;

class FclCollisionDetectionTester : public testing::Test
{

protected:

  virtual void SetUp()
  {
    boost::filesystem::path res_path(MOVEIT_TEST_RESOURCES_DIR);
    std::string urdf_file = (res_path / "pr2_description/urdf/robot.xml").string();
    std::string srdf_file = (res_path / "pr2_description/srdf/robot.xml").string();
    kinect_dae_resource_ = "package://moveit_resources/pr2_description/urdf/meshes/sensors/kinect_v0/kinect.dae";

    srdf_model_.reset(new srdf::Model());
    std::string xml_string;
    std::fstream xml_file(urdf_file.c_str(), std::fstream::in);

    if (xml_file.is_open())
    {
      while ( xml_file.good() )
      {
        std::string line;
        std::getline( xml_file, line);
        xml_string += (line + "\n");
      }
      xml_file.close();
      urdf_model_ = urdf::parseURDF(xml_string);
      urdf_ok_ = urdf_model_;
    }
    else
    {
      EXPECT_EQ("FAILED TO OPEN FILE", urdf_file);
      urdf_ok_ = false;
    }
    srdf_ok_ = srdf_model_->initFile(*urdf_model_, srdf_file);

    kmodel_.reset(new robot_model::RobotModel(urdf_model_, srdf_model_));

    acm_.reset(new collision_detection::AllowedCollisionMatrix(kmodel_->getLinkModelNames(), true));

    crobot_.reset(new DefaultCRobotType(kmodel_));
    cworld_.reset(new DefaultCWorldType());
  }

  virtual void TearDown()
  {

  }

protected:

  bool urdf_ok_;
  bool srdf_ok_;

  boost::shared_ptr<urdf::ModelInterface>  urdf_model_;
  boost::shared_ptr<srdf::Model>           srdf_model_;

  robot_model::RobotModelPtr               kmodel_;

  collision_detection::CollisionRobotPtr   crobot_;
  collision_detection::CollisionWorldPtr   cworld_;

  collision_detection::AllowedCollisionMatrixPtr acm_;

  std::string kinect_dae_resource_;
};


TEST_F(FclCollisionDetectionTester, InitOK)
{
  ASSERT_TRUE(urdf_ok_);
  ASSERT_TRUE(srdf_ok_);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
