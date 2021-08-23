/*
 * Copyright 2021 Open Source Robotics Foundation
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

#include <string>
#include <sstream>
#include <fstream>
#include <gtest/gtest.h>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include "sdf/Element.hh"
#include "sdf/Error.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"
#include "sdf/parser.hh"
#include "sdf/Filesystem.hh"
#include "test_config.h"
#include "test_utils.hh"

//////////////////////////////////////////////////
TEST(Pose1_9, ModelPose)
{
  using Pose = ignition::math::Pose3d;

  const std::string testFile = sdf::testing::TestFile(
      "sdf", "pose_snapping_rotation.sdf");
  const double pi = 3.14159265358979323846;

  // Load the SDF file
  sdf::Root root;
  auto errors = root.Load(testFile);
  ASSERT_TRUE(errors.empty()) << errors;
  EXPECT_EQ(SDF_PROTOCOL_VERSION, root.Version());

  const sdf::World *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  const sdf::Model *model = world->ModelByIndex(0);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_snap_to_zero_radians", model->Name());
  EXPECT_EQ(Pose::Zero, model->RawPose());

  model = world->ModelByIndex(1);
  ASSERT_NE(nullptr, model);
  ASSERT_EQ("model_snap_to_non_zero_radians", model->Name());
  EXPECT_EQ(Pose(0, 0, 0, pi / 4, -pi / 4, pi), model->RawPose());
}
