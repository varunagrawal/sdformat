/*
 * Copyright 2017 Open Source Robotics Foundation
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
#include <utility>
#include "Utils.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE {

/////////////////////////////////////////////////
bool isReservedName(const std::string &_name)
{
  const std::size_t size = _name.size();
  return _name == "world" ||
      (size >= 4 &&
       _name.compare(0, 2, "__") == 0 &&
       _name.compare(size-2, 2, "__") == 0);
}

/////////////////////////////////////////////////
bool loadName(sdf::ElementPtr _sdf, std::string &_name)
{
  // Read the name
  std::pair<std::string, bool> namePair = _sdf->Get<std::string>("name", "");

  _name = namePair.first;
  return namePair.second;
}

/////////////////////////////////////////////////
bool loadPose(sdf::ElementPtr _sdf, ignition::math::Pose3d &_pose,
              std::string &_frame)
{
  sdf::ElementPtr sdf = _sdf;
  if (_sdf->GetName() != "pose")
  {
    if (_sdf->HasElement("pose"))
      sdf = _sdf->GetElement("pose");
    else
      return false;
  }

  // Default zero pose.
  ignition::math::Pose3d pose(
      ignition::math::Vector3d::Zero, ignition::math::Quaterniond::Identity);

  // Read the frame. An empty frame implies the parent frame. This is optional.
  std::pair<std::string, bool> framePair =
      sdf->Get<std::string>("relative_to", "");

  // Start checking for translation and rotation elements.
  sdf::ElementPtr translationPtr = sdf->GetElement("translation");
  sdf::ElementPtr rotationPtr = sdf->GetElement("rotation");
  
  // If both are not explicitly set, the pose is parsed using the value.
  if (!translationPtr->GetExplicitlySetInFile() &&
      !rotationPtr->GetExplicitlySetInFile())
  {
    std::pair<ignition::math::Pose3d, bool> posePair =
        sdf->Get<ignition::math::Pose3d>("", ignition::math::Pose3d::Zero);
    _pose = posePair.first;
    _frame = framePair.first;

    // In this scenario, return true or false based on pose element value.
    return posePair.second;
  }
 
  // Read the translation values.
  if (translationPtr->GetExplicitlySetInFile())
  {
    std::pair<ignition::math::Vector3d, bool> translationPair =
        translationPtr->Get<ignition::math::Vector3d>(
            "", ignition::math::Vector3d::Zero);
    if (translationPair.second)
    {
      std::cout << "found translation" << std::endl;
      pose.Set(translationPair.first, pose.Rot());
    }
  }

  // Read the rotation values.
  if (rotationPtr->GetExplicitlySetInFile())
  {
    std::pair<std::string, bool> typePair =
        rotationPtr->Get<std::string>("type", "");
    if (!typePair.second)
      return false;

    if (typePair.first == "rpy_degrees")
    {
      std::cout << "found rpy deg" << std::endl;
      std::pair<ignition::math::Vector3d, bool> rpyDegPair =
          rotationPtr->Get<ignition::math::Vector3d>(
              "", ignition::math::Vector3d::Zero);
      if (rpyDegPair.second)
        pose.Set(pose.Pos(), IGN_DTOR(rpyDegPair.first));
    }
    else if (typePair.first == "rpy_radians")
    {
      std::cout << "found rpy rad" << std::endl;
      std::pair<ignition::math::Vector3d, bool> rpyRadPair =
          rotationPtr->Get<ignition::math::Vector3d>(
              "", ignition::math::Vector3d::Zero);
      if (rpyRadPair.second)
        pose.Set(pose.Pos(), rpyRadPair.first);
    }
    else if (typePair.first == "q_wxyz")
    {
      std::cout << "found quat" << std::endl;
      std::pair<ignition::math::Quaterniond, bool> quatPair =
          rotationPtr->Get<ignition::math::Quaterniond>(
              "", ignition::math::Quaterniond::Identity);
      if (quatPair.second)
      {
        pose.Set(pose.Pos(), quatPair.first);
        std::cout << quatPair.first << std::endl;
        std::cout << "set quat" << std::endl;
      }
    }
    else
      return false;
  }

  _pose = pose;
  _frame = framePair.first;
  return true;
}
}
}

