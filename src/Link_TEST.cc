/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <gtest/gtest.h>
#include <ignition/math/Inertial.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include "sdf/Collision.hh"
#include "sdf/Light.hh"
#include "sdf/Link.hh"
#include "sdf/ParticleEmitter.hh"
#include "sdf/Sensor.hh"
#include "sdf/Visual.hh"

/////////////////////////////////////////////////
TEST(DOMLink, Construction)
{
  sdf::Link link;
  EXPECT_EQ(nullptr, link.Element());
  EXPECT_TRUE(link.Name().empty());

  link.SetName("test_link");
  EXPECT_EQ("test_link", link.Name());

  EXPECT_EQ(0u, link.VisualCount());
  EXPECT_EQ(nullptr, link.VisualByIndex(0));
  EXPECT_EQ(nullptr, link.VisualByIndex(1));
  EXPECT_FALSE(link.VisualNameExists(""));
  EXPECT_FALSE(link.VisualNameExists("default"));

  EXPECT_EQ(0u, link.LightCount());
  EXPECT_EQ(nullptr, link.LightByIndex(0));
  EXPECT_EQ(nullptr, link.LightByIndex(1));
  EXPECT_FALSE(link.LightNameExists(""));
  EXPECT_FALSE(link.LightNameExists("default"));
  EXPECT_EQ(nullptr, link.LightByName("no_such_light"));

  EXPECT_EQ(0u, link.ParticleEmitterCount());
  EXPECT_EQ(nullptr, link.ParticleEmitterByIndex(0));
  EXPECT_EQ(nullptr, link.ParticleEmitterByIndex(1));
  EXPECT_FALSE(link.ParticleEmitterNameExists(""));
  EXPECT_FALSE(link.ParticleEmitterNameExists("default"));
  EXPECT_EQ(nullptr, link.ParticleEmitterByName("no_such_emitter"));

  EXPECT_FALSE(link.EnableWind());
  link.SetEnableWind(true);
  EXPECT_TRUE(link.EnableWind());

  EXPECT_EQ(0u, link.SensorCount());
  EXPECT_EQ(nullptr, link.SensorByIndex(0));
  EXPECT_EQ(nullptr, link.SensorByIndex(1));
  EXPECT_EQ(nullptr, link.SensorByName("empty"));
  EXPECT_FALSE(link.SensorNameExists(""));
  EXPECT_FALSE(link.SensorNameExists("default"));

  EXPECT_EQ(ignition::math::Pose3d::Zero, link.RawPose());
  EXPECT_TRUE(link.PoseRelativeTo().empty());
  {
    auto semanticPose = link.SemanticPose();
    EXPECT_EQ(ignition::math::Pose3d::Zero, semanticPose.RawPose());
    EXPECT_TRUE(semanticPose.RelativeTo().empty());
    ignition::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  link.SetRawPose({10, 20, 30, 0, IGN_PI, 0});
  EXPECT_EQ(ignition::math::Pose3d(10, 20, 30, 0, IGN_PI, 0), link.RawPose());

  link.SetPoseRelativeTo("model");
  EXPECT_EQ("model", link.PoseRelativeTo());
  {
    auto semanticPose = link.SemanticPose();
    EXPECT_EQ(link.RawPose(), semanticPose.RawPose());
    EXPECT_EQ("model", semanticPose.RelativeTo());
    ignition::math::Pose3d pose;
    // expect errors when trying to resolve pose
    EXPECT_FALSE(semanticPose.Resolve(pose).empty());
  }

  // Get the default inertial
  const ignition::math::Inertiald inertial = link.Inertial();
  EXPECT_DOUBLE_EQ(1.0, inertial.MassMatrix().Mass());
  EXPECT_DOUBLE_EQ(1.0, inertial.MassMatrix().DiagonalMoments().X());
  EXPECT_DOUBLE_EQ(1.0, inertial.MassMatrix().DiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(1.0, inertial.MassMatrix().DiagonalMoments().Z());
  EXPECT_DOUBLE_EQ(0.0, inertial.MassMatrix().OffDiagonalMoments().X());
  EXPECT_DOUBLE_EQ(0.0, inertial.MassMatrix().OffDiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(0.0, inertial.MassMatrix().OffDiagonalMoments().Z());
  EXPECT_TRUE(inertial.MassMatrix().IsValid());

  EXPECT_EQ(0u, link.CollisionCount());
  EXPECT_EQ(nullptr, link.CollisionByIndex(0));
  EXPECT_EQ(nullptr, link.CollisionByIndex(1));
  EXPECT_FALSE(link.CollisionNameExists(""));
  EXPECT_FALSE(link.CollisionNameExists("default"));

  ignition::math::Inertiald inertial2 {
    {2.3,
      ignition::math::Vector3d(1.4, 2.3, 3.2),
      ignition::math::Vector3d(0.1, 0.2, 0.3)},
      ignition::math::Pose3d(1, 2, 3, 0, 0, 0)};

  EXPECT_TRUE(link.SetInertial(inertial2));

  EXPECT_DOUBLE_EQ(2.3, link.Inertial().MassMatrix().Mass());
  EXPECT_DOUBLE_EQ(1.4, link.Inertial().MassMatrix().DiagonalMoments().X());
  EXPECT_DOUBLE_EQ(2.3, link.Inertial().MassMatrix().DiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(3.2, link.Inertial().MassMatrix().DiagonalMoments().Z());
  EXPECT_DOUBLE_EQ(0.1, link.Inertial().MassMatrix().OffDiagonalMoments().X());
  EXPECT_DOUBLE_EQ(0.2, link.Inertial().MassMatrix().OffDiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(0.3, link.Inertial().MassMatrix().OffDiagonalMoments().Z());
  EXPECT_TRUE(link.Inertial().MassMatrix().IsValid());
}

/////////////////////////////////////////////////
TEST(DOMLink, CopyConstructor)
{
  sdf::Link link;
  link.SetName("test_link");

  sdf::Link link2(link);
  EXPECT_EQ("test_link", link2.Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, CopyAssignmentOperator)
{
  sdf::Link link;
  link.SetName("test_link");

  sdf::Link link2;
  link2 = link;
  EXPECT_EQ("test_link", link2.Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, MoveConstructor)
{
  sdf::Link link;
  link.SetName("test_link");

  sdf::Link link2(link);
  EXPECT_EQ("test_link", link2.Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, MoveAssignmentOperator)
{
  sdf::Link link;
  link.SetName("test_link");

  sdf::Link link2;
  link2 = std::move(link);
  EXPECT_EQ("test_link", link2.Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, CopyAssignmentAfterMove)
{
  sdf::Link link1;
  link1.SetName("link1");

  sdf::Link link2;
  link2.SetName("link2");

  // This is similar to what std::swap does except it uses std::move for each
  // assignment
  sdf::Link tmp = std::move(link1);
  link1 = link2;
  link2 = tmp;

  EXPECT_EQ("link2", link1.Name());
  EXPECT_EQ("link1", link2.Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, InvalidInertia)
{
  sdf::Link link;
  EXPECT_EQ(nullptr, link.Element());
  EXPECT_TRUE(link.Name().empty());

  ignition::math::Inertiald invalidInertial {
    {2.3, ignition::math::Vector3d(0.1, 0.2, 0.3),
      ignition::math::Vector3d(1.2, 2.3, 3.4)},
      ignition::math::Pose3d(1, 2, 3, 0, 0, 0)};

  EXPECT_FALSE(link.SetInertial(invalidInertial));

  EXPECT_DOUBLE_EQ(2.3, link.Inertial().MassMatrix().Mass());
  EXPECT_DOUBLE_EQ(0.1, link.Inertial().MassMatrix().DiagonalMoments().X());
  EXPECT_DOUBLE_EQ(0.2, link.Inertial().MassMatrix().DiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(0.3, link.Inertial().MassMatrix().DiagonalMoments().Z());
  EXPECT_DOUBLE_EQ(1.2, link.Inertial().MassMatrix().OffDiagonalMoments().X());
  EXPECT_DOUBLE_EQ(2.3, link.Inertial().MassMatrix().OffDiagonalMoments().Y());
  EXPECT_DOUBLE_EQ(3.4, link.Inertial().MassMatrix().OffDiagonalMoments().Z());
  EXPECT_FALSE(link.Inertial().MassMatrix().IsValid());
}

/////////////////////////////////////////////////
TEST(DOMLink, AddCollision)
{
  sdf::Link link;
  EXPECT_EQ(0u, link.CollisionCount());

  sdf::Collision collision;
  collision.SetName("collision1");
  EXPECT_TRUE(link.AddCollision(collision));
  EXPECT_EQ(1u, link.CollisionCount());
  EXPECT_FALSE(link.AddCollision(collision));
  EXPECT_EQ(1u, link.CollisionCount());

  link.ClearCollisions();
  EXPECT_EQ(0u, link.CollisionCount());

  EXPECT_TRUE(link.AddCollision(collision));
  EXPECT_EQ(1u, link.CollisionCount());
  const sdf::Collision *collisionFromLink = link.CollisionByIndex(0);
  ASSERT_NE(nullptr, collisionFromLink);
  EXPECT_EQ(collisionFromLink->Name(), collision.Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, AddVisual)
{
  sdf::Link link;
  EXPECT_EQ(0u, link.VisualCount());

  sdf::Visual visual;
  visual.SetName("visual1");
  EXPECT_TRUE(link.AddVisual(visual));
  EXPECT_EQ(1u, link.VisualCount());
  EXPECT_FALSE(link.AddVisual(visual));
  EXPECT_EQ(1u, link.VisualCount());

  link.ClearVisuals();
  EXPECT_EQ(0u, link.VisualCount());

  EXPECT_TRUE(link.AddVisual(visual));
  EXPECT_EQ(1u, link.VisualCount());
  const sdf::Visual *visualFromLink = link.VisualByIndex(0);
  ASSERT_NE(nullptr, visualFromLink);
  EXPECT_EQ(visualFromLink->Name(), visual.Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, AddLight)
{
  sdf::Link link;
  EXPECT_EQ(0u, link.LightCount());

  sdf::Light light;
  light.SetName("light1");
  EXPECT_TRUE(link.AddLight(light));
  EXPECT_EQ(1u, link.LightCount());
  EXPECT_FALSE(link.AddLight(light));
  EXPECT_EQ(1u, link.LightCount());

  link.ClearLights();
  EXPECT_EQ(0u, link.LightCount());

  EXPECT_TRUE(link.AddLight(light));
  EXPECT_EQ(1u, link.LightCount());
  const sdf::Light *lightFromLink = link.LightByIndex(0);
  ASSERT_NE(nullptr, lightFromLink);
  EXPECT_EQ(lightFromLink->Name(), light.Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, AddSensor)
{
  sdf::Link link;
  EXPECT_EQ(0u, link.SensorCount());

  sdf::Sensor sensor;
  sensor.SetName("sensor1");
  EXPECT_TRUE(link.AddSensor(sensor));
  EXPECT_EQ(1u, link.SensorCount());
  EXPECT_FALSE(link.AddSensor(sensor));
  EXPECT_EQ(1u, link.SensorCount());

  link.ClearSensors();
  EXPECT_EQ(0u, link.SensorCount());

  EXPECT_TRUE(link.AddSensor(sensor));
  EXPECT_EQ(1u, link.SensorCount());
  const sdf::Sensor *sensorFromLink = link.SensorByIndex(0);
  ASSERT_NE(nullptr, sensorFromLink);
  EXPECT_EQ(sensorFromLink->Name(), sensor.Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, ToElement)
{
  sdf::Link link;
  link.SetName("my-name");

  ignition::math::Inertiald inertial {
    {2.3,
      ignition::math::Vector3d(1.4, 2.3, 3.2),
      ignition::math::Vector3d(0.1, 0.2, 0.3)},
      ignition::math::Pose3d(1, 2, 3, 0, 0, 0)};
  EXPECT_TRUE(link.SetInertial(inertial));
  link.SetRawPose(ignition::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));
  link.SetEnableWind(true);

  for (int j = 0; j <= 1; ++j)
  {
    for (int i = 0; i < 1; ++i)
    {
      sdf::Collision collision;
      collision.SetName("collision" + std::to_string(i));
      EXPECT_TRUE(link.AddCollision(collision));
      EXPECT_FALSE(link.AddCollision(collision));
    }
    if (j == 0)
      link.ClearCollisions();
  }

  for (int j = 0; j <= 1; ++j)
  {
    for (int i = 0; i < 2; i++)
    {
      sdf::Visual visual;
      visual.SetName("visual" + std::to_string(i));
      EXPECT_TRUE(link.AddVisual(visual));
      EXPECT_FALSE(link.AddVisual(visual));
    }
    if (j == 0)
     link.ClearVisuals();
  }

  for (int j = 0; j <= 1; ++j)
  {
    for (int i = 0; i < 3; i++)
    {
      sdf::Light light;
      light.SetName("light" + std::to_string(i));
      EXPECT_TRUE(link.AddLight(light));
      EXPECT_FALSE(link.AddLight(light));
    }
    if (j == 0)
      link.ClearLights();
  }

  for (int j = 0; j <= 1; ++j)
  {
    for (int i = 0; i < 4; i++)
    {
      sdf::Sensor sensor;
      sensor.SetName("sensor" + std::to_string(i));
      EXPECT_TRUE(link.AddSensor(sensor));
      EXPECT_FALSE(link.AddSensor(sensor));
    }
    if (j == 0)
      link.ClearSensors();
  }

  for (int j = 0; j <= 1; ++j)
  {
    for (int i = 0; i < 5; i++)
    {
      sdf::ParticleEmitter emitter;
      emitter.SetName("emitter" + std::to_string(i));
      EXPECT_TRUE(link.AddParticleEmitter(emitter));
      EXPECT_FALSE(link.AddParticleEmitter(emitter));
    }
    if (j == 0)
      link.ClearParticleEmitters();
  }

  sdf::ElementPtr elem = link.ToElement();
  ASSERT_NE(nullptr, elem);

  sdf::Link link2;
  link2.Load(elem);

  EXPECT_EQ(link.Name(), link2.Name());
  EXPECT_EQ(link.Inertial(), link2.Inertial());
  EXPECT_EQ(link.RawPose(), link2.RawPose());
  EXPECT_EQ(link.EnableWind(), link2.EnableWind());
  EXPECT_EQ(link.CollisionCount(), link2.CollisionCount());
  for (uint64_t i = 0; i < link2.CollisionCount(); ++i)
    EXPECT_NE(nullptr, link2.CollisionByIndex(i));

  EXPECT_EQ(link.VisualCount(), link2.VisualCount());
  for (uint64_t i = 0; i < link2.VisualCount(); ++i)
    EXPECT_NE(nullptr, link2.VisualByIndex(i));

  EXPECT_EQ(link.LightCount(), link2.LightCount());
  for (uint64_t i = 0; i < link2.LightCount(); ++i)
    EXPECT_NE(nullptr, link2.LightByIndex(i));

  EXPECT_EQ(link.SensorCount(), link2.SensorCount());
  for (uint64_t i = 0; i < link2.SensorCount(); ++i)
    EXPECT_NE(nullptr, link2.SensorByIndex(i));

  EXPECT_EQ(link.ParticleEmitterCount(), link2.ParticleEmitterCount());
  for (uint64_t i = 0; i < link2.ParticleEmitterCount(); ++i)
    EXPECT_NE(nullptr, link2.ParticleEmitterByIndex(i));
}

/////////////////////////////////////////////////
TEST(DOMLink, MutableByIndex)
{
  sdf::Link link;
  link.SetName("my-name");

  sdf::Visual visual;
  visual.SetName("visual1");
  EXPECT_TRUE(link.AddVisual(visual));

  sdf::Collision collision;
  collision.SetName("collision1");
  EXPECT_TRUE(link.AddCollision(collision));

  sdf::Light light;
  light.SetName("light1");
  EXPECT_TRUE(link.AddLight(light));

  sdf::Sensor sensor;
  sensor.SetName("sensor1");
  EXPECT_TRUE(link.AddSensor(sensor));

  sdf::ParticleEmitter pe;
  pe.SetName("pe1");
  EXPECT_TRUE(link.AddParticleEmitter(pe));

  // Modify the visual
  sdf::Visual *v = link.VisualByIndex(0);
  ASSERT_NE(nullptr, v);
  EXPECT_EQ("visual1", v->Name());
  v->SetName("visual2");
  EXPECT_EQ("visual2", link.VisualByIndex(0)->Name());

  // Modify the collision
  sdf::Collision *c = link.CollisionByIndex(0);
  ASSERT_NE(nullptr, c);
  EXPECT_EQ("collision1", c->Name());
  c->SetName("collision2");
  EXPECT_EQ("collision2", link.CollisionByIndex(0)->Name());

  // Modify the light
  sdf::Light *l = link.LightByIndex(0);
  ASSERT_NE(nullptr, l);
  EXPECT_EQ("light1", l->Name());
  l->SetName("light2");
  EXPECT_EQ("light2", link.LightByIndex(0)->Name());

  // Modify the sensor
  sdf::Sensor *s = link.SensorByIndex(0);
  ASSERT_NE(nullptr, s);
  EXPECT_EQ("sensor1", s->Name());
  s->SetName("sensor2");
  EXPECT_EQ("sensor2", link.SensorByIndex(0)->Name());

  // Modify the particle emitter
  sdf::ParticleEmitter *p = link.ParticleEmitterByIndex(0);
  ASSERT_NE(nullptr, p);
  EXPECT_EQ("pe1", p->Name());
  p->SetName("pe2");
  EXPECT_EQ("pe2", link.ParticleEmitterByIndex(0)->Name());
}

/////////////////////////////////////////////////
TEST(DOMLink, MutableByName)
{
  sdf::Link link;
  link.SetName("my-name");

  sdf::Visual visual;
  visual.SetName("visual1");
  EXPECT_TRUE(link.AddVisual(visual));

  sdf::Collision collision;
  collision.SetName("collision1");
  EXPECT_TRUE(link.AddCollision(collision));

  sdf::Light light;
  light.SetName("light1");
  EXPECT_TRUE(link.AddLight(light));

  sdf::Sensor sensor;
  sensor.SetName("sensor1");
  EXPECT_TRUE(link.AddSensor(sensor));

  sdf::ParticleEmitter pe;
  pe.SetName("pe1");
  EXPECT_TRUE(link.AddParticleEmitter(pe));

  // Modify the visual
  sdf::Visual *v = link.VisualByName("visual1");
  ASSERT_NE(nullptr, v);
  EXPECT_EQ("visual1", v->Name());
  v->SetName("visual2");
  EXPECT_FALSE(link.VisualNameExists("visual1"));
  EXPECT_TRUE(link.VisualNameExists("visual2"));

  // Modify the collision
  sdf::Collision *c = link.CollisionByName("collision1");
  ASSERT_NE(nullptr, c);
  EXPECT_EQ("collision1", c->Name());
  c->SetName("collision2");
  EXPECT_FALSE(link.CollisionNameExists("collision1"));
  EXPECT_TRUE(link.CollisionNameExists("collision2"));

  // Modify the light
  sdf::Light *l = link.LightByName("light1");
  ASSERT_NE(nullptr, l);
  EXPECT_EQ("light1", l->Name());
  l->SetName("light2");
  EXPECT_FALSE(link.LightNameExists("light1"));
  EXPECT_TRUE(link.LightNameExists("light2"));

  // Modify the sensor
  sdf::Sensor *s = link.SensorByName("sensor1");
  ASSERT_NE(nullptr, s);
  EXPECT_EQ("sensor1", s->Name());
  s->SetName("sensor2");
  EXPECT_FALSE(link.SensorNameExists("sensor1"));
  EXPECT_TRUE(link.SensorNameExists("sensor2"));

  // Modify the particle emitter
  sdf::ParticleEmitter *p = link.ParticleEmitterByName("pe1");
  ASSERT_NE(nullptr, p);
  EXPECT_EQ("pe1", p->Name());
  p->SetName("pe2");
  EXPECT_FALSE(link.ParticleEmitterNameExists("pe1"));
  EXPECT_TRUE(link.ParticleEmitterNameExists("pe2"));
}
