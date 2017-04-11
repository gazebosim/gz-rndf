/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <iostream>
#include <string>
#include <ignition/math/SphericalCoordinates.hh>

#include "gtest/gtest.h"
#include "ignition/rndf/Lane.hh"
#include "ignition/rndf/RNDFNode.hh"
#include "ignition/rndf/Segment.hh"
#include "ignition/rndf/UniqueId.hh"
#include "ignition/rndf/Waypoint.hh"
#include "ignition/rndf/Zone.hh"

using namespace ignition;
using namespace rndf;

//////////////////////////////////////////////////
/// \brief Check constructors.
TEST(UniqueIdTest, constructors)
{
  int segmentId = 1;
  int laneId = 2;
  int waypointId = 3;
  UniqueId id(segmentId, laneId, waypointId);
  RNDFNode rndfNode(id);
  EXPECT_EQ(rndfNode.UniqueId(), id);

  // Copy constructor.
  RNDFNode rndfNode2(rndfNode);
  EXPECT_EQ(rndfNode, rndfNode2);
}

//////////////////////////////////////////////////
/// \brief Check accesors.
TEST(UniqueIdTest, accessors)
{
  RNDFNode rndfNode;
  int segmentId = 1;
  int laneId = 2;
  int waypointId = 3;
  int zoneId = 4;
  UniqueId id(segmentId, laneId, waypointId);

  rndfNode.SetUniqueId(id);
  EXPECT_EQ(rndfNode.UniqueId(), id);

  Segment segment(segmentId);
  rndfNode.SetSegment(&segment);
  EXPECT_EQ(*rndfNode.Segment(), segment);

  Lane lane(laneId);
  rndfNode.SetLane(&lane);
  EXPECT_EQ(*rndfNode.Lane(), lane);

  Zone zone(zoneId);
  rndfNode.SetZone(&zone);
  EXPECT_EQ(*rndfNode.Zone(), zone);

  // Default surface type.
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);

  Waypoint waypoint(waypointId, sc);
  rndfNode.SetWaypoint(&waypoint);
  EXPECT_EQ(*rndfNode.Waypoint(), waypoint);
}

//////////////////////////////////////////////////
/// \brief Check operators.
TEST(UniqueIdTest, operators)
{
  int segmentId = 1;
  int laneId = 2;
  int waypointId = 3;
  UniqueId id(segmentId, laneId, waypointId);
  RNDFNode rndfNode(id);
  EXPECT_EQ(rndfNode.UniqueId(), id);

  // Copy constructor.
  RNDFNode rndfNode2(rndfNode);
  EXPECT_TRUE(rndfNode == rndfNode2);
  EXPECT_FALSE(rndfNode != rndfNode2);

  RNDFNode rndfNode3;
  EXPECT_NE(rndfNode, rndfNode3);
  rndfNode3 = rndfNode;
  EXPECT_EQ(rndfNode, rndfNode3);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
