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

#include "gtest/gtest.h"
#include "ignition/rndf/UniqueId.hh"

using namespace ignition;
using namespace rndf;

//////////////////////////////////////////////////
/// \brief Check accessors.
TEST(UniqueIdTest, accessors)
{
  // Test invalid Ids.
  {
    UniqueId id(-1, 1, 1);
    EXPECT_FALSE(id.Valid());
  }
  {
    UniqueId id(1, -1, 1);
    EXPECT_FALSE(id.Valid());
  }
  {
    UniqueId id(1, 1, 0);
    EXPECT_FALSE(id.Valid());
  }

  // Test valid Ids.
  {
    int segmentId = 1;
    int laneId = 2;
    int waypointId = 3;
    UniqueId id(segmentId, laneId, waypointId);
    EXPECT_TRUE(id.Valid());
    EXPECT_EQ(id.X(), segmentId);
    EXPECT_EQ(id.Y(), laneId);
    EXPECT_EQ(id.Z(), waypointId);

    // Try to modify the x with an invalid value.
    EXPECT_FALSE(id.SetX(-1));
    EXPECT_EQ(id.X(), segmentId);
    EXPECT_TRUE(id.Valid());

    // Modify x.
    int newSegmentId = 10;
    EXPECT_TRUE(id.SetX(newSegmentId));
    EXPECT_EQ(id.X(), newSegmentId);
    EXPECT_TRUE(id.Valid());

    // Try to modify the y with an invalid value.
    EXPECT_FALSE(id.SetY(-1));
    EXPECT_EQ(id.Y(), laneId);
    EXPECT_TRUE(id.Valid());

    // Modify y.
    int newLaneId = 10;
    EXPECT_TRUE(id.SetY(newLaneId));
    EXPECT_EQ(id.Y(), newLaneId);
    EXPECT_TRUE(id.Valid());

    // Try to modify the z with an invalid value.
    EXPECT_FALSE(id.SetZ(-1));
    EXPECT_EQ(id.Z(), waypointId);
    EXPECT_TRUE(id.Valid());

    // Modify z.
    int newWaypointId = 10;
    EXPECT_TRUE(id.SetZ(newWaypointId));
    EXPECT_EQ(id.Z(), newWaypointId);
    EXPECT_TRUE(id.Valid());
  }
}

//////////////////////////////////////////////////
/// \brief Check constructor using a string.
TEST(UniqueIdTest, stringConstructor)
{
  {
    UniqueId id("x.x.x");
    EXPECT_FALSE(id.Valid());
  }

  {
    UniqueId id("1");
    EXPECT_FALSE(id.Valid());
  }

  {
    UniqueId id("1.2");
    EXPECT_FALSE(id.Valid());
  }

  {
    UniqueId id("0.1.2");
    EXPECT_FALSE(id.Valid());
  }

  {
    UniqueId id("1.-1.2");
    EXPECT_FALSE(id.Valid());
  }

  {
    UniqueId id("1.1.0");
    EXPECT_FALSE(id.Valid());
  }

  {
    UniqueId id("32769.1.2");
    EXPECT_FALSE(id.Valid());
  }

  {
    UniqueId id("1.32769.2");
    EXPECT_FALSE(id.Valid());
  }

  {
    UniqueId id("1.2.32769");
    EXPECT_FALSE(id.Valid());
  }

  {
    UniqueId id("1x.0.2");
    EXPECT_FALSE(id.Valid());
  }

  {
    UniqueId id("1.0x.2");
    EXPECT_FALSE(id.Valid());
  }

  {
    UniqueId id("1.0.2x");
    EXPECT_FALSE(id.Valid());
  }

  {
    UniqueId id("1.0.2.3");
    EXPECT_FALSE(id.Valid());
  }

  {
    UniqueId id("1.0.2");
    EXPECT_TRUE(id.Valid());
  }
}

//////////////////////////////////////////////////
/// \brief Check [in]equality operators.
TEST(UniqueIdTest, equality)
{
  int segmentId1 = 1;
  int laneId1 = 2;
  int waypointId1 = 3;
  UniqueId id1(segmentId1, laneId1, waypointId1);

  int segmentId2 = 4;
  int laneId2 = 5;
  int waypointId2 = 6;
  UniqueId id2(segmentId2, laneId2, waypointId2);

  UniqueId id3(segmentId1, laneId2, waypointId2);

  EXPECT_FALSE(id1 == id2);
  EXPECT_TRUE(id1 != id2);

  EXPECT_FALSE(id1 == id3);
  EXPECT_TRUE(id1 != id3);
}

//////////////////////////////////////////////////
/// \brief Check assignment operator.
TEST(UniqueIdTest, assignment)
{
  int segmentId1 = 1;
  int laneId1 = 2;
  int waypointId1 = 3;
  UniqueId id1(segmentId1, laneId1, waypointId1);

  int segmentId2 = 4;
  int laneId2 = 5;
  int waypointId2 = 6;
  UniqueId id2(segmentId2, laneId2, waypointId2);
  EXPECT_NE(id1, id2);

  id2 = id1;
  EXPECT_EQ(id1, id2);
}

//////////////////////////////////////////////////
/// \brief Check the << operator
TEST(UniqueIdTest, streamInsertion)
{
  int segmentId = 1;
  int laneId = 2;
  int waypointId = 3;
  UniqueId id(segmentId, laneId, waypointId);

  std::ostringstream output;
  output << id;
  std::string expectedOutput = "1.2.3";

  EXPECT_EQ(output.str(), expectedOutput);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
