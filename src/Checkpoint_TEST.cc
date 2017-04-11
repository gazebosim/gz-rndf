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

#include "gtest/gtest.h"
#include "ignition/rndf/Checkpoint.hh"

using namespace ignition;
using namespace rndf;

//////////////////////////////////////////////////
/// \brief Check accessors.
TEST(CheckpointTest, accessors)
{
  // Test invalid Ids.
  {
    Checkpoint cp(-1, 1);
    EXPECT_FALSE(cp.Valid());
  }
  {
    Checkpoint cp(0, 1);
    EXPECT_FALSE(cp.Valid());
  }
  {
    Checkpoint cp(1, -1);
    EXPECT_FALSE(cp.Valid());
  }
  {
    Checkpoint cp(1, 0);
    EXPECT_FALSE(cp.Valid());
  }

  // Test valid Ids.
  {
    int checkpointId = 2;
    int waypointId = 1;
    Checkpoint cp(checkpointId, waypointId);
    EXPECT_TRUE(cp.Valid());
    EXPECT_EQ(cp.CheckpointId(), checkpointId);
    EXPECT_EQ(cp.WaypointId(), waypointId);

    // Try to modify the checkpoint with an invalid Id.
    EXPECT_FALSE(cp.SetCheckpointId(-1));
    EXPECT_EQ(cp.CheckpointId(), checkpointId);
    EXPECT_TRUE(cp.Valid());

    // Modify checkpoint Id.
    int newCheckpointId = 20;
    EXPECT_TRUE(cp.SetCheckpointId(newCheckpointId));
    EXPECT_EQ(cp.CheckpointId(), newCheckpointId);
    EXPECT_TRUE(cp.Valid());

    // Try to modify the waypoint with an invalid Id.
    EXPECT_FALSE(cp.SetWaypointId(-1));
    EXPECT_EQ(cp.WaypointId(), waypointId);
    EXPECT_TRUE(cp.Valid());

    // Modify waypoint Id.
    int newWaypointId = 10;
    EXPECT_TRUE(cp.SetWaypointId(newWaypointId));
    EXPECT_EQ(cp.WaypointId(), newWaypointId);
    EXPECT_TRUE(cp.Valid());
  }
}

//////////////////////////////////////////////////
/// \brief Check [in]equality operators.
TEST(CheckpointTest, equality)
{
  int checkpointId1 = 1;
  int waypointId1 = 2;
  Checkpoint cp1(checkpointId1, waypointId1);

  int checkpointId2 = 2;
  int waypointId2 = 3;
  Checkpoint cp2(checkpointId2, waypointId2);

  Checkpoint cp3(checkpointId1, waypointId2);
  Checkpoint cp4(checkpointId2, waypointId1);

  EXPECT_FALSE(cp1 == cp2);
  EXPECT_TRUE(cp1 != cp2);

  EXPECT_TRUE(cp1 == cp3);
  EXPECT_FALSE(cp1 != cp3);

  EXPECT_FALSE(cp1 == cp4);
  EXPECT_TRUE(cp1 != cp4);
}

//////////////////////////////////////////////////
/// \brief Check assignment operator.
TEST(CheckpointTest, assignment)
{
  int checkpointId1 = 1;
  int waypointId1 = 2;
  Checkpoint cp1(checkpointId1, waypointId1);

  int checkpointId2 = 2;
  int waypointId2 = 3;
  Checkpoint cp2(checkpointId2, waypointId2);
  EXPECT_NE(cp1, cp2);

  cp2 = cp1;
  EXPECT_EQ(cp1, cp2);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
