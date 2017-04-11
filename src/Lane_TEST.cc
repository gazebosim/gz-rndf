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
#include <map>
#include <string>
#include <tuple>
#include <vector>
#include <ignition/math/Helpers.hh>
#include <ignition/math/SphericalCoordinates.hh>

#include "gtest/gtest.h"
#include "ignition/rndf/test_config.h"
#include "ignition/rndf/Checkpoint.hh"
#include "ignition/rndf/Exit.hh"
#include "ignition/rndf/Lane.hh"
#include "ignition/rndf/UniqueId.hh"
#include "ignition/rndf/Waypoint.hh"

using namespace ignition;
using namespace rndf;

// The fixture for testing the Lane class.
class LaneTest : public testing::FileParserUtils
{
};

//////////////////////////////////////////////////
/// \brief Check id-related accessors.
TEST(Lane, id)
{
  int id = 1;
  Lane lane(id);

  EXPECT_EQ(lane.Id(), id);
  int newId = 2;
  EXPECT_TRUE(lane.SetId(newId));
  EXPECT_EQ(lane.Id(), newId);

  // Check that trying to set an incorrect Id does not take effect.
  int wrongId = -1;
  EXPECT_FALSE(lane.SetId(wrongId));
  EXPECT_EQ(lane.Id(), newId);

  // Check that using the constructor with a wrong id results in an invalid lane
  Lane wrongLane(wrongId);
  EXPECT_FALSE(wrongLane.Valid());
}

//////////////////////////////////////////////////
/// \brief Check waypoints-related functions.
TEST(Lane, waypoints)
{
  int id = 1;
  Lane lane(id);

  EXPECT_EQ(lane.NumWaypoints(), 0u);
  Waypoint wp;
  // Check an inexistent waypoint Id.
  EXPECT_FALSE(lane.Waypoint(id, wp));
  // Try to remove an inexistent waypoint id.
  EXPECT_FALSE(lane.RemoveWaypoint(id));
  // Try to add a waypoint with an invalid Id.
  EXPECT_FALSE(lane.AddWaypoint(wp));

  // Create a valid waypoint.
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);
  int waypointId = 1;
  wp.SetId(waypointId);
  wp.Location() = sc;

  // Add a valid waypoint.
  EXPECT_TRUE(lane.AddWaypoint(wp));
  EXPECT_EQ(lane.NumWaypoints(), 1u);

  // Try to add an existent waypoint.
  EXPECT_FALSE(lane.AddWaypoint(wp));
  EXPECT_EQ(lane.NumWaypoints(), 1u);

  // Get the waypoint.
  Waypoint wp2;
  EXPECT_TRUE(lane.Waypoint(wp.Id(), wp2));
  EXPECT_EQ(wp, wp2);

  // Update a waypoint.
  double newElevation = 2000;
  wp2.Location().SetElevationReference(newElevation);
  EXPECT_TRUE(lane.UpdateWaypoint(wp2));
  Waypoint wp3;
  EXPECT_TRUE(lane.Waypoint(wp2.Id(), wp3));
  EXPECT_EQ(wp3, wp2);

  // Get a mutable reference to all waypoints.
  std::vector<Waypoint> &waypoints = lane.Waypoints();
  ASSERT_EQ(waypoints.size(), 1u);
  // Modify a waypoint.
  Waypoint &aWp = waypoints.at(0);
  aWp.Location().SetElevationReference(500.0);
  EXPECT_TRUE(lane.Waypoint(wp2.Id(), wp3));
  EXPECT_TRUE(ignition::math::equal(wp3.Location().ElevationReference(),
    500.0));

  for (auto const &aWaypoint : lane.Waypoints())
    EXPECT_TRUE(aWaypoint.Valid());

  // Remove a waypoint.
  EXPECT_TRUE(lane.RemoveWaypoint(wp2.Id()));
  EXPECT_EQ(lane.NumWaypoints(), 0u);
}

//////////////////////////////////////////////////
/// \brief Check function that validates the Id of a lane.
TEST(Lane, valid)
{
  std::map<int, bool> cases =
  {
    {-1 , false},
    {0  , false},
    {1  , true},
    {100, true},
  };

  // Check all cases.
  for (auto const &usecase : cases)
  {
    // Create a valid waypoint.
    ignition::math::SphericalCoordinates::SurfaceType st =
      ignition::math::SphericalCoordinates::EARTH_WGS84;
    ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
    double elev = 354.1;
    ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);
    int waypointId = 1;
    Waypoint wp(waypointId, sc);

    int laneId = usecase.first;
    Lane lane(laneId);

    // Add a valid waypoint.
    EXPECT_TRUE(lane.AddWaypoint(wp));

    EXPECT_EQ(lane.Valid(), usecase.second);
  }
}

//////////////////////////////////////////////////
/// \brief Check lane width.
TEST(Lane, Width)
{
  int id = 1;
  Lane lane(id);

  // Default lane width is 0.
  EXPECT_TRUE(ignition::math::equal(lane.Width(), 0.0));

  // Unable to change the width with an incorrect value.
  EXPECT_FALSE(lane.SetWidth(-1));

  // Change the width.
  double width = 1.0;
  EXPECT_TRUE(lane.SetWidth(width));
  EXPECT_TRUE(ignition::math::equal(lane.Width(), width));
}

//////////////////////////////////////////////////
/// \brief Check lane boundaries.
TEST(Lane, Boundaries)
{
  int id = 1;
  Lane lane(id);

  // Default boundary is UNDEFINED.
  EXPECT_EQ(lane.LeftBoundary(), Marking::UNDEFINED);
  EXPECT_EQ(lane.RightBoundary(), Marking::UNDEFINED);

  // Change the left boundary.
  Marking leftBoundary = Marking::DOUBLE_YELLOW;
  lane.SetLeftBoundary(leftBoundary);
  EXPECT_EQ(lane.LeftBoundary(), leftBoundary);
  EXPECT_EQ(lane.RightBoundary(), Marking::UNDEFINED);

  // Change the right boundary.
  Marking rightBoundary = Marking::SOLID_YELLOW;
  lane.SetRightBoundary(rightBoundary);
  EXPECT_EQ(lane.RightBoundary(), rightBoundary);
  EXPECT_EQ(lane.LeftBoundary(), leftBoundary);
}

//////////////////////////////////////////////////
/// \brief Check checkpoints-related functions.
TEST(Lane, checkpoints)
{
  int id = 1;
  Lane lane(id);

  EXPECT_EQ(lane.NumCheckpoints(), 0u);
  Checkpoint cp;
  // Check an inexistent checkpoint Id.
  EXPECT_FALSE(lane.Checkpoint(id, cp));
  // Try to remove an inexistent checkpoint id.
  EXPECT_FALSE(lane.RemoveCheckpoint(id));
  // Try to add a checkpoint with an invalid Id.
  EXPECT_FALSE(lane.AddCheckpoint(cp));

  // Create a valid checkpoint.
  int checkpointId = 2;
  int waypointId = 3;
  cp.SetCheckpointId(checkpointId);
  cp.SetWaypointId(waypointId);
  EXPECT_TRUE(cp.Valid());

  // Add a valid checkpoint.
  EXPECT_TRUE(lane.AddCheckpoint(cp));
  EXPECT_EQ(lane.NumCheckpoints(), 1u);

  // Try to add an existent checkpoint.
  EXPECT_FALSE(lane.AddCheckpoint(cp));
  EXPECT_EQ(lane.NumCheckpoints(), 1u);

  // Get the checkpoint.
  Checkpoint cp2;
  EXPECT_TRUE(lane.Checkpoint(cp.CheckpointId(), cp2));
  EXPECT_EQ(cp, cp2);

  // Update a checkpoint.
  int newWaypointId = 4;
  cp2.SetWaypointId(newWaypointId);
  EXPECT_TRUE(lane.UpdateCheckpoint(cp2));
  Checkpoint cp3;
  EXPECT_TRUE(lane.Checkpoint(cp2.CheckpointId(), cp3));
  EXPECT_EQ(cp3, cp2);

  // Get a mutable reference to all checkpoints.
  std::vector<Checkpoint> &checkpoints = lane.Checkpoints();
  ASSERT_EQ(checkpoints.size(), 1u);
  // Modify a checkpoint.
  Checkpoint &aCp = checkpoints.at(0);
  aCp.SetWaypointId(5);
  EXPECT_TRUE(lane.Checkpoint(cp2.CheckpointId(), cp3));
  EXPECT_EQ(cp3.WaypointId(), 5);

  for (auto const &aCheckpoint : lane.Checkpoints())
    EXPECT_TRUE(aCheckpoint.Valid());

  // Create a valid waypoint.
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);
  Waypoint wp;
  wp.SetId(1);
  wp.Location() = sc;
  EXPECT_TRUE(lane.AddWaypoint(wp));

  EXPECT_TRUE(lane.Valid());

  // Remove a checkpoint.
  EXPECT_TRUE(lane.RemoveCheckpoint(cp2.CheckpointId()));
  EXPECT_EQ(lane.NumCheckpoints(), 0u);
}

//////////////////////////////////////////////////
/// \brief Check stops-related functions.
TEST(Lane, checkStops)
{
  int id = 1;
  Lane lane(id);

  EXPECT_EQ(lane.NumStops(), 0u);
  // Try to remove an inexistent stop id.
  EXPECT_FALSE(lane.RemoveStop(id));
  // Try to add a stop with an invalid Id.
  EXPECT_FALSE(lane.AddStop(0));

  // Add a valid stop.
  EXPECT_TRUE(lane.AddStop(1));
  EXPECT_EQ(lane.NumStops(), 1u);

  // Try to add an existent stop.
  EXPECT_FALSE(lane.AddStop(1));
  EXPECT_EQ(lane.NumStops(), 1u);

  // Get a mutable reference to all stops.
  std::vector<int> &stops = lane.Stops();
  ASSERT_EQ(stops.size(), 1u);
  // Modify a stop.
  stops.at(0) = 2;
  EXPECT_EQ(lane.Stops().at(0), 2);

  for (auto const &waypointId : lane.Stops())
    EXPECT_GT(waypointId, 0);

  // Remove a stop.
  EXPECT_TRUE(lane.RemoveStop(2));
  EXPECT_EQ(lane.NumStops(), 0u);
}

//////////////////////////////////////////////////
/// \brief Check exits-related functions.
TEST(Lane, checkExits)
{
  int id = 1;
  Lane lane(id);

  EXPECT_EQ(lane.NumExits(), 0u);
  // Try to remove an inexistent exit.
  EXPECT_FALSE(lane.RemoveExit(Exit()));
  // Try to add an invalid exit.
  EXPECT_FALSE(lane.AddExit(Exit()));

  // Add a valid exit.
  UniqueId exitId(1, 2, 3);
  UniqueId entryId(4, 5, 6);
  Exit exit1(exitId, entryId);
  EXPECT_TRUE(lane.AddExit(exit1));
  EXPECT_EQ(lane.NumExits(), 1u);

  // Try to add an existent exit.
  EXPECT_FALSE(lane.AddExit(exit1));
  EXPECT_EQ(lane.NumExits(), 1u);

  // Get a mutable reference to all exits.
  std::vector<Exit> &exits = lane.Exits();
  ASSERT_EQ(exits.size(), 1u);

  for (auto const &exit : lane.Exits())
    EXPECT_TRUE(exit.Valid());

  // Remove an exit.
  EXPECT_TRUE(lane.RemoveExit(exit1));
  EXPECT_EQ(lane.NumExits(), 0u);
}

//////////////////////////////////////////////////
/// \brief Check [in]equality operators.
TEST(Lane, equality)
{
  int id1 = 1;
  Lane lane1(id1);

  int id2 = 2;
  Lane lane2(id2);

  Lane lane3(id1);

  EXPECT_FALSE(lane1 == lane2);
  EXPECT_TRUE(lane1 != lane2);

  EXPECT_TRUE(lane1 == lane3);
  EXPECT_FALSE(lane1 != lane3);
}

//////////////////////////////////////////////////
/// \brief Check assignment operator.
TEST(Lane, assignment)
{
  int id1 = 1;
  Lane lane1(id1);

  int id2 = 2;
  Lane lane2(id2);
  EXPECT_NE(lane1, lane2);

  lane2 = lane1;
  EXPECT_EQ(lane1, lane2);
}

//////////////////////////////////////////////////
/// \brief Check loading a lane from a text file.
TEST_F(LaneTest, Load)
{
  // The first element is the content to be parsed.
  // The second element is the expected return value.
  // The third element is the test Id.
  // The forth element is the expected line value.
  std::vector<std::tuple<std::string, bool, int, int>> testCases =
  {
    std::make_tuple(""                              , false, 0, 1),
    std::make_tuple("\n\n"                          , false, 0, 3),
    // Missing lane.
    std::make_tuple(
      "\n\n"
      "xxx  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 3),
    // Missing lane Id.
    std::make_tuple(
      "\n\n"
      "lane \n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 3),
    // Invalid lane Id.
    std::make_tuple(
      "\n\n"
      "lane  xxx\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 3),
    // Missing lane Id.
    std::make_tuple(
      "\n\n"
      "lane  60\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 3),
    // Invalid lane Id.
    std::make_tuple(
      "\n\n"
      "lane  60.x\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 3),
    // Invalid lane Id.
    std::make_tuple(
      "\n\n"
      "lane  60.0\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 3),
    // num_waypoints missing.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 4),
    // Missing num_waypoints value.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 4),
    // Invalid num_waypoints value.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints \nxxx"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 4),
    // Invalid num_waypoints value.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 0\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 4),
    // Missing lane_width.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 5),
    // Missing lane_width value.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lan\ne_width "
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 5),
    // Invalid lane_width value.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width -1\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 5),
    // Repeated lane_width value.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width 12\n"
      "lane_width 12\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 6),
    // Missing left/right_boundary.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "double_yellow\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 5),
    // Missing left_boundary value.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "left_boundary \n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 5),
    // Invalid left_boundary value.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "left_boundary x\nxx"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 5),
    // Repeated left_boundary value.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "left_boundary double_yellow\n"
      "left_boundary solid_yellow\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 6),
    // Missing right_boundary value.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "right_boundary \n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 5),
    // Invalid right_boundary value.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "right_boundary xxx\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 5),
    // Repeated right_boundary value.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "right_boundary double_yellow\n"
      "right_boundary solid_yellow\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 6),
    // Missing checkpoint.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "60.1.1 10\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 5),
    // Missing waypoint when using a checkpoint.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "checkpoint 10\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 5),
    // Missing checkpoint Id when using a checkpoint.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "checkpoint 60.1.1 \n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 5),
    // Missing stop.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "60.1.2\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 5),
    // Missing waypoint when using a stop.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "stop \n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 5),
    // Incorrect waypoint when using a stop
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "stop xxx \n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 5),
    // Missing exit.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "60.1.2 8.1.1\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 5),
    // Missing exit waypoint when using an exit.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "exit   63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 5),
    // Missing entry waypoint when using an exit.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "exit  60.1.5   /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 5),
    // Missing most of the fields.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
                                                    , false, 0, 5),
    // Missing "end_lane" terminator.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
                                                     , false, 0, 12),
    // Wrong "end_lane" terminator.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end\n"
                                                    , false, 0, 12),
    // Missing "end_lane" terminator and found the next lane.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "lane 60.2\n"
                                                    , false, 0, 12),
    // Missing "waypoints" (there are only four of them, instead of five).
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "end_lane\n"
                                                    , false, 0, 11),
    // More "waypoints" than the expected number.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "60.1.6  34.587114 -117.368019\n"
      "end_lane\n"
                                                    , false, 0, 12),
    // Invalid lane Id in the waypoint (60 is the expected value).
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "99.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 7),
    // Non-consecutive waypoints.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.9  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , false, 0, 7),
    // No options.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , true, 1, 10),
    // 1 option (width).
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , true, 2, 11),
    // Two options (lane boundaries).
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "left_boundary double_yellow\n"
      "right_boundary solid_yellow\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , true, 3, 12),
    // Three options (width, two checkpoints).
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "checkpoint 60.1.1 10\n"
      "checkpoint 60.1.2 11\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , true, 4, 13),
    // Four options (width, two checkpoints, stop).
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "checkpoint 60.1.1 10\n"
      "stop 60.1.4\n"
      "checkpoint 60.1.2 11\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , true, 5, 14),
    // All options.
    std::make_tuple(
      "\n\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "checkpoint 60.1.1 10\n"
      "stop 60.1.4\n"
      "checkpoint 60.1.2 11\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "exit  60.1.5  63.0.5  /* Comment */\n"
      "right_boundary solid_yellow\n"
      "left_boundary double_yellow\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                    , true, 6, 18),
  };

  for (auto const &testCase : testCases)
  {
    int line = 0;
    std::string content = std::get<0>(testCase);
    int testId = std::get<2>(testCase);

    // Expectations.
    bool expectedResult = std::get<1>(testCase);
    int expectedLine = std::get<3>(testCase);

    // Write the content of this test case into the test file.
    this->PopulateFile(content);
    std::ifstream f(this->fileName);

    // Leave this comment for knowing wich test case failed if needed.
    std::cout << "Testing [" << content << "]" << std::endl;

    // Check expectations.
    Lane lane;
    bool res;
    EXPECT_EQ(res = lane.Load(f, 60, line), expectedResult);
    EXPECT_EQ(line, expectedLine);
    if (res)
    {
      switch (testId)
      {
        case 1:
          EXPECT_EQ(lane.Id(), 1);
          ASSERT_EQ(lane.NumWaypoints(), 5u);
          EXPECT_FLOAT_EQ(lane.Width(), 0);
          EXPECT_EQ(lane.LeftBoundary(), Marking::UNDEFINED);
          EXPECT_EQ(lane.RightBoundary(), Marking::UNDEFINED);
          EXPECT_EQ(lane.NumCheckpoints(), 0u);
          EXPECT_EQ(lane.NumExits(), 0u);
          break;
        case 2:
          EXPECT_EQ(lane.Id(), 1);
          ASSERT_EQ(lane.NumWaypoints(), 5u);
          EXPECT_FLOAT_EQ(lane.Width(), 15 * 0.3048);
          EXPECT_EQ(lane.LeftBoundary(), Marking::UNDEFINED);
          EXPECT_EQ(lane.RightBoundary(), Marking::UNDEFINED);
          EXPECT_EQ(lane.NumCheckpoints(), 0u);
          EXPECT_EQ(lane.NumExits(), 0u);
          break;
        case 3:
          EXPECT_EQ(lane.Id(), 1);
          ASSERT_EQ(lane.NumWaypoints(), 5u);
          EXPECT_FLOAT_EQ(lane.Width(), 0);
          EXPECT_EQ(lane.LeftBoundary(), Marking::DOUBLE_YELLOW);
          EXPECT_EQ(lane.RightBoundary(), Marking::SOLID_YELLOW);
          EXPECT_EQ(lane.NumCheckpoints(), 0u);
          EXPECT_EQ(lane.NumExits(), 0u);
          break;
        case 4:
          EXPECT_EQ(lane.Id(), 1);
          ASSERT_EQ(lane.NumWaypoints(), 5u);
          EXPECT_FLOAT_EQ(lane.Width(), 15 * 0.3048);
          EXPECT_EQ(lane.LeftBoundary(), Marking::UNDEFINED);
          EXPECT_EQ(lane.RightBoundary(), Marking::UNDEFINED);
          EXPECT_EQ(lane.NumCheckpoints(), 2u);
          EXPECT_EQ(lane.NumStops(), 0u);
          EXPECT_EQ(lane.NumExits(), 0u);
          break;
        case 5:
          EXPECT_EQ(lane.Id(), 1);
          ASSERT_EQ(lane.NumWaypoints(), 5u);
          EXPECT_FLOAT_EQ(lane.Width(), 15 * 0.3048);
          EXPECT_EQ(lane.LeftBoundary(), Marking::UNDEFINED);
          EXPECT_EQ(lane.RightBoundary(), Marking::UNDEFINED);
          EXPECT_EQ(lane.NumCheckpoints(), 2u);
          EXPECT_EQ(lane.NumStops(), 1u);
          EXPECT_EQ(lane.NumExits(), 0u);
          break;
        case 6:
          EXPECT_EQ(lane.Id(), 1);
          ASSERT_EQ(lane.NumWaypoints(), 5u);
          EXPECT_FLOAT_EQ(lane.Width(), 15 * 0.3048);
          EXPECT_EQ(lane.LeftBoundary(), Marking::DOUBLE_YELLOW);
          EXPECT_EQ(lane.RightBoundary(), Marking::SOLID_YELLOW);
          EXPECT_EQ(lane.NumCheckpoints(), 2u);
          EXPECT_EQ(lane.NumStops(), 1u);
          EXPECT_EQ(lane.NumExits(), 2u);
          break;
        default:
          break;
      };
    }
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
