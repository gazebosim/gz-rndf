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
#include "ignition/rndf/ParkingSpot.hh"
#include "ignition/rndf/Waypoint.hh"

using namespace ignition;
using namespace rndf;

// The fixture for testing ParkingSpot class.
class ParkingSpotTest : public testing::FileParserUtils
{
};

//////////////////////////////////////////////////
/// \brief Check id-related accessors.
TEST(ParkingSpot, id)
{
  int id = 1;
  ParkingSpot spot(id);

  EXPECT_EQ(spot.Id(), id);
  int newId = 2;
  EXPECT_TRUE(spot.SetId(newId));
  EXPECT_EQ(spot.Id(), newId);

  // Check that trying to set an incorrect Id does not take effect.
  int wrongId = -1;
  EXPECT_FALSE(spot.SetId(wrongId));
  EXPECT_EQ(spot.Id(), newId);

  // Check that using the constructor with a wrong id results in an invalid spot
  ParkingSpot wrongSpot(wrongId);
  EXPECT_FALSE(wrongSpot.Valid());
}

//////////////////////////////////////////////////
/// \brief Check waypoints-related functions.
TEST(ParkingSpot, waypoints)
{
  int id = 1;
  ParkingSpot spot(id);

  EXPECT_EQ(spot.NumWaypoints(), 0u);
  Waypoint wp;
  // Check an inexistent waypoint Id.
  EXPECT_FALSE(spot.Waypoint(id, wp));
  // Try to remove an inexistent waypoint id.
  EXPECT_FALSE(spot.RemoveWaypoint(id));
  // Try to add a waypoint with an invalid Id.
  EXPECT_FALSE(spot.AddWaypoint(wp));

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
  EXPECT_TRUE(spot.AddWaypoint(wp));
  EXPECT_EQ(spot.NumWaypoints(), 1u);

  // Try to add an existent waypoint.
  EXPECT_FALSE(spot.AddWaypoint(wp));
  EXPECT_EQ(spot.NumWaypoints(), 1u);

  // Get the waypoint.
  Waypoint wp2;
  EXPECT_TRUE(spot.Waypoint(wp.Id(), wp2));
  EXPECT_EQ(wp, wp2);

  // Update a waypoint.
  double newElevation = 2000;
  wp2.Location().SetElevationReference(newElevation);
  EXPECT_TRUE(spot.UpdateWaypoint(wp2));
  Waypoint wp3;
  EXPECT_TRUE(spot.Waypoint(wp2.Id(), wp3));
  EXPECT_EQ(wp3, wp2);

  // Get a mutable reference to all waypoints.
  std::vector<Waypoint> &waypoints = spot.Waypoints();
  ASSERT_EQ(waypoints.size(), 1u);
  // Modify a waypoint.
  Waypoint &aWp = waypoints.at(0);
  aWp.Location().SetElevationReference(500.0);
  EXPECT_TRUE(spot.Waypoint(wp2.Id(), wp3));
  EXPECT_TRUE(ignition::math::equal(wp3.Location().ElevationReference(),
    500.0));

  for (auto const &aWaypoint : spot.Waypoints())
    EXPECT_TRUE(aWaypoint.Valid());

  // Remove a waypoint.
  EXPECT_TRUE(spot.RemoveWaypoint(wp2.Id()));
  EXPECT_EQ(spot.NumWaypoints(), 0u);
}

//////////////////////////////////////////////////
/// \brief Check parking spot width.
TEST(ParkingSpot, Width)
{
  int id = 1;
  ParkingSpot spot(id);

  // Default parking spot width is 0.
  EXPECT_TRUE(ignition::math::equal(spot.Width(), 0.0));

  // Unable to change the width with an incorrect value.
  EXPECT_FALSE(spot.SetWidth(-1));

  // Change the width.
  double width = 1.0;
  EXPECT_TRUE(spot.SetWidth(width));
  EXPECT_TRUE(ignition::math::equal(spot.Width(), width));
}

//////////////////////////////////////////////////
/// \brief Check checkpoints-related functions.
TEST(ParkingSpot, checkpoints)
{
  int id = 1;
  ParkingSpot spot(id);

  // Create a valid checkpoint.
  Checkpoint cp;
  int checkpointId = 2;
  int waypointId = 3;
  cp.SetCheckpointId(checkpointId);
  cp.SetWaypointId(waypointId);
  EXPECT_TRUE(cp.Valid());

  // Set the checkpoint.
  spot.Checkpoint() = cp;

  // Get the checkpoint.
  Checkpoint cp2;
  cp2 = spot.Checkpoint();
  EXPECT_EQ(cp, cp2);
}

//////////////////////////////////////////////////
/// \brief Check function that validates the Id of a parking spot.
TEST(ParkingSpot, valid)
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
    int id = usecase.first;
    ParkingSpot spot(id);

    // An Id is not enough, we need two waypoints.
    EXPECT_FALSE(spot.Valid());

    // Create a valid waypoint.
    ignition::math::SphericalCoordinates::SurfaceType st =
      ignition::math::SphericalCoordinates::EARTH_WGS84;
    ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
    double elev = 354.1;
    ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);
    Waypoint wp1;
    wp1.SetId(1);
    wp1.Location() = sc;

    EXPECT_FALSE(spot.Valid());

    Waypoint wp2;
    wp2.SetId(2);
    wp2.Location() = sc;

    // Add the waypoints.
    EXPECT_TRUE(spot.AddWaypoint(wp1));
    EXPECT_TRUE(spot.AddWaypoint(wp2));

    EXPECT_EQ(spot.Valid(), usecase.second);

    // Try to add a third waypoint (not allowed).
    Waypoint wp3;
    wp3.SetId(3);
    wp3.Location() = sc;
    EXPECT_FALSE(spot.AddWaypoint(wp3));
  }
}

//////////////////////////////////////////////////
/// \brief Check [in]equality operators.
TEST(ParkingSpot, equality)
{
  int id1 = 1;
  ParkingSpot ps1(id1);

  int id2 = 2;
  ParkingSpot ps2(id2);

  ParkingSpot ps3(id1);

  EXPECT_FALSE(ps1 == ps2);
  EXPECT_TRUE(ps1 != ps2);

  EXPECT_TRUE(ps1 == ps3);
  EXPECT_FALSE(ps1 != ps3);
}

//////////////////////////////////////////////////
/// \brief Check assignment operator.
TEST(ParkingSpot, assignment)
{
  int id1 = 1;
  ParkingSpot ps1(id1);

  int id2 = 2;
  ParkingSpot ps2(id2);
  EXPECT_NE(ps1, ps2);

  ps2 = ps1;
  EXPECT_EQ(ps1, ps2);
}

//////////////////////////////////////////////////
/// \brief Check loading a parking spot from a file.
TEST_F(ParkingSpotTest, load)
{
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(34.587347), lon1(-117.366326), lon2(-117.366275);
  ignition::math::Angle heading(0.0);
  double elev = 354.1;

  ignition::math::SphericalCoordinates sc1(st, lat, lon1, elev, heading);
  ignition::math::SphericalCoordinates sc2(st, lat, lon2, elev, heading);

  Waypoint waypoint1(1, sc1);
  Waypoint waypoint2(2, sc1);

  Checkpoint checkpoint(130, 2);

  double spotWidth = 12 * 0.3048;

  // The first element is the content to be parsed.
  // The second element is the expected return value.
  // The third element is the test Id.
  // The forth element is the expected line value.
  std::vector<std::tuple<std::string, bool, int, int>> testCases =
  {
    std::make_tuple(""                              , false, 0, 1),
    std::make_tuple("\n\n"                          , false, 1, 3),
    // Missing spot.
    std::make_tuple(
      "\n\n"
      "xxx 61.1\n"
      "checkpoint  61.1.2  130\n"
      "61.1.1  34.587347 -117.366326\n"
      "61.1.2  34.587347 -117.366275\n"
      "end_spot\n"
                                                    , false, 2, 3),
    // Invalid spot Id.
    std::make_tuple(
      "\n\n"
      "spot\n"
      "checkpoint  61.1.2  130\n"
      "61.1.1  34.587347 -117.366326\n"
      "61.1.2  34.587347 -117.366275\n"
      "end_spot"
                                                    , false, 3, 3),
    // Invalid spot Id.
    std::make_tuple(
      "\n\n"
      "spot xxx\n"
      "checkpoint  61.1.2  130\n"
      "61.1.1  34.587347 -117.366326\n"
      "61.1.2  34.587347 -117.366275\n"
      "end_spot"
                                                    , false, 4, 3),
    // Waypoints missing.
    std::make_tuple(
      "\n\n"
      "spot 61.1\n"
      "checkpoint  61.1.2  130\n"
      "end_spot\n"
                                                    , false, 5, 5),
    // Invalid waypoint.
    std::make_tuple(
      "\n\n"
      "spot 61.1\n"
      "checkpoint  61.1.2  130\n"
      "61.1.1  34.587347\n"
      "61.1.2  34.587347 -117.366275\n"
      "end_spot\n"
                                                    , false, 6, 5),
    // Invalid waypoint.
    std::make_tuple(
      "\n\n"
      "spot 61.1\n"
      "checkpoint  61.1.2  130\n"
      "68.1.1  34.587347 -117.366326\n"
      "61.1.2  34.587347 -117.366275\n"
      "end_spot\n"
                                                    , false, 7, 5),
    // Invalid waypoint.
    std::make_tuple(
      "\n\n"
      "spot 61.1\n"
      "checkpoint  61.1.2  130\n"
      "61.8.1  34.587347 -117.366326\n"
      "61.1.2  34.587347 -117.366275\n"
      "end_spot\n"
                                                    , false, 8, 5),
    // Missing "end_spot" terminator.
    std::make_tuple(
      "\n\n"
      "spot 61.1\n"
      "checkpoint  61.1.2  130\n"
      "61.1.1  34.587347 -117.366326\n"
      "61.1.2  34.587347 -117.366275\n"
                                                    , false, 9, 7),
    // Missing "end_spot" terminator and find the next spot.
    std::make_tuple(
      "\n\n"
      "spot 61.1\n"
      "checkpoint  61.1.2  130\n"
      "61.1.1  34.587347 -117.366326\n"
      "61.1.2  34.587347 -117.366275\n"
      "spot"
                                                    , false, 10, 7),
    // No options.
    std::make_tuple(
      "\n/* comment */\n"
      "spot 61.1/* comment */\n"
      "61.1.1  34.587347 -117.366326/* comment */\n"
      "61.1.2  34.587347 -117.366275 /* comment */  \n"
      "end_spot\n"
                                                    , true, 11, 6),
    // Checkpoint option.
    std::make_tuple(
      "\n/* comment */\n"
      "spot 61.1/* comment */\n"
      "checkpoint  61.1.2  130 /* comment */\n"
      "61.1.1  34.587347 -117.366326/* comment */\n"
      "61.1.2  34.587347 -117.366275 /* comment */  \n"
      "end_spot\n"
                                                    , true, 12, 7),
    // Width options.
    std::make_tuple(
      "\n\n"
      "spot 61.1\n"
      "spot_width  12\n"
      "61.1.1  34.587347 -117.366326\n"
      "61.1.2  34.587347 -117.366275\n"
      "end_spot\n"
                                                    , true, 13, 7),
    // Both options.
    std::make_tuple(
      "\n\n"
      "spot 61.1\n"
      "spot_width  12\n"
      "checkpoint  61.1.2  130\n"
      "61.1.1  34.587347 -117.366326\n"
      "61.1.2  34.587347 -117.366275\n"
      "end_spot\n"
                                                    , true, 14, 8),
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
    ParkingSpot spot;
    bool res;
    EXPECT_EQ(res = spot.Load(f, 61, line), expectedResult);
    EXPECT_EQ(line, expectedLine);
    if (res)
    {
      switch (testId)
      {
        case 11:
          EXPECT_EQ(spot.Id(), 1);
          ASSERT_EQ(spot.NumWaypoints(), 2u);
          EXPECT_EQ(spot.Waypoints().at(0), waypoint1);
          EXPECT_EQ(spot.Waypoints().at(1), waypoint2);
          EXPECT_FLOAT_EQ(spot.Width(), 0);
          EXPECT_FALSE(spot.Checkpoint().Valid());
          break;
        case 12:
          EXPECT_EQ(spot.Id(), 1);
          ASSERT_EQ(spot.NumWaypoints(), 2u);
          EXPECT_EQ(spot.Waypoints().at(0), waypoint1);
          EXPECT_EQ(spot.Waypoints().at(1), waypoint2);
          EXPECT_FLOAT_EQ(spot.Width(), 0);
          EXPECT_EQ(spot.Checkpoint(), checkpoint);
          break;
        case 13:
          EXPECT_EQ(spot.Id(), 1);
          ASSERT_EQ(spot.NumWaypoints(), 2u);
          EXPECT_EQ(spot.Waypoints().at(0), waypoint1);
          EXPECT_EQ(spot.Waypoints().at(1), waypoint2);
          EXPECT_FLOAT_EQ(spot.Width(), spotWidth);
          EXPECT_FALSE(spot.Checkpoint().Valid());
          break;
        case 14:
          EXPECT_EQ(spot.Id(), 1);
          ASSERT_EQ(spot.NumWaypoints(), 2u);
          EXPECT_EQ(spot.Waypoints().at(0), waypoint1);
          EXPECT_EQ(spot.Waypoints().at(1), waypoint2);
          EXPECT_EQ(spot.Checkpoint(), checkpoint);
          EXPECT_FLOAT_EQ(spot.Width(), spotWidth);
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
