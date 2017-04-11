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
#include <tuple>
#include <vector>
#include <ignition/math/Helpers.hh>
#include <ignition/math/SphericalCoordinates.hh>

#include "gtest/gtest.h"
#include "ignition/rndf/test_config.h"
#include "ignition/rndf/Exit.hh"
#include "ignition/rndf/Perimeter.hh"
#include "ignition/rndf/UniqueId.hh"
#include "ignition/rndf/Waypoint.hh"

using namespace ignition;
using namespace rndf;

// The fixture for testing the Perimeter class.
class PerimeterTest : public testing::FileParserUtils
{
};

//////////////////////////////////////////////////
/// \brief Check points-related functions.
TEST(Perimeter, points)
{
  Perimeter perimeter;
  EXPECT_FALSE(perimeter.Valid());

  EXPECT_EQ(perimeter.NumPoints(), 0u);
  Waypoint wp;
  // Check an inexistent waypoint Id.
  int id = 1;
  EXPECT_FALSE(perimeter.Point(id, wp));
  // Try to remove an inexistent waypoint id.
  EXPECT_FALSE(perimeter.RemovePoint(id));
  // Try to add a waypoint with an invalid Id.
  EXPECT_FALSE(perimeter.AddPoint(wp));

  // Create a valid waypoint.
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);
  int waypointId = 1;
  wp.SetId(waypointId);
  wp.Location() = sc;

  // Add a valid point.
  EXPECT_TRUE(perimeter.AddPoint(wp));
  EXPECT_EQ(perimeter.NumPoints(), 1u);
  EXPECT_TRUE(perimeter.Valid());

  // Try to add an existent point.
  EXPECT_FALSE(perimeter.AddPoint(wp));
  EXPECT_EQ(perimeter.NumPoints(), 1u);
  EXPECT_TRUE(perimeter.Valid());

  // Get the point.
  Waypoint wp2;
  EXPECT_TRUE(perimeter.Point(wp.Id(), wp2));
  EXPECT_EQ(wp, wp2);

  // Update a point.
  double newElevation = 2000;
  wp2.Location().SetElevationReference(newElevation);
  EXPECT_TRUE(perimeter.UpdatePoint(wp2));
  Waypoint wp3;
  EXPECT_TRUE(perimeter.Point(wp2.Id(), wp3));
  EXPECT_EQ(wp3, wp2);

  // Get a mutable reference to all points.
  std::vector<Waypoint> &points = perimeter.Points();
  ASSERT_EQ(points.size(), 1u);
  // Modify a point.
  Waypoint &aWp = points.at(0);
  aWp.Location().SetElevationReference(500.0);
  EXPECT_TRUE(perimeter.Point(wp2.Id(), wp3));
  EXPECT_TRUE(ignition::math::equal(wp3.Location().ElevationReference(),
    500.0));

  for (auto const &aPoint : perimeter.Points())
    EXPECT_TRUE(aPoint.Valid());

  // Remove a point.
  EXPECT_TRUE(perimeter.RemovePoint(wp2.Id()));
  EXPECT_EQ(perimeter.NumPoints(), 0u);
  EXPECT_FALSE(perimeter.Valid());
}

//////////////////////////////////////////////////
/// \brief Check exits-related functions.
TEST(Perimeter, checkExits)
{
  Perimeter perimeter;

  EXPECT_EQ(perimeter.NumExits(), 0u);
  // Try to remove an inexistent exit.
  EXPECT_FALSE(perimeter.RemoveExit(Exit()));
  // Try to add an invalid exit.
  EXPECT_FALSE(perimeter.AddExit(Exit()));

  // Add a valid exit.
  UniqueId exitId(1, 2, 3);
  UniqueId entryId(4, 5, 6);
  Exit exit1(exitId, entryId);
  EXPECT_TRUE(perimeter.AddExit(exit1));
  EXPECT_EQ(perimeter.NumExits(), 1u);

  // Try to add an existent exit.
  EXPECT_FALSE(perimeter.AddExit(exit1));
  EXPECT_EQ(perimeter.NumExits(), 1u);

  // Get a mutable reference to all exits.
  std::vector<Exit> &exits = perimeter.Exits();
  ASSERT_EQ(exits.size(), 1u);

  for (auto const &exit : perimeter.Exits())
    EXPECT_TRUE(exit.Valid());

  // Remove an exit.
  EXPECT_TRUE(perimeter.RemoveExit(exit1));
  EXPECT_EQ(perimeter.NumExits(), 0u);
}

//////////////////////////////////////////////////
/// \brief Check [in]equality operators.
TEST(Perimeter, equality)
{
  // Create a valid waypoint.
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc1(st, lat, lon, elev, heading);
  int waypointId = 1;
  Waypoint wp1;
  wp1.SetId(waypointId);
  wp1.Location() = sc1;

  UniqueId exitId1(1, 2, 3);
  UniqueId entryId1(4, 5, 6);
  UniqueId entryId2(5, 6, 7);

  Exit exit1(exitId1, entryId1);
  Exit exit2(exitId1, entryId2);

  // Perimeter #1 (one point).
  Perimeter perimeter1;
  EXPECT_TRUE(perimeter1.AddPoint(wp1));

  // Perimeter #2 (zero points).
  Perimeter perimeter2;

  EXPECT_FALSE(perimeter1 == perimeter2);
  EXPECT_TRUE(perimeter1 != perimeter2);

  // Add the same point to perimeter2.
  EXPECT_TRUE(perimeter2.AddPoint(wp1));

  EXPECT_TRUE(perimeter1 == perimeter2);
  EXPECT_FALSE(perimeter1 != perimeter2);

  // Add two exits to perimeter 1.
  EXPECT_TRUE(perimeter1.AddExit(exit1));
  EXPECT_TRUE(perimeter1.AddExit(exit2));

  EXPECT_FALSE(perimeter1 == perimeter2);
  EXPECT_TRUE(perimeter1 != perimeter2);

  // Add the same exits to perimeter 2.
  EXPECT_TRUE(perimeter2.AddExit(exit1));
  EXPECT_TRUE(perimeter2.AddExit(exit2));

  EXPECT_TRUE(perimeter1 == perimeter2);
  EXPECT_FALSE(perimeter1 != perimeter2);
}

//////////////////////////////////////////////////
/// \brief Check assignment operator.
TEST(Perimeter, assignment)
{
  // Create a valid waypoint.
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc1(st, lat, lon, elev, heading);
  int waypointId = 1;
  Waypoint wp1;
  wp1.SetId(waypointId);
  wp1.Location() = sc1;

  UniqueId exitId1(1, 2, 3);
  UniqueId entryId1(4, 5, 6);

  Exit exit1(exitId1, entryId1);

  // Perimeter #1 (one point and one exit) and perimeter #2 (no points/exits).
  Perimeter perimeter1;
  Perimeter perimeter2;
  EXPECT_TRUE(perimeter1.AddPoint(wp1));
  EXPECT_TRUE(perimeter1.AddExit(exit1));

  EXPECT_NE(perimeter1, perimeter2);

  perimeter2 = perimeter1;
  EXPECT_EQ(perimeter1, perimeter2);
}

//////////////////////////////////////////////////
/// \brief Check loading a perimeter from a file.
TEST_F(PerimeterTest, load)
{
  // The first element is the content to be parsed.
  // The second element is the expected return value.
  // The third element is the test Id.
  // The forth element is the expected line value.
  std::vector<std::tuple<std::string, bool, int, int>> testCases =
  {
    std::make_tuple(""                              , false, 0, 1),
    std::make_tuple("\n\n"                          , false, 1, 3),
    // Missing perimeter.
    std::make_tuple(
      "\n\n"
      "xxx 61.0\n"
      "num_perimeterpoints  3\n"
      "61.0.1 34.587434 -117.367061\n"
      "61.0.2 34.587436 -117.366847\n"
      "61.0.3 34.587440 -117.366444\n"
      "end_perimeter\n"
                                                    , false, 2, 3),
    // Invalid perimeter Id.
    std::make_tuple(
      "\n\n"
      "perimeter\n"
      "num_perimeterpoints  3\n"
      "61.0.1 34.587434 -117.367061\n"
      "61.0.2 34.587436 -117.366847\n"
      "61.0.3 34.587440 -117.366444\n"
      "end_perimeter"
                                                    , false, 3, 3),
    // Invalid perimeter Id.
    std::make_tuple(
      "\n\n"
      "perimeter xxx\n"
      "num_perimeterpoints  3\n"
      "61.0.1 34.587434 -117.367061\n"
      "61.0.2 34.587436 -117.366847\n"
      "61.0.3 34.587440 -117.366444\n"
      "end_perimeter"
                                                    , false, 4, 3),
    // Invalid perimeter Id (should be 0 and it's 1).
    std::make_tuple(
      "\n\n"
      "perimeter 61.1\n"
      "num_perimeterpoints  3\n"
      "61.0.1 34.587434 -117.367061\n"
      "61.0.2 34.587436 -117.366847\n"
      "61.0.3 34.587440 -117.366444\n"
      "end_perimeter"
                                                    , false, 5, 3),
    // Missing num_perimeterpoints.
    std::make_tuple(
      "\n\n"
      "perimeter 61.0\n"
      "61.0.1 34.587434 -117.367061\n"
      "61.0.2 34.587436 -117.366847\n"
      "61.0.3 34.587440 -117.366444\n"
      "end_perimeter"
                                                    , false, 6, 4),
    // Invalid num_perimeterpoints.
    std::make_tuple(
      "\n\n"
      "perimeter 61.0\n"
      "xxx  3\n"
      "61.0.1 34.587434 -117.367061\n"
      "61.0.2 34.587436 -117.366847\n"
      "61.0.3 34.587440 -117.366444\n"
      "end_perimeter"
                                                    , false, 7, 4),
    // Missing num_perimeterpoints value.
    std::make_tuple(
      "\n\n"
      "perimeter 61.0\n"
      "num_perimeterpoints \n"
      "61.0.1 34.587434 -117.367061\n"
      "61.0.2 34.587436 -117.366847\n"
      "61.0.3 34.587440 -117.366444\n"
      "end_perimeter"
                                                    , false, 8, 4),
    // Invalid num_perimeterpoints value.
    std::make_tuple(
      "\n\n"
      "perimeter 61.0\n"
      "num_perimeterpoints xxx\n"
      "61.0.1 34.587434 -117.367061\n"
      "61.0.2 34.587436 -117.366847\n"
      "61.0.3 34.587440 -117.366444\n"
      "end_perimeter"
                                                    , false, 9, 4),
    // Invalid num_perimeterpoints value (zero).
    std::make_tuple(
      "\n\n"
      "perimeter 61.0\n"
      "num_perimeterpoints 0\n"
      "61.0.1 34.587434 -117.367061\n"
      "61.0.2 34.587436 -117.366847\n"
      "61.0.3 34.587440 -117.366444\n"
      "end_perimeter"
                                                    , false, 10, 4),
    // Missing one perimeterpoint.
    std::make_tuple(
      "\n\n"
      "perimeter 61.0\n"
      "num_perimeterpoints 4\n"
      "61.0.1 34.587434 -117.367061\n"
      "61.0.2 34.587436 -117.366847\n"
      "61.0.3 34.587440 -117.366444\n"
      "end_perimeter"
                                                    , false, 11, 8),
    // Extra perimeterpoint found.
    std::make_tuple(
      "\n\n"
      "perimeter 61.0\n"
      "num_perimeterpoints 3\n"
      "61.0.1 34.587434 -117.367061\n"
      "61.0.2 34.587436 -117.366847\n"
      "61.0.3 34.587440 -117.366444\n"
      "61.0.3 34.587440 -117.366939\n"
      "end_perimeter"
                                                    , false, 12, 8),
    // Invalid num_perimeterpoints value (negative).
    std::make_tuple(
      "\n\n"
      "perimeter 61.0\n"
      "num_perimeterpoints -1\n"
      "61.0.1 34.587434 -117.367061\n"
      "61.0.2 34.587436 -117.366847\n"
      "61.0.3 34.587440 -117.366444\n"
      "end_perimeter"
                                                    , false, 13, 4),
    // Invalid num_perimeterpoints value (> 32768).
    std::make_tuple(
      "\n\n"
      "perimeter 61.0\n"
      "num_perimeterpoints 32769\n"
      "61.0.1 34.587434 -117.367061\n"
      "61.0.2 34.587436 -117.366847\n"
      "61.0.3 34.587440 -117.366444\n"
      "end_perimeter"
                                                    , false, 14, 4),
    // Perimeterpoints missing.
    std::make_tuple(
      "\n\n"
      "perimeter 61.0\n"
      "num_perimeterpoints 3\n"
      "end_perimeter\n"
                                                    , false, 15, 5),

    // Invalid perimeterpoint (missing longitude).
    std::make_tuple(
      "\n\n"
      "perimeter 61.0\n"
      "num_perimeterpoints 3\n"
      "61.0.1 34.587434\n"
      "61.0.2 34.587436 -117.366847\n"
      "61.0.3 34.587440 -117.366444\n"
      "end_perimeter\n"
                                                    , false, 16, 5),
    // Invalid perimeterpoint (zone Id doesn't match).
    std::make_tuple(
      "\n\n"
      "perimeter 61.0\n"
      "num_perimeterpoints 3\n"
      "68.0.1 34.587434 -117.367061\n"
      "61.0.2 34.587436 -117.366847\n"
      "61.0.3 34.587440 -117.366444\n"
      "end_perimeter\n"
                                                    , false, 17, 5),
    // Invalid perimeterpoint (perimeter Id is not 0).
    std::make_tuple(
      "\n\n"
      "perimeter 61.0\n"
      "num_perimeterpoints 3\n"
      "61.8.1 34.587434 -117.367061\n"
      "61.0.2 34.587436 -117.366847\n"
      "61.0.3 34.587440 -117.366444\n"
      "end_perimeter\n"
                                                    , false, 18, 5),
    // Non-consecutive perimeterpoints.
    std::make_tuple(
      "\n/* comment */\n"
      "perimeter 61.0/* comment */\n"
      "num_perimeterpoints 3\n"
      "61.0.1 34.587434 -117.367061/* comment */\n"
      "61.0.2 34.587436 -117.366847 /* comment */  \n"
      "61.0.5 34.587440 -117.366444\n"
      "end_perimeter\n"
                                                    , false, 18, 7),
    // Missing terminator.
    std::make_tuple(
      "\n\n"
      "perimeter 61.0\n"
      "num_perimeterpoints 3\n"
      "61.0.1 34.587434 -117.367061\n"
      "61.0.2 34.587436 -117.366847\n"
      "61.0.3 34.587440 -117.366444\n"
                                                    , false, 19, 8),
    // Missing terminator and find the next label.
    std::make_tuple(
      "\n\n"
      "perimeter 61.0\n"
      "num_perimeterpoints 3\n"
      "61.0.1 34.587434 -117.367061\n"
      "61.0.2 34.587436 -117.366847\n"
      "61.0.3 34.587440 -117.366444\n"
      "end_zone"
                                                    , false, 20, 8),
    // Missing exit.
    std::make_tuple(
      "\n/* comment */\n"
      "perimeter 61.0/* comment */\n"
      "num_perimeterpoints 3\n"
      "61.0.2  34.1.1  /* onto Red_stub_Rd2 */\n"
      "61.0.1 34.587434 -117.367061/* comment */\n"
      "61.0.2 34.587436 -117.366847 /* comment */  \n"
      "61.0.3 34.587440 -117.366444\n"
      "end_perimeter\n"
                                                    , false, 21, 5),
    // Invalid zone Id on exit.
    std::make_tuple(
      "\n/* comment */\n"
      "perimeter 61.0/* comment */\n"
      "num_perimeterpoints 3\n"
      "exit 99.0.2  34.1.1  /* onto Red_stub_Rd2 */\n"
      "61.0.1 34.587434 -117.367061/* comment */\n"
      "61.0.2 34.587436 -117.366847 /* comment */  \n"
      "61.0.3 34.587440 -117.366444\n"
      "end_perimeter\n"
                                                    , false, 22, 5),
    // Invalid perimeter Id on exit.
    std::make_tuple(
      "\n/* comment */\n"
      "perimeter 61.0/* comment */\n"
      "num_perimeterpoints 3\n"
      "exit 61.9.2  34.1.1  /* onto Red_stub_Rd2 */\n"
      "61.0.1 34.587434 -117.367061/* comment */\n"
      "61.0.2 34.587436 -117.366847 /* comment */  \n"
      "61.0.3 34.587440 -117.366444\n"
      "end_perimeter\n"
                                                    , false, 23, 5),
    // Invalid entry Id on exit.
    std::make_tuple(
      "\n/* comment */\n"
      "perimeter 61.0/* comment */\n"
      "num_perimeterpoints 3\n"
      "exit 61.0.2  34.1  /* onto Red_stub_Rd2 */\n"
      "61.0.1 34.587434 -117.367061/* comment */\n"
      "61.0.2 34.587436 -117.366847 /* comment */  \n"
      "61.0.3 34.587440 -117.366444\n"
      "end_perimeter\n"
                                                    , false, 24, 5),
    // No options.
    std::make_tuple(
      "\n/* comment */\n"
      "perimeter 61.0/* comment */\n"
      "num_perimeterpoints 3\n"
      "61.0.1 34.587434 -117.367061/* comment */\n"
      "61.0.2 34.587436 -117.366847 /* comment */  \n"
      "61.0.3 34.587440 -117.366444\n"
      "end_perimeter\n"
                                                    , true, 25, 8),
    // One exit.
    std::make_tuple(
      "\n/* comment */\n"
      "perimeter 61.0/* comment */\n"
      "num_perimeterpoints 3\n"
      "exit  61.0.2  34.1.1  /* onto Red_stub_Rd2 */\n"
      "61.0.1 34.587434 -117.367061/* comment */\n"
      "61.0.2 34.587436 -117.366847 /* comment */  \n"
      "61.0.3 34.587440 -117.366444\n"
      "end_perimeter\n"
                                                    , true, 26, 9),
    // Two exits.
    std::make_tuple(
      "\n\n"
      "perimeter 61.0\n"
      "num_perimeterpoints 3\n"
      "exit  61.0.2  34.1.1  /* onto Red_stub_Rd2 */ \n"
      "exit  61.0.3  35.1.1  /* onto Red_stub_Rd3 */ \n"
      "61.0.1 34.587434 -117.367061\n"
      "61.0.2 34.587436 -117.366847\n"
      "61.0.3 34.587440 -117.366444\n"
      "end_perimeter\n"
                                                    , true, 27, 10),
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
    Perimeter perimeter;
    bool res;
    EXPECT_EQ(res = perimeter.Load(f, 61, line), expectedResult);
    EXPECT_EQ(line, expectedLine);
    if (res)
    {
      switch (testId)
      {
        case 25:
        {
          ASSERT_EQ(perimeter.NumPoints(), 3u);
          EXPECT_EQ(perimeter.Points().at(0).Id(), 1);
          EXPECT_EQ(perimeter.Points().at(1).Id(), 2);
          EXPECT_EQ(perimeter.Points().at(2).Id(), 3);
          EXPECT_EQ(perimeter.NumExits(), 0u);
          break;
        }
        case 26:
        {
          ASSERT_EQ(perimeter.NumPoints(), 3u);
          EXPECT_EQ(perimeter.Points().at(0).Id(), 1);
          EXPECT_EQ(perimeter.Points().at(1).Id(), 2);
          EXPECT_EQ(perimeter.Points().at(2).Id(), 3);
          ASSERT_EQ(perimeter.NumExits(), 1u);
          UniqueId exitId1(61, 0, 2);
          UniqueId entryId1(34, 1, 1);
          EXPECT_EQ(perimeter.Exits().at(0).ExitId(), exitId1);
          break;
        }
        case 27:
        {
          ASSERT_EQ(perimeter.NumPoints(), 3u);
          EXPECT_EQ(perimeter.Points().at(0).Id(), 1);
          EXPECT_EQ(perimeter.Points().at(1).Id(), 2);
          EXPECT_EQ(perimeter.Points().at(2).Id(), 3);
          ASSERT_EQ(perimeter.NumExits(), 2u);
          UniqueId exitId1(61, 0, 2);
          UniqueId exitId2(61, 0, 3);
          UniqueId entryId1(34, 1, 1);
          UniqueId entryId2(35, 1, 1);
          EXPECT_EQ(perimeter.Exits().at(0).ExitId(), exitId1);
          EXPECT_EQ(perimeter.Exits().at(0).EntryId(), entryId1);
          EXPECT_EQ(perimeter.Exits().at(1).ExitId(), exitId2);
          EXPECT_EQ(perimeter.Exits().at(1).EntryId(), entryId2);
          break;
        }
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
