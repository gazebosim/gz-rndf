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

#include <map>
#include <string>
#include <tuple>
#include <vector>
#include <ignition/math/Angle.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/SphericalCoordinates.hh>

#include "gtest/gtest.h"
#include "ignition/rndf/Waypoint.hh"
#include "ignition/rndf/test_config.h"

using namespace ignition;
using namespace rndf;

// The fixture for testing the Waypoint class.
class WaypointTest : public testing::FileParserUtils
{
};

//////////////////////////////////////////////////
/// \brief Check id-related accessors.
TEST(Waypoint, id)
{
  // Default surface type.
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);

  int id = 1;
  Waypoint waypoint(id, sc);

  EXPECT_EQ(waypoint.Id(), id);
  int newId = 2;
  EXPECT_TRUE(waypoint.SetId(newId));
  EXPECT_EQ(waypoint.Id(), newId);

  // Check that trying to set an incorrect Id does not take effect.
  int wrongId = -99;
  EXPECT_FALSE(waypoint.SetId(wrongId));
  EXPECT_EQ(waypoint.Id(), newId);

  // Check that using the constructor with a wrong id results creates an
  // invalid waypoint.
  Waypoint wrongWp(wrongId, sc);
  EXPECT_FALSE(wrongWp.Valid());
}

//////////////////////////////////////////////////
/// \brief Check location-related accessors.
TEST(Waypoint, location)
{
  // Default surface type.
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);

  int id = 1;
  Waypoint waypoint(id, sc);

  // Check that I can read and modify the location with the mutable accessor.
  auto &location = waypoint.Location();
  EXPECT_EQ(location, sc);
  double newElev = 1000.0;
  location.SetElevationReference(newElev);
  EXPECT_TRUE(
    ignition::math::equal(waypoint.Location().ElevationReference(), newElev));

  // Non-mutable accessor.
  const Waypoint nonMutableWaypoint(id, location);

  // Check that I can read and modify the location with the mutable accessor.
  auto &nonMutablelocation = nonMutableWaypoint.Location();
  EXPECT_EQ(nonMutablelocation, location);
}

//////////////////////////////////////////////////
/// \brief Check exit/entry accessors.
TEST(Waypoint, exitEntry)
{
  // Default surface type.
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);

  int id = 1;
  Waypoint waypoint(id, sc);

  EXPECT_FALSE(waypoint.IsEntry());
  EXPECT_FALSE(waypoint.IsExit());

  waypoint.SetEntry(true);

  EXPECT_TRUE(waypoint.IsEntry());
  EXPECT_FALSE(waypoint.IsExit());

  waypoint.SetExit(true);

  EXPECT_TRUE(waypoint.IsEntry());
  EXPECT_TRUE(waypoint.IsExit());

  waypoint.SetEntry(false);
  waypoint.SetExit(false);

  EXPECT_FALSE(waypoint.IsEntry());
  EXPECT_FALSE(waypoint.IsExit());
}

//////////////////////////////////////////////////
/// \brief Check function that validates the Id of a waypoint.
TEST(Waypoint, valid)
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
    // Default surface type
    ignition::math::SphericalCoordinates::SurfaceType st =
      ignition::math::SphericalCoordinates::EARTH_WGS84;
    ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
    double elev = 354.1;
    ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);

    int id = usecase.first;
    Waypoint wp(id, sc);
    EXPECT_EQ(wp.Valid(), usecase.second);
  }
}

//////////////////////////////////////////////////
/// \brief Check [in]equality operators.
TEST(Waypoint, equality)
{
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc1(st, lat, lon, elev, heading);
  ignition::math::SphericalCoordinates sc2(st, lat, lon, elev + 1, heading);

  int id1 = 1;
  Waypoint wp1(id1, sc1);

  int id2 = 2;
  Waypoint wp2(id2, sc2);

  Waypoint wp3(id1, sc2);
  Waypoint wp4(id2, sc1);

  EXPECT_FALSE(wp1 == wp2);
  EXPECT_TRUE(wp1 != wp2);

  EXPECT_TRUE(wp1 == wp3);
  EXPECT_FALSE(wp1 != wp3);

  EXPECT_FALSE(wp1 == wp4);
  EXPECT_TRUE(wp1 != wp4);
}

//////////////////////////////////////////////////
/// \brief Check assignment operator.
TEST(Waypoint, assignment)
{
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc1(st, lat, lon, elev, heading);
  ignition::math::SphericalCoordinates sc2(st, lat, lon, elev + 1, heading);

  int id1 = 1;
  Waypoint wp1(id1, sc1);
  int id2 = 2;
  Waypoint wp2(id2, sc2);
  EXPECT_NE(wp1, wp2);

  wp2 = wp1;
  EXPECT_EQ(wp1, wp2);
}

//////////////////////////////////////////////////
/// \brief Check loading a waypoint from a file.
TEST_F(WaypointTest, load)
{
  // The first element is the content to be parsed.
  // The second element is the expected return value.
  // The third element is the test Id.
  // The forth element is the expected line value.
  std::vector<std::tuple<std::string, bool, int, int>> testCases =
  {
    std::make_tuple(""                              , false, 0, 1),
    std::make_tuple("\n\n"                          , false, 1, 3),
    // Missing waypoint Id.
    std::make_tuple(
      "\n\n"
      "34.579979 -117.365607\n"
                                                    , false, 2, 3),
    // Invalid waypoint Id.
    std::make_tuple(
      "\n\n"
      "0.3.1 34.579979 -117.365607\n"
                                                    , false, 3, 3),
    // Invalid waypoint Id.
    std::make_tuple(
      "\n\n"
      "6.3.-1 34.579979 -117.365607\n"
                                                    , false, 3, 3),
    // Invalid waypoint Id.
    std::make_tuple(
      "\n\n"
      "6.3.32769 34.579979 -117.365607\n"
                                                    , false, 3, 3),

    // Unexpected waypoint Id (it should be 6.
    std::make_tuple(
      "\n\n"
      "5.3.1 34.579979 -117.365607\n"
                                                    , false, 3, 3),

    // Unexpected lane Id (it should be 3.
    std::make_tuple(
      "\n\n"
      "6.9.1 34.579979 -117.365607\n"
                                                    , false, 3, 3),

    // Missing latitude.
    std::make_tuple(
      "\n\n"
      "6.3.1 -117.365607\n"
                                                    , false, 4, 3),

    // Missing longitude.
    std::make_tuple(
      "\n\n"
      "6.3.1 34.579979\n"
                                                    , false, 5, 3),
    // Ivalid latitude.
    std::make_tuple(
      "\n\n"
      "6.3.1 xxx -117.365607\n"
                                                    , false, 6, 3),

    // Invalid longitude.
    std::make_tuple(
      "\n\n"
      "6.3.1 34.579979 xxx\n"
                                                    , false, 7, 3),
    std::make_tuple(
      "\n\n"
      "6.3.1  34.579979   -117.365607 /* a comment  */ \n"
                                                    , true, 8, 3),
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
    Waypoint w;
    bool res;
    EXPECT_EQ(res = w.Load(f, 6, 3, line), expectedResult);
    EXPECT_EQ(line, expectedLine);
    if (res)
    {
      switch (testId)
      {
        case 8:
        {
          ignition::math::SphericalCoordinates::SurfaceType st =
            ignition::math::SphericalCoordinates::EARTH_WGS84;
          ignition::math::Angle lat(IGN_DTOR(34.579979));
          ignition::math::Angle lon(IGN_DTOR(-117.365607));
          ignition::math::Angle heading(0.0);
          double elev = 0.0;
          ignition::math::SphericalCoordinates loc(st, lat, lon, elev, heading);

          EXPECT_EQ(w.Id(), 1);
          EXPECT_EQ(w.Location(), loc);
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
