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
#include "ignition/rndf/ParkingSpot.hh"
#include "ignition/rndf/Perimeter.hh"
#include "ignition/rndf/Waypoint.hh"
#include "ignition/rndf/Zone.hh"

using namespace ignition;
using namespace rndf;

// The fixture for testing the Zone class.
class ZoneTest : public testing::FileParserUtils
{
};

//////////////////////////////////////////////////
/// \brief Check id-related accessors.
TEST(Zone, id)
{
  int id = 1;
  Zone zone(id);

  EXPECT_EQ(zone.Id(), id);
  int newId = 2;
  EXPECT_TRUE(zone.SetId(newId));
  EXPECT_EQ(zone.Id(), newId);

  // Check that trying to set an incorrect Id does not take effect.
  int wrongId = -1;
  EXPECT_FALSE(zone.SetId(wrongId));
  EXPECT_EQ(zone.Id(), newId);

  // Check that using the constructor with a wrong id results in an invalid zone
  Zone wrongZone(wrongId);
  EXPECT_FALSE(wrongZone.Valid());
}

//////////////////////////////////////////////////
/// \brief Check parking spots-related functions.
TEST(Zone, spots)
{
  int id = 1;
  Zone zone(id);

  EXPECT_EQ(zone.NumSpots(), 0u);
  ParkingSpot ps;
  // Check an inexistent parking spot Id.
  EXPECT_FALSE(zone.Spot(id, ps));
  // Try to remove an inexistent spot id.
  EXPECT_FALSE(zone.RemoveSpot(id));
  // Try to add a parking spot with an invalid Id.
  EXPECT_FALSE(zone.AddSpot(ps));

  // Create a valid parking spot.
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);
  Waypoint wp1;
  wp1.SetId(1);
  wp1.Location() = sc;
  Waypoint wp2;
  wp2.SetId(2);
  wp2.Location() = sc;
  ps.SetId(1);
  ps.AddWaypoint(wp1);
  ps.AddWaypoint(wp2);
  EXPECT_EQ(ps.NumWaypoints(), 2u);

  // Add a valid parking spot.
  EXPECT_TRUE(zone.AddSpot(ps));
  EXPECT_EQ(zone.NumSpots(), 1u);

  // Try to add an existent parking spot.
  EXPECT_FALSE(zone.AddSpot(ps));
  EXPECT_EQ(zone.NumSpots(), 1u);

  // Get the parking spot.
  ParkingSpot ps2;
  EXPECT_TRUE(zone.Spot(ps.Id(), ps2));
  EXPECT_EQ(ps, ps2);
  EXPECT_EQ(ps2.NumWaypoints(), 2u);

  // Update a parking spot.
  double newElevation = 2000;
  ASSERT_GT(ps2.NumWaypoints(), 0u);
  ps2.Waypoints().at(0).Location().SetElevationReference(newElevation);
  EXPECT_TRUE(zone.UpdateSpot(ps2));
  ParkingSpot ps3;
  EXPECT_TRUE(zone.Spot(ps2.Id(), ps3));
  EXPECT_EQ(ps3, ps2);

  // Get a mutable reference to all parking spots.
  std::vector<ParkingSpot> &spots = zone.Spots();
  ASSERT_EQ(spots.size(), 1u);
  // Modify a parking spot.
  ParkingSpot &aPs = spots.at(0);
  aPs.Waypoints().at(0).Location().SetElevationReference(500.0);
  EXPECT_TRUE(zone.Spot(ps2.Id(), ps3));
  EXPECT_TRUE(ignition::math::equal(
    ps3.Waypoints().at(0).Location().ElevationReference(), 500.0));

  for (auto const &aSpot : zone.Spots())
    EXPECT_TRUE(aSpot.Valid());

  // Remove a parking spot.
  EXPECT_TRUE(zone.RemoveSpot(ps2.Id()));
  EXPECT_EQ(zone.NumSpots(), 0u);
}

//////////////////////////////////////////////////
/// \brief Check perimeter-related functions.
TEST(Zone, perimeter)
{
  int id = 1;
  Zone zone(id);

  // Add a perimeter point.
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);
  int waypointId = 1;
  Waypoint wp;
  wp.SetId(waypointId);
  wp.Location() = sc;

  EXPECT_TRUE(zone.Perimeter().AddPoint(wp));
  EXPECT_EQ(zone.Perimeter().Points().size(), 1u);
}

//////////////////////////////////////////////////
/// \brief Check zone name.
TEST(Zone, Name)
{
  int id = 1;
  Zone zone(id);
  EXPECT_TRUE(zone.Name().empty());

  std::string name = "North_parking_lot";
  zone.SetName(name);
  EXPECT_EQ(zone.Name(), name);
}

//////////////////////////////////////////////////
/// \brief Check zone validation.
TEST(Zone, Validation)
{
  int id = 1;
  Zone zone(id);
  EXPECT_TRUE(zone.Name().empty());
  EXPECT_FALSE(zone.Valid());

  std::string name = "North_parking_lot";
  zone.SetName(name);
  EXPECT_EQ(zone.Name(), name);
  EXPECT_FALSE(zone.Valid());

  // Add a perimeter point.
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);
  int waypointId = 1;
  Waypoint wp;
  wp.SetId(waypointId);
  wp.Location() = sc;

  EXPECT_TRUE(zone.Perimeter().AddPoint(wp));
  EXPECT_TRUE(zone.Valid());

  // Create a valid parking spot.
  ParkingSpot ps;
  Waypoint wp1;
  wp1.SetId(1);
  wp1.Location() = sc;
  Waypoint wp2;
  wp2.SetId(2);
  wp2.Location() = sc;
  ps.SetId(1);
  ps.AddWaypoint(wp1);
  ps.AddWaypoint(wp2);
  EXPECT_EQ(ps.NumWaypoints(), 2u);

  // Add a valid parking spot.
  EXPECT_TRUE(zone.AddSpot(ps));
  EXPECT_EQ(zone.NumSpots(), 1u);
  EXPECT_TRUE(zone.Valid());

  // Adding a non-consecutive parking spot is OK but the zone is not valid.
  ParkingSpot ps2;
  ps2.SetId(3);
  ps2.AddWaypoint(wp1);
  ps2.AddWaypoint(wp2);
  EXPECT_EQ(ps2.NumWaypoints(), 2u);
  EXPECT_TRUE(zone.AddSpot(ps2));
  EXPECT_EQ(zone.NumSpots(), 2u);
  EXPECT_FALSE(zone.Valid());
}

//////////////////////////////////////////////////
/// \brief Check [in]equality operators.
TEST(Zone, equality)
{
  int id1 = 1;
  Zone zone1(id1);

  int id2 = 2;
  Zone zone2(id2);

  Zone zone3(id1);

  EXPECT_FALSE(zone1 == zone2);
  EXPECT_TRUE(zone1 != zone2);

  EXPECT_TRUE(zone1 == zone3);
  EXPECT_FALSE(zone1 != zone3);
}

//////////////////////////////////////////////////
/// \brief Check assignment operator.
TEST(Zone, assignment)
{
  int id1 = 1;
  Zone zone1(id1);

  int id2 = 2;
  Zone zone2(id2);
  EXPECT_NE(zone1, zone2);

  zone2 = zone1;
  EXPECT_EQ(zone1, zone2);
}

//////////////////////////////////////////////////
/// \brief Check loading a zone from a text file.
TEST_F(ZoneTest, Load)
{
  // The first element is the content to be parsed.
  // The second element is the expected return value.
  // The third element is the test Id.
  // The forth element is the expected line value.
  std::vector<std::tuple<std::string, bool, int, int>> testCases =
  {
    std::make_tuple(""                              , false, 0, 1),
    std::make_tuple("\n\n"                          , false, 0, 3),
    // Missing zone.
    std::make_tuple(
      "\n\n"
      "xxx 67\n"
      "num_spots 0\n"
      "zone_name Blue_zone_south\n"
      "perimeter 67.0\n"
      "num_perimeterpoints 3\n"
      "exit  67.0.2  22.1.1  /* leaving Blue zone south onto Mississippi */\n"
      "67.0.1  34.581322 -117.362619\n"
      "67.0.2  34.581078 -117.361760\n"
      "67.0.3  34.580865 -117.361852\n"
      "end_perimeter\n"
      "end_zone\n"
                                                    , false, 0, 3),
    // Missing zone Id.
    std::make_tuple(
      "\n\n"
      "zone \n"
      "num_spots 0\n"
      "zone_name Blue_zone_south\n"
      "perimeter 67.0\n"
      "num_perimeterpoints 3\n"
      "exit  67.0.2  22.1.1  /* leaving Blue zone south onto Mississippi */\n"
      "67.0.1  34.581322 -117.362619\n"
      "67.0.2  34.581078 -117.361760\n"
      "67.0.3  34.580865 -117.361852\n"
      "end_perimeter\n"
      "end_zone\n"
                                                    , false, 0, 3),
    // Invalid zone Id.
    std::make_tuple(
      "\n\n"
      "zone  xxx\n"
      "num_spots 0\n"
      "zone_name Blue_zone_south\n"
      "perimeter 67.0\n"
      "num_perimeterpoints 3\n"
      "exit  67.0.2  22.1.1  /* leaving Blue zone south onto Mississippi */\n"
      "67.0.1  34.581322 -117.362619\n"
      "67.0.2  34.581078 -117.361760\n"
      "67.0.3  34.580865 -117.361852\n"
      "end_perimeter\n"
      "end_zone\n"
                                                    , false, 0, 3),
    // num_spots missing.
    std::make_tuple(
      "\n\n"
      "zone  67\n"
      "zone_name Blue_zone_south\n"
      "perimeter 67.0\n"
      "num_perimeterpoints 3\n"
      "exit  67.0.2  22.1.1  /* leaving Blue zone south onto Mississippi */\n"
      "67.0.1  34.581322 -117.362619\n"
      "67.0.2  34.581078 -117.361760\n"
      "67.0.3  34.580865 -117.361852\n"
      "end_perimeter\n"
      "end_zone\n"
                                                    , false, 0, 4),
    // Missing num_spots value.
    std::make_tuple(
      "\n\n"
      "zone  67\n"
      "num_spots \n"
      "zone_name Blue_zone_south\n"
      "perimeter 67.0\n"
      "num_perimeterpoints 3\n"
      "exit  67.0.2  22.1.1  /* leaving Blue zone south onto Mississippi */\n"
      "67.0.1  34.581322 -117.362619\n"
      "67.0.2  34.581078 -117.361760\n"
      "67.0.3  34.580865 -117.361852\n"
      "end_perimeter\n"
      "end_zone\n"
                                                    , false, 0, 4),
    // Invalid num_spots value.
    std::make_tuple(
      "\n\n"
      "zone  67\n"
      "num_spots xxx\n"
      "zone_name Blue_zone_south\n"
      "perimeter 67.0\n"
      "num_perimeterpoints 3\n"
      "exit  67.0.2  22.1.1  /* leaving Blue zone south onto Mississippi */\n"
      "67.0.1  34.581322 -117.362619\n"
      "67.0.2  34.581078 -117.361760\n"
      "67.0.3  34.580865 -117.361852\n"
      "end_perimeter\n"
      "end_zone\n"
                                                    , false, 0, 4),
    // Invalid num_spots value.
    std::make_tuple(
      "\n\n"
      "zone  67\n"
      "num_spots -1\n"
      "zone_name Blue_zone_south\n"
      "perimeter 67.0\n"
      "num_perimeterpoints 3\n"
      "exit  67.0.2  22.1.1  /* leaving Blue zone south onto Mississippi */\n"
      "67.0.1  34.581322 -117.362619\n"
      "67.0.2  34.581078 -117.361760\n"
      "67.0.3  34.580865 -117.361852\n"
      "end_perimeter\n"
      "end_zone\n"
                                                    , false, 0, 4),
    // Missing zone_name.
    std::make_tuple(
      "\n\n"
      "zone  67\n"
      "num_spots 0\n"
      "Blue_zone_south\n"
      "perimeter 67.0\n"
      "num_perimeterpoints 3\n"
      "exit  67.0.2  22.1.1  /* leaving Blue zone south onto Mississippi */\n"
      "67.0.1  34.581322 -117.362619\n"
      "67.0.2  34.581078 -117.361760\n"
      "67.0.3  34.580865 -117.361852\n"
      "end_perimeter\n"
      "end_zone\n"
                                                    , false, 0, 5),
    // Missing zone_name value.
    std::make_tuple(
      "\n\n"
      "zone  67\n"
      "num_spots 0\n"
      "zone_name \n"
      "perimeter 67.0\n"
      "num_perimeterpoints 3\n"
      "exit  67.0.2  22.1.1  /* leaving Blue zone south onto Mississippi */\n"
      "67.0.1  34.581322 -117.362619\n"
      "67.0.2  34.581078 -117.361760\n"
      "67.0.3  34.580865 -117.361852\n"
      "end_perimeter\n"
      "end_zone\n"
                                                    , false, 0, 5),
    // Missing most of the fields.
    std::make_tuple(
      "\n\n"
      "zone  67\n"
      "num_spots 0\n"
                                                    , false, 0, 5),
    // Missing "end_zone" terminator.
    std::make_tuple(
      "\n\n"
      "zone  67\n"
      "num_spots 0\n"
      "zone_name Blue_zone_south\n"
      "perimeter 67.0\n"
      "num_perimeterpoints 3\n"
      "exit  67.0.2  22.1.1  /* leaving Blue zone south onto Mississippi */\n"
      "67.0.1  34.581322 -117.362619\n"
      "67.0.2  34.581078 -117.361760\n"
      "67.0.3  34.580865 -117.361852\n"
      "end_perimeter\n"
                                                     , false, 0, 13),
    // Wrong "end_zone" terminator.
    std::make_tuple(
      "\n\n"
      "zone  67\n"
      "num_spots 0\n"
      "zone_name Blue_zone_south\n"
      "perimeter 67.0\n"
      "num_perimeterpoints 3\n"
      "exit  67.0.2  22.1.1  /* leaving Blue zone south onto Mississippi */\n"
      "67.0.1  34.581322 -117.362619\n"
      "67.0.2  34.581078 -117.361760\n"
      "67.0.3  34.580865 -117.361852\n"
      "end_perimeter\n"
      "end\n"
                                                    , false, 0, 13),
    // Missing "end_zone" terminator and found the next zone.
    std::make_tuple(
      "\n\n"
      "zone  67\n"
      "num_spots 0\n"
      "zone_name Blue_zone_south\n"
      "perimeter 67.0\n"
      "num_perimeterpoints 3\n"
      "exit  67.0.2  22.1.1  /* leaving Blue zone south onto Mississippi */\n"
      "67.0.1  34.581322 -117.362619\n"
      "67.0.2  34.581078 -117.361760\n"
      "67.0.3  34.580865 -117.361852\n"
      "end_perimeter\n"
      "zone 68\n"
                                                    , false, 0, 13),
    // Missing "spots".
    std::make_tuple(
      "\n\n"
      "zone  67\n"
      "num_spots 1\n"
      "zone_name Blue_zone_south\n"
      "perimeter 67.0\n"
      "num_perimeterpoints 3\n"
      "exit  67.0.2  22.1.1  /* leaving Blue zone south onto Mississippi */\n"
      "67.0.1  34.581322 -117.362619\n"
      "67.0.2  34.581078 -117.361760\n"
      "67.0.3  34.580865 -117.361852\n"
      "end_perimeter\n"
      "end_zone\n"
                                                    , false, 0, 13),
    // More "spots" than the expected number.
    std::make_tuple(
      "\n\n"
      "zone  67\n"
      "num_spots 0\n"
      "zone_name Blue_zone_south\n"
      "perimeter 67.0\n"
      "num_perimeterpoints 3\n"
      "exit  67.0.2  22.1.1  /* leaving Blue zone south onto Mississippi */\n"
      "67.0.1  34.581322 -117.362619\n"
      "67.0.2  34.581078 -117.361760\n"
      "67.0.3  34.580865 -117.361852\n"
      "end_perimeter\n"
      "spot  67.1\n"
      "spot_width  12\n"
      "checkpoint  67.1.2  76\n"
      "67.1.1  34.583721 -117.368955 /* Finish Spot Waypoint */\n"
      "67.1.2  34.583738 -117.368955 /* Finish Spot Checkpoint */\n"
      "end_spot\n"
      "end_zone\n"
                                                    , false, 0, 13),
    // Invalid zone Id in the spot.
    std::make_tuple(
      "\n\n"
      "zone  67\n"
      "num_spots 1\n"
      "zone_name Blue_zone_south\n"
      "perimeter 67.0\n"
      "num_perimeterpoints 3\n"
      "exit  67.0.2  22.1.1  /* leaving Blue zone south onto Mississippi */\n"
      "67.0.1  34.581322 -117.362619\n"
      "67.0.2  34.581078 -117.361760\n"
      "67.0.3  34.580865 -117.361852\n"
      "end_perimeter\n"
      "spot  68.1\n"
      "spot_width  12\n"
      "checkpoint  67.1.2  76\n"
      "68.1.1  34.583721 -117.368955 /* Finish Spot Waypoint */\n"
      "68.1.2  34.583738 -117.368955 /* Finish Spot Checkpoint */\n"
      "end_spot\n"
      "end_zone\n"
                                                    , false, 0, 13),
    // Non-consecutive parking spots.
    std::make_tuple(
      "\n/* comment */\n"
      "zone  67\n"
      "num_spots 2\n"
      "perimeter 67.0\n"
      "num_perimeterpoints 3\n"
      "exit  67.0.2  22.1.1  /* leaving Blue zone south onto Mississippi */\n"
      "67.0.1  34.581322 -117.362619\n"
      "67.0.2  34.581078 -117.361760\n"
      "67.0.3  34.580865 -117.361852\n"
      "end_perimeter\n"
      "spot  67.1\n"
      "spot_width  12\n"
      "checkpoint  67.1.2  76\n"
      "67.1.1  34.583721 -117.368955 /* Finish Spot Waypoint */\n"
      "67.1.2  34.583738 -117.368955 /* Finish Spot Checkpoint */\n"
      "end_spot\n"
      "spot  67.3\n"
      "spot_width  12\n"
      "checkpoint  67.3.2  76\n"
      "67.3.1  34.583721 -117.368955 /* Finish Spot Waypoint */\n"
      "67.3.2  34.583738 -117.368955 /* Finish Spot Checkpoint */\n"
      "end_spot\n"
      "end_zone\n"
                                                    , false, 1, 23),
    // Invalid perimeter.
    std::make_tuple(
      "\n/* comment */\n"
      "zone  67\n"
      "num_spots 0\n"
      "perimeter 67.0\n"
      "num_perimeterpoints 5\n"
      "exit  67.0.2  22.1.1  /* leaving Blue zone south onto Mississippi */\n"
      "67.0.1  34.581322 -117.362619\n"
      "67.0.2  34.581078 -117.361760\n"
      "67.0.3  34.580865 -117.361852\n"
      "end_perimeter\n"
      "end_zone\n"
                                                    , false, 1, 11),
    // No options.
    std::make_tuple(
      "\n/* comment */\n"
      "zone  67\n"
      "num_spots 0\n"
      "perimeter 67.0\n"
      "num_perimeterpoints 3\n"
      "exit  67.0.2  22.1.1  /* leaving Blue zone south onto Mississippi */\n"
      "67.0.1  34.581322 -117.362619\n"
      "67.0.2  34.581078 -117.361760\n"
      "67.0.3  34.580865 -117.361852\n"
      "end_perimeter\n"
      "end_zone\n"
                                                    , true, 1, 12),
    // Name option.
    std::make_tuple(
      "\n/* comment */\n"
      "zone  67  /* comment */ \n"
      "num_spots 1  /* comment */   \n"
      "zone_name Blue_zone_south  /* comment */\n"
      "perimeter 67.0/* comment */\n"
      "num_perimeterpoints 3  /* comment */\n"
      "exit  67.0.2  22.1.1  /* leaving Blue zone south onto Mississippi */\n"
      "67.0.1  34.581322 -117.362619 /* comment */\n"
      "67.0.2  34.581078 -117.361760  /* comment */\n"
      "67.0.3  34.580865 -117.361852/* comment */\n"
      "end_perimeter/* comment */\n"
      "spot  67.1\n"
      "spot_width  12\n"
      "checkpoint  67.1.2  76\n"
      "67.1.1  34.583721 -117.368955 /* Finish Spot Waypoint */\n"
      "67.1.2  34.583738 -117.368955 /* Finish Spot Checkpoint */\n"
      "end_spot\n"
      "end_zone  /* comment */\n"
                                                    , true, 2, 19),
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
    Zone zone;
    bool res;
    EXPECT_EQ(res = zone.Load(f, line), expectedResult);
    EXPECT_EQ(line, expectedLine);
    if (res)
    {
      switch (testId)
      {
        case 1:
          EXPECT_EQ(zone.Id(), 67);
          EXPECT_EQ(zone.NumSpots(), 0u);
          EXPECT_EQ(zone.Perimeter().NumPoints(), 3u);
          break;
        case 2:
          EXPECT_EQ(zone.Id(), 67);
          ASSERT_EQ(zone.NumSpots(), 1u);
          ASSERT_EQ(zone.Perimeter().NumPoints(), 3u);
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
