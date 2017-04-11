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

#include <string>
#include <vector>
#include <ignition/math/Helpers.hh>
#include <ignition/math/SphericalCoordinates.hh>

#include "gtest/gtest.h"
#include "ignition/rndf/test_config.h"
#include "ignition/rndf/Lane.hh"
#include "ignition/rndf/Segment.hh"
#include "ignition/rndf/Waypoint.hh"

using namespace ignition;
using namespace rndf;

// The fixture for testing the Segment class.
class SegmentTest : public testing::FileParserUtils
{
};

//////////////////////////////////////////////////
/// \brief Check id-related accessors.
TEST(Segment, id)
{
  int id = 1;
  Segment segment(id);

  EXPECT_EQ(segment.Id(), id);
  int newId = 2;
  EXPECT_TRUE(segment.SetId(newId));
  EXPECT_EQ(segment.Id(), newId);

  // Check that trying to set an incorrect Id does not take effect.
  int wrongId = -1;
  EXPECT_FALSE(segment.SetId(wrongId));
  EXPECT_EQ(segment.Id(), newId);

  // Check that using the constructor with a wrong id results in an invalid
  // segment.
  Segment wrongSegment(wrongId);
  EXPECT_FALSE(wrongSegment.Valid());
}

//////////////////////////////////////////////////
/// \brief Check lanes-related functions.
TEST(Segment, waypoints)
{
  int id = 1;
  Segment segment(id);

  EXPECT_EQ(segment.NumLanes(), 0u);
  Lane lane(id);
  // Check an inexistent lane id.
  EXPECT_FALSE(segment.Lane(id, lane));
  // Try to remove an inexistent lane id.
  EXPECT_FALSE(segment.RemoveLane(id));
  // Try to add a lane with an invalid Id.
  EXPECT_FALSE(segment.AddLane(lane));

  // Create a valid waypoint.
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);
  int waypointId = 1;
  Waypoint wp;
  wp.SetId(waypointId);
  wp.Location() = sc;

  // Create a valid lane.
  EXPECT_TRUE(lane.AddWaypoint(wp));
  EXPECT_EQ(lane.NumWaypoints(), 1u);

  // Add the lane to the segment
  EXPECT_TRUE(segment.AddLane(lane));
  EXPECT_EQ(segment.NumLanes(), 1u);

  // Try to add an existent lane.
  EXPECT_FALSE(segment.AddLane(lane));
  EXPECT_EQ(segment.NumLanes(), 1u);

  // Get the Lane.
  Lane lane2;
  EXPECT_TRUE(segment.Lane(lane.Id(), lane2));
  EXPECT_EQ(lane, lane2);

  // Update a lane.
  double newElevation = 2000;
  lane2.Waypoints().at(0).Location().SetElevationReference(newElevation);
  EXPECT_TRUE(segment.UpdateLane(lane2));
  Lane lane3;
  EXPECT_TRUE(segment.Lane(lane2.Id(), lane3));
  EXPECT_EQ(lane3, lane2);

  // Get a mutable reference to all lanes.
  std::vector<Lane> &lanes = segment.Lanes();
  ASSERT_EQ(lanes.size(), 1u);
  // Modify a lane.
  Lane &aLane = lanes.at(0);
  aLane.Waypoints().at(0).Location().SetElevationReference(500.0);
  EXPECT_TRUE(segment.Lane(lane2.Id(), lane3));
  EXPECT_TRUE(ignition::math::equal(
    lane3.Waypoints().at(0).Location().ElevationReference(), 500.0));

  for (auto const &l : segment.Lanes())
    EXPECT_TRUE(l.Valid());

  // Remove a lane.
  EXPECT_TRUE(segment.RemoveLane(lane2.Id()));
  EXPECT_EQ(segment.NumLanes(), 0u);
}

//////////////////////////////////////////////////
/// \brief Check segment name.
TEST(Segment, Name)
{
  int id = 1;
  Segment segment(id);
  EXPECT_TRUE(segment.Name().empty());

  std::string name = "Wisconsin_Ave";
  segment.SetName(name);
  EXPECT_EQ(segment.Name(), name);
}

//////////////////////////////////////////////////
/// \brief Check segment validation.
TEST(Segment, Validation)
{
  int id = 1;
  Segment segment(id);
  EXPECT_TRUE(segment.Name().empty());
  EXPECT_FALSE(segment.Valid());

  std::string name = "Wisconsin_Ave";
  segment.SetName(name);
  EXPECT_EQ(segment.Name(), name);
  EXPECT_FALSE(segment.Valid());

  // Create a waypoint.
  ignition::math::SphericalCoordinates::SurfaceType st =
    ignition::math::SphericalCoordinates::EARTH_WGS84;
  ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
  double elev = 354.1;
  ignition::math::SphericalCoordinates sc(st, lat, lon, elev, heading);
  int waypointId = 1;
  Waypoint wp;
  wp.SetId(waypointId);
  wp.Location() = sc;

  // Create a valid lane.
  Lane lane(id);
  EXPECT_TRUE(lane.AddWaypoint(wp));
  EXPECT_EQ(lane.NumWaypoints(), 1u);

  // Add the lane to the segment
  EXPECT_TRUE(segment.AddLane(lane));
  EXPECT_EQ(segment.NumLanes(), 1u);

  EXPECT_TRUE(segment.Valid());
}

//////////////////////////////////////////////////
/// \brief Check [in]equality operators.
TEST(Segment, equality)
{
  int id1 = 1;
  Segment segment1(id1);

  int id2 = 2;
  Segment segment2(id2);

  Segment segment3(id1);

  EXPECT_FALSE(segment1 == segment2);
  EXPECT_TRUE(segment1 != segment2);

  EXPECT_TRUE(segment1 == segment3);
  EXPECT_FALSE(segment1 != segment3);
}

//////////////////////////////////////////////////
/// \brief Check assignment operator.
TEST(Segment, assignment)
{
  int id1 = 1;
  Segment segment1(id1);

  int id2 = 2;
  Segment segment2(id2);
  EXPECT_NE(segment1, segment2);

  segment2 = segment1;
  EXPECT_EQ(segment1, segment2);
}

//////////////////////////////////////////////////
/// \brief Check loading a segment from a text file.
TEST_F(SegmentTest, Load)
{
  // The first element is the content to be parsed.
  // The second element is the expected return value.
  // The third element is the test Id.
  // The forth element is the expected line value.
  std::vector<std::tuple<std::string, bool, int, int>> testCases =
  {
    std::make_tuple(""                              , false, 0, 1),
    std::make_tuple("\n\n"                          , false, 0, 3),
    // Missing segment.
    std::make_tuple(
      "\n\n"
      "xxx 60\n"
      "num_lanes 1\n"
      "segment_name  Red_zone_Wlot_Access_Rd\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
      "end_segment\n"
                                                    , false, 0, 3),
    // Missing segment Id.
    std::make_tuple(
      "\n\n"
      "segment \n"
      "num_lanes 1\n"
      "segment_name  Red_zone_Wlot_Access_Rd\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
      "end_segment\n"
                                                    , false, 0, 3),
    // Invalid segment Id.
    std::make_tuple(
      "\n\n"
      "segment xxx\n"
      "num_lanes 1\n"
      "segment_name  Red_zone_Wlot_Access_Rd\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
      "end_segment\n"
                                                    , false, 0, 3),
    // num_lanes missing.
    std::make_tuple(
      "\n\n"
      "segment 60\n"
      "segment_name  Red_zone_Wlot_Access_Rd\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
      "end_segment\n"
                                                    , false, 0, 4),
    // Missing num_lanes value.
    std::make_tuple(
      "\n\n"
      "segment 60\n"
      "num_lanes \n"
      "segment_name  Red_zone_Wlot_Access_Rd\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
      "end_segment\n"
                                                    , false, 0, 4),
    // Invalid num_lanes value.
    std::make_tuple(
      "\n\n"
      "segment 60\n"
      "num_lanes 0\n"
      "segment_name  Red_zone_Wlot_Access_Rd\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
      "end_segment\n"
                                                    , false, 0, 4),
    // Invalid num_lanes value.
    std::make_tuple(
      "\n\n"
      "segment 60\n"
      "num_lanes -1\n"
      "segment_name  Red_zone_Wlot_Access_Rd\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
      "end_segment\n"
                                                    , false, 0, 4),
    // Missing segment_name.
    std::make_tuple(
      "\n\n"
      "segment 60\n"
      "num_lanes 1\n"
      "Red_zone_Wlot_Access_Rd\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
      "end_segment\n"
                                                    , false, 0, 5),
    // Missing segment_name value.
    std::make_tuple(
      "\n\n"
      "segment 60\n"
      "num_lanes 1\n"
      "segment_name  \n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
      "end_segment\n"
                                                    , false, 0, 5),
    // Missing most of the fields.
    std::make_tuple(
      "\n\n"
      "segment 60\n"
      "num_lanes 1\n"
                                                    , false, 0, 5),
    // Missing "end_segment" terminator.
    std::make_tuple(
      "\n\n"
      "segment 60\n"
      "num_lanes 1\n"
      "segment_name  Red_zone_Wlot_Access_Rd\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
                                                     , false, 0, 16),
    // Wrong "end_segment" terminator.
    std::make_tuple(
      "\n\n"
      "segment 60\n"
      "num_lanes 1\n"
      "segment_name  Red_zone_Wlot_Access_Rd\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
      "end\n"
                                                    , false, 0, 16),
    // Missing "end_segment" terminator and found the next segment.
    std::make_tuple(
      "\n\n"
      "segment 60\n"
      "num_lanes 1\n"
      "segment_name  Red_zone_Wlot_Access_Rd\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
      "segment 61\n"
                                                    , false, 0, 16),
    // Missing "lanes".
    std::make_tuple(
      "\n\n"
      "segment 60\n"
      "num_lanes 1\n"
      "segment_name  Red_zone_Wlot_Access_Rd\n"
      "end_segment\n"
                                                    , false, 0, 6),
    // More "lanes" than the expected number.
    std::make_tuple(
      "\n\n"
      "segment 60\n"
      "num_lanes 1\n"
      "segment_name  Red_zone_Wlot_Access_Rd\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
      "lane  60.2\n"
      "num_waypoints 1\n"
      "lane_width  15\n"
      "exit  60.2.1  63.0.6  /* into Red zone West */\n"
      "60.2.1  34.583562 -117.347396\n"
      "end_lane\n"
      "end_segment\n"
                                                    , false, 0, 16),
    // Invalid segment Id in the lane.
    std::make_tuple(
      "\n\n"
      "segment 60\n"
      "num_lanes 1\n"
      "segment_name  Red_zone_Wlot_Access_Rd\n"
      "lane  64.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
      "end_segment\n"
                                                    , false, 0, 6),
    // Non-consecutive lanes.
    std::make_tuple(
      "\n/* comment */\n"
      "segment 60\n"
      "num_lanes 2\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
      "lane  60.3\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.3.5  63.0.6  /* into Red zone West */\n"
      "60.3.1  34.587562 -117.367396\n"
      "60.3.2  34.587560 -117.367631\n"
      "60.3.3  34.587557 -117.367941\n"
      "60.3.4  34.587469 -117.368018\n"
      "60.3.5  34.587116 -117.368016\n"
      "end_lane\n"
      "end_segment\n"
                                                    , false, 1, 24),
    // No options.
    std::make_tuple(
      "\n/* comment */\n"
      "segment 60\n"
      "num_lanes 1\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
      "end_segment\n"
                                                    , true, 1, 15),
    // Name option.
    std::make_tuple(
      "\n/* comment */\n"
      "segment 60\n"
      "num_lanes 1\n"
      "segment_name  Red_zone_Wlot_Access_Rd\n"
      "lane  60.1\n"
      "num_waypoints 5\n"
      "lane_width  15\n"
      "exit  60.1.5  63.0.6  /* into Red zone West */\n"
      "60.1.1  34.587562 -117.367396\n"
      "60.1.2  34.587560 -117.367631\n"
      "60.1.3  34.587557 -117.367941\n"
      "60.1.4  34.587469 -117.368018\n"
      "60.1.5  34.587116 -117.368016\n"
      "end_lane\n"
      "end_segment\n"
                                                    , true, 2, 16),
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
    Segment segment;
    bool res;
    EXPECT_EQ(res = segment.Load(f, line), expectedResult);
    EXPECT_EQ(line, expectedLine);
    if (res)
    {
      switch (testId)
      {
        case 1:
          EXPECT_EQ(segment.Id(), 60);
          EXPECT_EQ(segment.NumLanes(), 1u);
          EXPECT_TRUE(segment.Name().empty());
          break;
        case 2:
          EXPECT_EQ(segment.Id(), 60);
          ASSERT_EQ(segment.NumLanes(), 1u);
          ASSERT_EQ(segment.Name(), "Red_zone_Wlot_Access_Rd");
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
