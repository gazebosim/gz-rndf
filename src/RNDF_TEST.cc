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
#include <vector>
#include <tuple>
#include <ignition/math/Helpers.hh>
#include <ignition/math/SphericalCoordinates.hh>

#include "gtest/gtest.h"
#include "ignition/rndf/test_config.h"
#include "ignition/rndf/Lane.hh"
#include "ignition/rndf/Perimeter.hh"
#include "ignition/rndf/RNDF.hh"
#include "ignition/rndf/RNDFNode.hh"
#include "ignition/rndf/Segment.hh"
#include "ignition/rndf/UniqueId.hh"
#include "ignition/rndf/Waypoint.hh"
#include "ignition/rndf/Zone.hh"

using namespace ignition;
using namespace rndf;

// The fixture for testing the RNDF class.
class RNDFTest : public testing::FileParserUtils
{
};

//////////////////////////////////////////////////
/// \brief Check RNDF name.
TEST(RNDF, Name)
{
  RNDF rndf;
  EXPECT_TRUE(rndf.Name().empty());

  std::string name = "test.rndf";
  rndf.SetName(name);
  EXPECT_EQ(rndf.Name(), name);
}

//////////////////////////////////////////////////
/// \brief Check segments-related functions.
TEST(RNDF, segments)
{
  RNDF rndf;
  EXPECT_EQ(rndf.NumSegments(), 0u);

  int id = 1;
  Segment segment;
  // Check an inexistent segment id.
  EXPECT_FALSE(rndf.Segment(id, segment));
  // Try to remove an inexistent segment id.
  EXPECT_FALSE(rndf.RemoveSegment(id));
  // Try to add a segment with an invalid Id.
  EXPECT_FALSE(rndf.AddSegment(segment));

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
  Lane lane(id);
  EXPECT_TRUE(lane.AddWaypoint(wp));
  EXPECT_EQ(lane.NumWaypoints(), 1u);

  // Add the lane to the segment.
  segment.SetId(id);
  EXPECT_TRUE(segment.AddLane(lane));
  EXPECT_EQ(segment.NumLanes(), 1u);

  // Add the segment to the RNDF.
  EXPECT_TRUE(rndf.AddSegment(segment));
  EXPECT_EQ(rndf.NumSegments(), 1u);

  // Try to add an existent segment.
  EXPECT_FALSE(rndf.AddSegment(segment));
  EXPECT_EQ(rndf.NumSegments(), 1u);

  // Get the segment.
  Segment segment2;
  EXPECT_TRUE(rndf.Segment(segment.Id(), segment2));
  EXPECT_EQ(segment, segment2);

  // Update a segment.
  segment2.SetName("segment2");
  EXPECT_TRUE(rndf.UpdateSegment(segment2));
  Segment segment3;
  EXPECT_TRUE(rndf.Segment(segment2.Id(), segment3));
  EXPECT_EQ(segment3, segment2);

  // Get a mutable reference to all segments.
  std::vector<Segment> &segments = rndf.Segments();
  ASSERT_EQ(segments.size(), 1u);
  // Modify a segment.
  Segment &aSegment = segments.at(0);
  aSegment.SetName("updated_name");
  EXPECT_TRUE(rndf.Segment(segment2.Id(), segment3));
  EXPECT_EQ(segment3.Name(), "updated_name");

  for (auto const &s : rndf.Segments())
    EXPECT_TRUE(s.Valid());

  // Remove a segment.
  EXPECT_TRUE(rndf.RemoveSegment(segment2.Id()));
  EXPECT_EQ(rndf.NumSegments(), 0u);
}

//////////////////////////////////////////////////
/// \brief Check zones-related functions.
TEST(RNDF, zones)
{
  RNDF rndf;
  EXPECT_EQ(rndf.NumZones(), 0u);

  int id = 1;
  Zone zone;
  // Check an inexistent zone id.
  EXPECT_FALSE(rndf.Zone(id, zone));
  // Try to remove an inexistent zone id.
  EXPECT_FALSE(rndf.RemoveZone(id));
  // Try to add a zone with an invalid Id.
  EXPECT_FALSE(rndf.AddZone(zone));

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

  // Add the zone to the RNDF.
  zone.SetId(id);
  EXPECT_TRUE(rndf.AddZone(zone));
  EXPECT_EQ(rndf.NumZones(), 1u);

  // Try to add an existent zone.
  EXPECT_FALSE(rndf.AddZone(zone));
  EXPECT_EQ(rndf.NumZones(), 1u);

  // Get the zone.
  Zone zone2;
  EXPECT_TRUE(rndf.Zone(zone.Id(), zone2));
  EXPECT_EQ(zone, zone2);

  // Update a zone.
  zone2.SetName("segment2");
  EXPECT_TRUE(rndf.UpdateZone(zone2));
  Zone zone3;
  EXPECT_TRUE(rndf.Zone(zone2.Id(), zone3));
  EXPECT_EQ(zone3, zone2);

  // Get a mutable reference to all zones.
  std::vector<Zone> &zones = rndf.Zones();
  ASSERT_EQ(zones.size(), 1u);
  // Modify a zone.
  Zone &aZone = zones.at(0);
  aZone.SetName("updated_name");
  EXPECT_TRUE(rndf.Zone(zone2.Id(), zone3));
  EXPECT_EQ(zone3.Name(), "updated_name");

  for (auto const &z : rndf.Zones())
    EXPECT_TRUE(z.Valid());

  // Remove a zone.
  EXPECT_TRUE(rndf.RemoveZone(zone2.Id()));
  EXPECT_EQ(rndf.NumZones(), 0u);
}

//////////////////////////////////////////////////
/// \brief Check RNDF date.
TEST(RNDF, Date)
{
  RNDF rndf;
  EXPECT_TRUE(rndf.Date().empty());

  std::string date = "2016.11.06";
  rndf.SetDate(date);
  EXPECT_EQ(rndf.Date(), date);
}

//////////////////////////////////////////////////
/// \brief Check RNDF version.
TEST(RNDF, Version)
{
  RNDF rndf;
  EXPECT_TRUE(rndf.Version().empty());

  std::string version = "2.3.6";
  rndf.SetVersion(version);
  EXPECT_EQ(rndf.Version(), version);
}

//////////////////////////////////////////////////
/// \brief Check RNDF validation.
TEST(RNDF, Validation)
{
  RNDF rndf;
  EXPECT_FALSE(rndf.Valid());

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
  int id = 1;
  Lane lane(id);
  EXPECT_TRUE(lane.AddWaypoint(wp));
  EXPECT_EQ(lane.NumWaypoints(), 1u);

  // Add the lane to the segment.
  Segment segment;
  EXPECT_TRUE(segment.AddLane(lane));
  EXPECT_EQ(segment.NumLanes(), 1u);

  // Add the segment to the RNDF.
  segment.SetId(id);
  EXPECT_TRUE(rndf.AddSegment(segment));

  // The segment is not valid (missing Id).
  EXPECT_FALSE(rndf.Valid());
}

//////////////////////////////////////////////////
/// \brief Check loading an RNDF from an inexistent file.
TEST(RNDF, loadInexistentFiles)
{
  {
    RNDF rndf("__inexistentFile___.rndf");
    EXPECT_FALSE(rndf.Valid());
  }

  {
    RNDF rndf;
    rndf.Load("__inexistentFile___.rndf");
    EXPECT_FALSE(rndf.Valid());
  }
}

//////////////////////////////////////////////////
/// \brief Check loading real RNDF files.
TEST(RNDF, loadSamples)
{
  std::string dirPath(std::string(PROJECT_SOURCE_PATH));
  {
    RNDF rndf(dirPath + "/test/rndf/sample1.rndf");
    EXPECT_TRUE(rndf.Valid());

    rndf::UniqueId id(1, 1, 1);
    RNDFNode *nodeInfo = rndf.Info(id);
    ASSERT_TRUE(nodeInfo != nullptr);
    ASSERT_TRUE(nodeInfo->Segment() != nullptr);
    ASSERT_EQ(nodeInfo->Segment()->Id(), 1);
    ASSERT_TRUE(nodeInfo->Lane() != nullptr);
    ASSERT_EQ(nodeInfo->Lane()->Id(), 1);
    ASSERT_TRUE(nodeInfo->Zone() == nullptr);
  }
  {
    RNDF rndf(dirPath + "/test/rndf/sample2.rndf");
    EXPECT_TRUE(rndf.Valid());

    rndf::UniqueId id(68, 0, 20);
    RNDFNode *nodeInfo = rndf.Info(id);
    ASSERT_TRUE(nodeInfo != nullptr);
    ASSERT_TRUE(nodeInfo->Segment() == nullptr);
    ASSERT_TRUE(nodeInfo->Lane() == nullptr);
    ASSERT_TRUE(nodeInfo->Zone() != nullptr);
    EXPECT_EQ(nodeInfo->Zone()->Id(), 68);

    rndf::UniqueId spotId(61, 2, 2);
    RNDFNode *spotInfo = rndf.Info(spotId);
    ASSERT_TRUE(spotInfo != nullptr);
    ASSERT_TRUE(spotInfo->Segment() == nullptr);
    ASSERT_TRUE(spotInfo->Lane() == nullptr);
    ASSERT_TRUE(spotInfo->Zone() != nullptr);
    EXPECT_EQ(spotInfo->Zone()->Id(), 61);

    // This point does not exist.
    rndf::UniqueId unknownId(99, 1, 2);
    nodeInfo = rndf.Info(unknownId);
    EXPECT_EQ(nodeInfo, nullptr);
  }
}

//////////////////////////////////////////////////
/// \brief Check loading specific RNDF blocks from files.
TEST_F(RNDFTest, load)
{
  // The first element is the content to be parsed.
  // The second element is the expected return value.
  // The third element is the test Id.
  std::vector<std::tuple<std::string, bool, int>> testCases =
  {
    std::make_tuple(""                              , false, 0),
    std::make_tuple("\n\n"                          , false, 0),
    // Missing RNDF_name.
    std::make_tuple(
      "roadA /*A comment */\n"
      "\n"
                                                    , false, 0),
    // Invalid RNDF_name.
    std::make_tuple(
      "xxx roadA /*A comment */\n"
      "\n"
                                                    , false, 0),
    // Missing RNDF_name value.
    std::make_tuple(
      "\n\n"
      "RNDF_name /* A comment */\n"
      "num_segments 2\n"
                                                    , false, 0),
    // Missing num_segments.
    std::make_tuple(
      "\n\n"
      "RNDF_name roadA /* A comment */\n"
                                                    , false, 0),
    // Missing num_segments.
    std::make_tuple(
      "\n\n"
      "RNDF_name roadA /* A comment */\n"
      "1\n"
      "num_zones 1\n"
                                                    , false, 0),
    // Missing num_segments value.
    std::make_tuple(
      "\n\n"
      "RNDF_name roadA /* A comment */\n"
      "num_segments \n"
      "num_zones 1\n"
                                                    , false, 0),
    // Invalid num_segments value.
    std::make_tuple(
      "\n\n"
      "RNDF_name roadA /* A comment */\n"
      "num_segments xxx\n"
      "num_zones 1\n"
                                                    , false, 0),
    // Invalid num_segments value.
    std::make_tuple(
      "\n\n"
      "RNDF_name roadA /* A comment */\n"
      "num_segments -1\n"
      "num_zones 1\n"
                                                    , false, 0),
    // Invalid num_segments value.
    std::make_tuple(
      "\n\n"
      "RNDF_name roadA /* A comment */\n"
      "num_segments 0\n"
      "num_zones 1\n"
                                                    , false, 0),
    // Missing num_zones.
    std::make_tuple(
      "\n\n"
      "RNDF_name roadA /* A comment */\n"
      "num_segments 2\n"
      "format_version 1.2.2\n"
                                                    , false, 0),
    // Missing num_zones.
    std::make_tuple(
      "\n\n"
      "RNDF_name roadA /* A comment */\n"
      "num_segments 2\n"
      "1\n"
      "format_version 1.2.2\n"
                                                    , false, 0),
    // Missing num_zones value.
    std::make_tuple(
      "\n\n"
      "RNDF_name roadA /* A comment */\n"
      "num_segments 2\n"
      "num_zones \n"
      "format_version 1.2.2\n"
                                                    , false, 0),
    // Invalid num_zones value.
    std::make_tuple(
      "\n\n"
      "RNDF_name roadA /* A comment */\n"
      "num_segments 2\n"
      "num_zones xxx\n"
      "format_version 1.2.2\n"
                                                    , false, 0),
    // Invalid num_zones value.
    std::make_tuple(
      "\n\n"
      "RNDF_name roadA /* A comment */\n"
      "num_segments 2\n"
      "num_zones -1\n"
      "format_version 1.2.2\n"
                                                    , false, 0),
    // Missing format_version.
    std::make_tuple(
      "\n\n"
      "RNDF_name roadA /* A comment */\n"
      "num_segments 2\n"
      "num_zones 1\n"
      "1.2.2\n"
      "segment 1\n"
                                                    , false, 0),
    // Missing format_version value.
    std::make_tuple(
      "\n\n"
      "RNDF_name roadA /* A comment */\n"
      "num_segments 2\n"
      "num_zones 1\n"
      "format_version /* comment */\n"
      "segment 1\n"
                                                    , false, 0),
    // Repeated format_version value.
    std::make_tuple(
      "\n\n"
      "RNDF_name roadA /* A comment */\n"
      "num_segments 2\n"
      "num_zones 1\n"
      "format_version 1.2.2/* comment */\n"
      "format_version 1.2.2/* comment */\n"
      "segment 1\n"
                                                    , false, 0),
    // Missing creation_date.
    std::make_tuple(
      "\n\n"
      "RNDF_name roadA /* A comment */\n"
      "num_segments 2\n"
      "num_zones 1\n"
      "3-Nov-07\n"
      "segment 1\n"
                                                    , false, 0),
    // Missing format_version value.
    std::make_tuple(
      "\n\n"
      "RNDF_name roadA /* A comment */\n"
      "num_segments 2\n"
      "num_zones 1\n"
      "creation_date /* comment */\n"
      "segment 1\n"
                                                    , false, 0),
    // Repeated creation_date value.
    std::make_tuple(
      "\n\n"
      "RNDF_name roadA /* A comment */\n"
      "num_segments 2\n"
      "num_zones 1\n"
      "creation_date 3-Nov-07\n"
      "creation_date 3-Nov-07\n"
      "segment 1\n"
                                                    , false, 0),
    // Missing one segment.
    std::make_tuple(
      "RNDF_name roadA /*A comment */\n"
      "num_segments 2\n"
      "num_zones 0\n"
      "format_version 1.2.2\n"
      "segment 1\n"
      "num_lanes 1\n"
      "lane 1.1\n"
      "num_waypoints 9\n"
      "1.1.1 30.3870130 -97.7276181\n"
      "1.1.2 30.3876366 -97.7273710\n"
      "1.1.3 30.3881655 -97.7271432\n"
      "1.1.4 30.3885908 -97.7269603\n"
      "1.1.5 30.3888965 -97.7268688\n"
      "1.1.6 30.3891127 -97.7268779\n"
      "1.1.7 30.3893659 -97.7268964\n"
      "1.1.8 30.3896061 -97.7268374\n"
      "1.1.9 30.3900594 -97.7266452\n"
      "end_lane\n"
      "end_segment\n"
      "end_file\n"
                                                    , false, 0),
    // Extra segment found.
    std::make_tuple(
      "RNDF_name roadA /*A comment */\n"
      "num_segments 1\n"
      "num_zones 0\n"
      "format_version 1.2.2\n"
      "segment 1\n"
      "num_lanes 1\n"
      "lane 1.1\n"
      "num_waypoints 9\n"
      "1.1.1 30.3870130 -97.7276181\n"
      "1.1.2 30.3876366 -97.7273710\n"
      "1.1.3 30.3881655 -97.7271432\n"
      "1.1.4 30.3885908 -97.7269603\n"
      "1.1.5 30.3888965 -97.7268688\n"
      "1.1.6 30.3891127 -97.7268779\n"
      "1.1.7 30.3893659 -97.7268964\n"
      "1.1.8 30.3896061 -97.7268374\n"
      "1.1.9 30.3900594 -97.7266452\n"
      "end_lane\n"
      "end_segment\n"
      "segment 2\n"
      "num_lanes 1\n"
      "lane 2.1\n"
      "num_waypoints 8\n"
      "2.1.1 30.3902771 -97.7266013\n"
      "2.1.2 30.3897954 -97.7268091\n"
      "2.1.3 30.3894485 -97.7269254\n"
      "2.1.4 30.3891929 -97.7269262\n"
      "2.1.5 30.3889129 -97.7269210\n"
      "2.1.6 30.3886607 -97.7269929\n"
      "2.1.7 30.3883870 -97.7271055\n"
      "2.1.8 30.3880462 -97.7272477\n"
      "end_lane\n"
      "end_segment\n"
      "end_file\n"
                                                    , false, 0),
    // Two non-consecutive segments.
    std::make_tuple(
      "RNDF_name roadA /*A comment */\n"
      "num_segments 2\n"
      "num_zones 0\n"
      "format_version 1.2.2\n"
      "segment 1\n"
      "num_lanes 1\n"
      "lane 1.1\n"
      "num_waypoints 9\n"
      "1.1.1 30.3870130 -97.7276181\n"
      "1.1.2 30.3876366 -97.7273710\n"
      "1.1.3 30.3881655 -97.7271432\n"
      "1.1.4 30.3885908 -97.7269603\n"
      "1.1.5 30.3888965 -97.7268688\n"
      "1.1.6 30.3891127 -97.7268779\n"
      "1.1.7 30.3893659 -97.7268964\n"
      "1.1.8 30.3896061 -97.7268374\n"
      "1.1.9 30.3900594 -97.7266452\n"
      "end_lane\n"
      "end_segment\n"
      "segment 3\n"
      "num_lanes 1\n"
      "lane 3.1\n"
      "num_waypoints 8\n"
      "3.1.1 30.3902771 -97.7266013\n"
      "3.1.2 30.3897954 -97.7268091\n"
      "3.1.3 30.3894485 -97.7269254\n"
      "3.1.4 30.3891929 -97.7269262\n"
      "3.1.5 30.3889129 -97.7269210\n"
      "3.1.6 30.3886607 -97.7269929\n"
      "3.1.7 30.3883870 -97.7271055\n"
      "3.1.8 30.3880462 -97.7272477\n"
      "end_lane\n"
      "end_segment\n"
      "end_file\n"
                                                    , false, 0),
    // Missing one zone.
    std::make_tuple(
      "RNDF_name roadA /*A comment */\n"
      "\n"
      " /* Ignore */\n"
      "\n"
      "num_segments 2\n"
      "num_zones 1\n"
      "format_version 1.2.2\n"
      "segment 1\n"
      "num_lanes 1\n"
      "lane 1.1\n"
      "num_waypoints 9\n"
      "1.1.1 30.3870130 -97.7276181\n"
      "1.1.2 30.3876366 -97.7273710\n"
      "1.1.3 30.3881655 -97.7271432\n"
      "1.1.4 30.3885908 -97.7269603\n"
      "1.1.5 30.3888965 -97.7268688\n"
      "1.1.6 30.3891127 -97.7268779\n"
      "1.1.7 30.3893659 -97.7268964\n"
      "1.1.8 30.3896061 -97.7268374\n"
      "1.1.9 30.3900594 -97.7266452\n"
      "end_lane\n"
      "end_segment\n"
      "segment 2\n"
      "num_lanes 1\n"
      "lane 2.1\n"
      "num_waypoints 8\n"
      "2.1.1 30.3902771 -97.7266013\n"
      "2.1.2 30.3897954 -97.7268091\n"
      "2.1.3 30.3894485 -97.7269254\n"
      "2.1.4 30.3891929 -97.7269262\n"
      "2.1.5 30.3889129 -97.7269210\n"
      "2.1.6 30.3886607 -97.7269929\n"
      "2.1.7 30.3883870 -97.7271055\n"
      "2.1.8 30.3880462 -97.7272477\n"
      "end_lane\n"
      "end_segment\n"
      "end_file\n"
                                                    , false, 0),
    // Extra zone found.
    std::make_tuple(
      "RNDF_name roadA /*A comment */\n"
      "\n"
      " /* Ignore */\n"
      "\n"
      "num_segments 2\n"
      "num_zones 0\n"
      "format_version 1.2.2\n"
      "segment 1\n"
      "num_lanes 1\n"
      "lane 1.1\n"
      "num_waypoints 9\n"
      "1.1.1 30.3870130 -97.7276181\n"
      "1.1.2 30.3876366 -97.7273710\n"
      "1.1.3 30.3881655 -97.7271432\n"
      "1.1.4 30.3885908 -97.7269603\n"
      "1.1.5 30.3888965 -97.7268688\n"
      "1.1.6 30.3891127 -97.7268779\n"
      "1.1.7 30.3893659 -97.7268964\n"
      "1.1.8 30.3896061 -97.7268374\n"
      "1.1.9 30.3900594 -97.7266452\n"
      "end_lane\n"
      "end_segment\n"
      "segment 2\n"
      "num_lanes 1\n"
      "lane 2.1\n"
      "num_waypoints 8\n"
      "2.1.1 30.3902771 -97.7266013\n"
      "2.1.2 30.3897954 -97.7268091\n"
      "2.1.3 30.3894485 -97.7269254\n"
      "2.1.4 30.3891929 -97.7269262\n"
      "2.1.5 30.3889129 -97.7269210\n"
      "2.1.6 30.3886607 -97.7269929\n"
      "2.1.7 30.3883870 -97.7271055\n"
      "2.1.8 30.3880462 -97.7272477\n"
      "end_lane\n"
      "end_segment\n"
      "zone  3  /*zones & segments have unique identifiers.*/\n"
      "num_spots 1\n"
      "zone_name Central_Parking_Lot\n"
      "perimeter 3.0\n"
      "num_perimeterpoints 3\n"
      "exit  3.0.1  2.1.1  /*out of zone 3*/\n"
      "3.0.1  38.872271 -77.203339\n"
      "3.0.2  38.872258 -77.202804\n"
      "3.0.3  38.872264 -77.202315\n"
      "end_perimeter\n"
      "spot  3.1\n"
      "spot_width  16\n"
      "checkpoint  3.1.2  1\n"
      "3.1.1  38.872151 -77.202972\n"
      "3.1.2  38.872103 -77.202971\n"
      "end_spot\n"
      "end_zone\n"
      "end_file\n"
                                                    , false, 0),
    // The first zone Id should be equal to the last segment Id + 1.
    std::make_tuple(
      "RNDF_name roadA /*A comment */\n"
      "\n"
      " /* Ignore */\n"
      "\n"
      "num_segments 2\n"
      "num_zones 1\n"
      "format_version 1.2.2\n"
      "segment 1\n"
      "num_lanes 1\n"
      "lane 1.1\n"
      "num_waypoints 9\n"
      "1.1.1 30.3870130 -97.7276181\n"
      "1.1.2 30.3876366 -97.7273710\n"
      "1.1.3 30.3881655 -97.7271432\n"
      "1.1.4 30.3885908 -97.7269603\n"
      "1.1.5 30.3888965 -97.7268688\n"
      "1.1.6 30.3891127 -97.7268779\n"
      "1.1.7 30.3893659 -97.7268964\n"
      "1.1.8 30.3896061 -97.7268374\n"
      "1.1.9 30.3900594 -97.7266452\n"
      "end_lane\n"
      "end_segment\n"
      "segment 2\n"
      "num_lanes 1\n"
      "lane 2.1\n"
      "num_waypoints 8\n"
      "2.1.1 30.3902771 -97.7266013\n"
      "2.1.2 30.3897954 -97.7268091\n"
      "2.1.3 30.3894485 -97.7269254\n"
      "2.1.4 30.3891929 -97.7269262\n"
      "2.1.5 30.3889129 -97.7269210\n"
      "2.1.6 30.3886607 -97.7269929\n"
      "2.1.7 30.3883870 -97.7271055\n"
      "2.1.8 30.3880462 -97.7272477\n"
      "end_lane\n"
      "end_segment\n"
      "zone  4  /*zones & segments have unique identifiers.*/\n"
      "num_spots 1\n"
      "zone_name Central_Parking_Lot\n"
      "perimeter 4.0\n"
      "num_perimeterpoints 3\n"
      "exit  4.0.1  2.1.1  /*out of zone 3*/\n"
      "4.0.1  38.872271 -77.203339\n"
      "4.0.2  38.872258 -77.202804\n"
      "4.0.3  38.872264 -77.202315\n"
      "end_perimeter\n"
      "spot  4.1\n"
      "spot_width  16\n"
      "checkpoint  4.1.2  1\n"
      "4.1.1  38.872151 -77.202972\n"
      "4.1.2  38.872103 -77.202971\n"
      "end_spot\n"
      "end_zone\n"
      "end_file\n"
                                                    , false, 0),
    // Non-consecutive zones.
    std::make_tuple(
      "RNDF_name roadA /*A comment */\n"
      "\n"
      " /* Ignore */\n"
      "\n"
      "num_segments 2\n"
      "num_zones 2\n"
      "format_version 1.2.2\n"
      "segment 1\n"
      "num_lanes 1\n"
      "lane 1.1\n"
      "num_waypoints 9\n"
      "1.1.1 30.3870130 -97.7276181\n"
      "1.1.2 30.3876366 -97.7273710\n"
      "1.1.3 30.3881655 -97.7271432\n"
      "1.1.4 30.3885908 -97.7269603\n"
      "1.1.5 30.3888965 -97.7268688\n"
      "1.1.6 30.3891127 -97.7268779\n"
      "1.1.7 30.3893659 -97.7268964\n"
      "1.1.8 30.3896061 -97.7268374\n"
      "1.1.9 30.3900594 -97.7266452\n"
      "end_lane\n"
      "end_segment\n"
      "segment 2\n"
      "num_lanes 1\n"
      "lane 2.1\n"
      "num_waypoints 8\n"
      "2.1.1 30.3902771 -97.7266013\n"
      "2.1.2 30.3897954 -97.7268091\n"
      "2.1.3 30.3894485 -97.7269254\n"
      "2.1.4 30.3891929 -97.7269262\n"
      "2.1.5 30.3889129 -97.7269210\n"
      "2.1.6 30.3886607 -97.7269929\n"
      "2.1.7 30.3883870 -97.7271055\n"
      "2.1.8 30.3880462 -97.7272477\n"
      "end_lane\n"
      "end_segment\n"
      "zone  3  /*zones & segments have unique identifiers.*/\n"
      "num_spots 1\n"
      "zone_name Central_Parking_Lot\n"
      "perimeter 3.0\n"
      "num_perimeterpoints 3\n"
      "exit  3.0.1  2.1.1  /*out of zone 3*/\n"
      "3.0.1  38.872271 -77.203339\n"
      "3.0.2  38.872258 -77.202804\n"
      "3.0.3  38.872264 -77.202315\n"
      "end_perimeter\n"
      "spot  3.1\n"
      "spot_width  16\n"
      "checkpoint  3.1.2  1\n"
      "3.1.1  38.872151 -77.202972\n"
      "3.1.2  38.872103 -77.202971\n"
      "end_spot\n"
      "end_zone\n"
      "zone  5  /*zones & segments have unique identifiers.*/\n"
      "num_spots 1\n"
      "zone_name Central_Parking_Lot\n"
      "perimeter 5.0\n"
      "num_perimeterpoints 3\n"
      "exit  5.0.1  2.1.1  /*out of zone 3*/\n"
      "5.0.1  38.872271 -77.203339\n"
      "5.0.2  38.872258 -77.202804\n"
      "5.0.3  38.872264 -77.202315\n"
      "end_perimeter\n"
      "spot  5.1\n"
      "spot_width  16\n"
      "checkpoint  5.1.2  1\n"
      "5.1.1  38.872151 -77.202972\n"
      "5.1.2  38.872103 -77.202971\n"
      "end_spot\n"
      "end_zone\n"
      "end_file\n"
                                                    , false, 0),
    // Missing terminator.
    std::make_tuple(
      "RNDF_name roadA /*A comment */\n"
      "num_segments 2\n"
      "num_zones 0\n"
      "format_version 1.2.2\n"
      "segment 1\n"
      "num_lanes 1\n"
      "lane 1.1\n"
      "num_waypoints 9\n"
      "1.1.1 30.3870130 -97.7276181\n"
      "1.1.2 30.3876366 -97.7273710\n"
      "1.1.3 30.3881655 -97.7271432\n"
      "1.1.4 30.3885908 -97.7269603\n"
      "1.1.5 30.3888965 -97.7268688\n"
      "1.1.6 30.3891127 -97.7268779\n"
      "1.1.7 30.3893659 -97.7268964\n"
      "1.1.8 30.3896061 -97.7268374\n"
      "1.1.9 30.3900594 -97.7266452\n"
      "end_lane\n"
      "end_segment\n"
      "segment 2\n"
      "num_lanes 1\n"
      "lane 2.1\n"
      "num_waypoints 8\n"
      "2.1.1 30.3902771 -97.7266013\n"
      "2.1.2 30.3897954 -97.7268091\n"
      "2.1.3 30.3894485 -97.7269254\n"
      "2.1.4 30.3891929 -97.7269262\n"
      "2.1.5 30.3889129 -97.7269210\n"
      "2.1.6 30.3886607 -97.7269929\n"
      "2.1.7 30.3883870 -97.7271055\n"
      "2.1.8 30.3880462 -97.7272477\n"
      "end_lane\n"
      "end_segment\n"
      "\n"
                                                    , false, 0),
    // No options.
    std::make_tuple(
      "RNDF_name roadA /*A comment */\n"
      "\n"
      " /* Ignore */\n"
      "\n"
      "num_segments 2\n"
      "num_zones 1\n"
      "segment 1\n"
      "num_lanes 1\n"
      "lane 1.1\n"
      "num_waypoints 9\n"
      "1.1.1 30.3870130 -97.7276181\n"
      "1.1.2 30.3876366 -97.7273710\n"
      "1.1.3 30.3881655 -97.7271432\n"
      "1.1.4 30.3885908 -97.7269603\n"
      "1.1.5 30.3888965 -97.7268688\n"
      "1.1.6 30.3891127 -97.7268779\n"
      "1.1.7 30.3893659 -97.7268964\n"
      "1.1.8 30.3896061 -97.7268374\n"
      "1.1.9 30.3900594 -97.7266452\n"
      "end_lane\n"
      "end_segment\n"
      "segment 2\n"
      "num_lanes 1\n"
      "lane 2.1\n"
      "num_waypoints 8\n"
      "2.1.1 30.3902771 -97.7266013\n"
      "2.1.2 30.3897954 -97.7268091\n"
      "2.1.3 30.3894485 -97.7269254\n"
      "2.1.4 30.3891929 -97.7269262\n"
      "2.1.5 30.3889129 -97.7269210\n"
      "2.1.6 30.3886607 -97.7269929\n"
      "2.1.7 30.3883870 -97.7271055\n"
      "2.1.8 30.3880462 -97.7272477\n"
      "end_lane\n"
      "end_segment\n"
      "zone  3  /*zones & segments have unique identifiers.*/\n"
      "num_spots 1\n"
      "zone_name Central_Parking_Lot\n"
      "perimeter 3.0\n"
      "num_perimeterpoints 3\n"
      "exit  3.0.1  2.1.1  /*out of zone 3*/\n"
      "3.0.1  38.872271 -77.203339\n"
      "3.0.2  38.872258 -77.202804\n"
      "3.0.3  38.872264 -77.202315\n"
      "end_perimeter\n"
      "spot  3.1\n"
      "spot_width  16\n"
      "checkpoint  3.1.2  1\n"
      "3.1.1  38.872151 -77.202972\n"
      "3.1.2  38.872103 -77.202971\n"
      "end_spot\n"
      "end_zone\n"
      "end_file\n"
                                                    , true, 1),
    // One option (format_version).
    std::make_tuple(
      "RNDF_name roadA /*A comment */\n"
      "\n"
      " /* Ignore */\n"
      "\n"
      "num_segments 2\n"
      "num_zones 1\n"
      "format_version 1.2.2\n"
      "segment 1\n"
      "num_lanes 1\n"
      "lane 1.1\n"
      "num_waypoints 9\n"
      "1.1.1 30.3870130 -97.7276181\n"
      "1.1.2 30.3876366 -97.7273710\n"
      "1.1.3 30.3881655 -97.7271432\n"
      "1.1.4 30.3885908 -97.7269603\n"
      "1.1.5 30.3888965 -97.7268688\n"
      "1.1.6 30.3891127 -97.7268779\n"
      "1.1.7 30.3893659 -97.7268964\n"
      "1.1.8 30.3896061 -97.7268374\n"
      "1.1.9 30.3900594 -97.7266452\n"
      "end_lane\n"
      "end_segment\n"
      "segment 2\n"
      "num_lanes 1\n"
      "lane 2.1\n"
      "num_waypoints 8\n"
      "2.1.1 30.3902771 -97.7266013\n"
      "2.1.2 30.3897954 -97.7268091\n"
      "2.1.3 30.3894485 -97.7269254\n"
      "2.1.4 30.3891929 -97.7269262\n"
      "2.1.5 30.3889129 -97.7269210\n"
      "2.1.6 30.3886607 -97.7269929\n"
      "2.1.7 30.3883870 -97.7271055\n"
      "2.1.8 30.3880462 -97.7272477\n"
      "end_lane\n"
      "end_segment\n"
      "zone  3  /*zones & segments have unique identifiers.*/\n"
      "num_spots 1\n"
      "zone_name Central_Parking_Lot\n"
      "perimeter 3.0\n"
      "num_perimeterpoints 3\n"
      "exit  3.0.1  2.1.1  /*out of zone 3*/\n"
      "3.0.1  38.872271 -77.203339\n"
      "3.0.2  38.872258 -77.202804\n"
      "3.0.3  38.872264 -77.202315\n"
      "end_perimeter\n"
      "spot  3.1\n"
      "spot_width  16\n"
      "checkpoint  3.1.2  1\n"
      "3.1.1  38.872151 -77.202972\n"
      "3.1.2  38.872103 -77.202971\n"
      "end_spot\n"
      "end_zone\n"
      "end_file\n"
                                                    , true, 2),
    // Two options (format_version and creation_date).
    std::make_tuple(
      "RNDF_name roadA /*A comment */\n"
      "\n"
      " /* Ignore */\n"
      "\n"
      "num_segments 2\n"
      "num_zones 1\n"
      "creation_date 29-Mar-07\n"
      "format_version 1.2.2\n"
      "segment 1\n"
      "num_lanes 1\n"
      "lane 1.1\n"
      "num_waypoints 9\n"
      "1.1.1 30.3870130 -97.7276181\n"
      "1.1.2 30.3876366 -97.7273710\n"
      "1.1.3 30.3881655 -97.7271432\n"
      "1.1.4 30.3885908 -97.7269603\n"
      "1.1.5 30.3888965 -97.7268688\n"
      "1.1.6 30.3891127 -97.7268779\n"
      "1.1.7 30.3893659 -97.7268964\n"
      "1.1.8 30.3896061 -97.7268374\n"
      "1.1.9 30.3900594 -97.7266452\n"
      "end_lane\n"
      "end_segment\n"
      "segment 2\n"
      "num_lanes 1\n"
      "lane 2.1\n"
      "num_waypoints 8\n"
      "2.1.1 30.3902771 -97.7266013\n"
      "2.1.2 30.3897954 -97.7268091\n"
      "2.1.3 30.3894485 -97.7269254\n"
      "2.1.4 30.3891929 -97.7269262\n"
      "2.1.5 30.3889129 -97.7269210\n"
      "2.1.6 30.3886607 -97.7269929\n"
      "2.1.7 30.3883870 -97.7271055\n"
      "2.1.8 30.3880462 -97.7272477\n"
      "end_lane\n"
      "end_segment\n"
      "zone  3  /*zones & segments have unique identifiers.*/\n"
      "num_spots 1\n"
      "zone_name Central_Parking_Lot\n"
      "perimeter 3.0\n"
      "num_perimeterpoints 3\n"
      "exit  3.0.1  2.1.1  /*out of zone 3*/\n"
      "3.0.1  38.872271 -77.203339\n"
      "3.0.2  38.872258 -77.202804\n"
      "3.0.3  38.872264 -77.202315\n"
      "end_perimeter\n"
      "spot  3.1\n"
      "spot_width  16\n"
      "checkpoint  3.1.2  1\n"
      "3.1.1  38.872151 -77.202972\n"
      "3.1.2  38.872103 -77.202971\n"
      "end_spot\n"
      "end_zone\n"
      "end_file\n"
                                                    , true, 3),
  };

  for (auto const &testCase : testCases)
  {
    std::string content = std::get<0>(testCase);
    int testId = std::get<2>(testCase);

    // Expectations.
    bool expectedResult = std::get<1>(testCase);

    // Write the content of this test case into the test file.
    this->PopulateFile(content);

    // Leave this comment for knowing wich test case failed if needed.
    std::cout << "Testing [" << content << "]" << std::endl;

    // Check expectations.
    RNDF rndf;
    bool res;
    EXPECT_EQ(res = rndf.Load(this->fileName), expectedResult);
    EXPECT_EQ(rndf.Valid(), res);
    if (res)
    {
      switch (testId)
      {
        case 1:
        {
          EXPECT_EQ(rndf.Name(), "roadA");
          EXPECT_EQ(rndf.NumSegments(), 2u);
          EXPECT_EQ(rndf.NumZones(), 1u);
          EXPECT_TRUE(rndf.Version().empty());
          EXPECT_TRUE(rndf.Date().empty());
          break;
        }
        case 2:
        {
          EXPECT_EQ(rndf.Name(), "roadA");
          EXPECT_EQ(rndf.NumSegments(), 2u);
          EXPECT_EQ(rndf.NumZones(), 1u);
          EXPECT_EQ(rndf.Version(), "1.2.2");
          EXPECT_TRUE(rndf.Date().empty());
          break;
        }
        case 3:
        {
          EXPECT_EQ(rndf.Name(), "roadA");
          EXPECT_EQ(rndf.NumSegments(), 2u);
          EXPECT_EQ(rndf.NumZones(), 1u);
          EXPECT_EQ(rndf.Version(), "1.2.2");
          EXPECT_EQ(rndf.Date(), "29-Mar-07");
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
