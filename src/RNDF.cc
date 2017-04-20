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

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "ignition/rndf/Lane.hh"
#include "ignition/rndf/ParkingSpot.hh"
#include "ignition/rndf/ParserUtils.hh"
#include "ignition/rndf/Perimeter.hh"
#include "ignition/rndf/RNDF.hh"
#include "ignition/rndf/RNDFNode.hh"
#include "ignition/rndf/Segment.hh"
#include "ignition/rndf/UniqueId.hh"
#include "ignition/rndf/Waypoint.hh"
#include "ignition/rndf/Zone.hh"

using namespace ignition;
using namespace rndf;

namespace ignition
{
  namespace rndf
  {
    /// \internal
    /// \brief Private data for RNDFHeader class.
    class RNDFHeaderPrivate
    {
      /// \brief Default constructor.
      public: RNDFHeaderPrivate() = default;

      /// \brief Destructor.
      public: virtual ~RNDFHeaderPrivate() = default;

      /// \brief Format version.
      public: std::string version = "";

      /// \brief Creation date.
      public: std::string date = "";
    };

    /// \internal
    /// \brief Private data for RNDF class.
    class RNDFPrivate
    {
      /// \brief Constructor.
      public: RNDFPrivate() = default;

      /// \brief Destructor.
      public: virtual ~RNDFPrivate() = default;

      /// \brief RNDF name.
      public: std::string name = "";

      /// \brief The collection of segments.
      public: std::vector<rndf::Segment> segments;

      /// \brief The collection of zones.
      public: std::vector<rndf::Zone> zones;

      /// \brief Optional segment header members.
      public: RNDFHeader header;

      /// \brief A map used as a cache where the keys are the string
      /// representation of unique Ids (e.g. 1.2.1) and the values are the
      /// associated RNDFNode objects containing the metadata associated to
      /// the unique Id..
      public: std::map<std::string, rndf::RNDFNode> cache;
    };
  }
}

//////////////////////////////////////////////////
RNDFHeader::RNDFHeader()
{
  this->dataPtr.reset(new RNDFHeaderPrivate());
}

//////////////////////////////////////////////////
RNDFHeader::~RNDFHeader()
{
}

//////////////////////////////////////////////////
bool RNDFHeader::Load(std::ifstream &_rndfFile, int &_lineNumber)
{
  bool versionFound = false;
  bool dateFound = false;

  for (auto i = 0; i < 2; ++i)
  {
    auto oldPos = _rndfFile.tellg();
    int oldLineNumber = _lineNumber;

    std::string lineread;
    nextRealLine(_rndfFile, lineread, _lineNumber);

    auto tokens = split(lineread, " ");

    // Check if we found the "segment" element.
    // If this is the case we should leave.
    if (tokens.size() == 2 && tokens.at(0) == "segment")
    {
      // Restore the file position and line number.
      // ParseHeader() shouldn't have any effect.
      _rndfFile.seekg(oldPos);
      _lineNumber = oldLineNumber;
      return true;
    }

    if ((tokens.size() != 2)                            ||
        (tokens[0] == "format_version" && versionFound) ||
        (tokens[0] == "creation_date" && dateFound))
    {
      // Invalid or repeated header element.
      std::cerr << "[Line " << _lineNumber << "]: Unable to parse file header "
                << "element." << std::endl;
      std::cerr << " \"" << lineread << "\"" << std::endl;
      return false;
    }

    assert(tokens.size() == 2);

    if (tokens[0] == "format_version")
    {
      this->SetVersion(tokens[1]);
      versionFound = true;
    }
    // creation_date option.
    else
    {
      this->SetDate(tokens[1]);
      dateFound = true;
    }
  }

  return true;
}

//////////////////////////////////////////////////
std::string RNDFHeader::Version() const
{
  return this->dataPtr->version;
}

//////////////////////////////////////////////////
void RNDFHeader::SetVersion(const std::string &_version) const
{
  this->dataPtr->version = _version;
}

//////////////////////////////////////////////////
std::string RNDFHeader::Date() const
{
  return this->dataPtr->date;
}

//////////////////////////////////////////////////
void RNDFHeader::SetDate(const std::string &_newDate) const
{
  this->dataPtr->date = _newDate;
}

//////////////////////////////////////////////////
RNDF::RNDF()
  : dataPtr(new RNDFPrivate())
{
}

//////////////////////////////////////////////////
RNDF::RNDF(const std::string &_filepath)
  : RNDF()
{
  this->Load(_filepath);
}

//////////////////////////////////////////////////
RNDF::~RNDF()
{
}

//////////////////////////////////////////////////
bool RNDF::Load(const std::string &_filePath)
{
  std::ifstream rndfFile(_filePath, std::ifstream::binary);
  if (!rndfFile.good())
  {
    std::cerr << "Error opening RNDF [" << _filePath << "]" << std::endl;
    return false;
  }

  int lineNumber = -1;

  // Parse "RNDF_name"
  std::string fileName;
  if (!parseString(rndfFile, "RNDF_name", fileName, lineNumber))
    return false;

  // Parse "num_segments".
  int numSegments;
  if (!parsePositive(rndfFile, "num_segments", numSegments, lineNumber))
    return false;

  // Parse "num_zones".
  int numZones;
  if (!parseNonNegative(rndfFile, "num_zones", numZones, lineNumber))
    return false;

  // Parse optional file header (format_version and/or creation_date).
  RNDFHeader header;
  if (!header.Load(rndfFile, lineNumber))
    return false;

  // Parse all segments.
  std::vector<rndf::Segment> segments;
  for (auto i = 0; i < numSegments; ++i)
  {
    rndf::Segment segment;
    if (!segment.Load(rndfFile, lineNumber))
      return false;

    // Check that all segments are consecutive.
    if (segment.Id() != i + 1)
    {
      std::cerr << "[Line " << lineNumber << "]: Found non-consecutive segment "
                << "Id [" << segment.Id() << "]" << std::endl;
      return false;
    }

    segments.push_back(segment);
  }

  // Parse all zones.
  std::vector<rndf::Zone> zones;
  for (auto i = 0; i < numZones; ++i)
  {
    rndf::Zone zone;
    if (!zone.Load(rndfFile, lineNumber))
      return false;

    // Check that all zones are consecutive.
    size_t expectedZoneId = segments.size() + i + 1;
    if (static_cast<size_t>(zone.Id()) != expectedZoneId)
    {
      std::cerr << "[Line " << lineNumber << "]: Found non-consecutive zone "
                << "Id [" << zone.Id() << "]" << std::endl;
      return false;
    }

    zones.push_back(zone);
  }

  // Parse "end_file".
  if (!parseDelimiter(rndfFile, "end_file", lineNumber))
    return false;

  rndfFile.close();

  // Populate the RNDF.
  this->SetName(fileName);
  this->Segments() = segments;
  this->Zones() = zones;
  this->SetVersion(header.Version());
  this->SetDate(header.Date());

  this->UpdateCache();

  return true;
}

//////////////////////////////////////////////////
std::string RNDF::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
void RNDF::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

//////////////////////////////////////////////////
size_t RNDF::NumSegments() const
{
  return this->dataPtr->segments.size();
}

//////////////////////////////////////////////////
std::vector<Segment> &RNDF::Segments()
{
  return this->dataPtr->segments;
}

//////////////////////////////////////////////////
const std::vector<Segment> &RNDF::Segments() const
{
  return this->dataPtr->segments;
}

//////////////////////////////////////////////////
bool RNDF::Segment(const int _segmentId, rndf::Segment &_segment) const
{
  auto it = std::find_if(this->dataPtr->segments.begin(),
    this->dataPtr->segments.end(),
    [_segmentId](const rndf::Segment &_aSegment)
    {
      return _aSegment.Id() == _segmentId;
    });

  bool found = it != this->dataPtr->segments.end();
  if (found)
    _segment = *it;

  return found;
}

//////////////////////////////////////////////////
bool RNDF::UpdateSegment(const rndf::Segment &_segment)
{
  auto it = std::find(this->dataPtr->segments.begin(),
    this->dataPtr->segments.end(), _segment);

  bool found = it != this->dataPtr->segments.end();
  if (found)
    *it = _segment;

  return found;
}

//////////////////////////////////////////////////
bool RNDF::AddSegment(const rndf::Segment &_newSegment)
{
  // Validate the segment.
  if (!_newSegment.Valid())
  {
    std::cerr << "RNDF::AddSegment() Invalid segment with Id ["
              << _newSegment.Id() << "]" << std::endl;
    return false;
  }

  // Check whether the segment already exists.
  if (std::find(this->dataPtr->segments.begin(), this->dataPtr->segments.end(),
    _newSegment) != this->dataPtr->segments.end())
  {
    std::cerr << "RNDF::AddSegment() error: Existing segment" << std::endl;
    return false;
  }

  this->dataPtr->segments.push_back(_newSegment);
  assert(this->NumSegments() == this->dataPtr->segments.size());
  return true;
}

//////////////////////////////////////////////////
bool RNDF::RemoveSegment(const int _segmentId)
{
  rndf::Segment segment(_segmentId);
  return (this->dataPtr->segments.erase(std::remove(
    this->dataPtr->segments.begin(), this->dataPtr->segments.end(), segment),
      this->dataPtr->segments.end()) != this->dataPtr->segments.end());
}

//////////////////////////////////////////////////
size_t RNDF::NumZones() const
{
  return this->dataPtr->zones.size();
}

//////////////////////////////////////////////////
std::vector<Zone> &RNDF::Zones()
{
  return this->dataPtr->zones;
}

//////////////////////////////////////////////////
const std::vector<Zone> &RNDF::Zones() const
{
  return this->dataPtr->zones;
}

//////////////////////////////////////////////////
bool RNDF::Zone(const int _zoneId, rndf::Zone &_zone) const
{
  auto it = std::find_if(this->dataPtr->zones.begin(),
    this->dataPtr->zones.end(),
    [_zoneId](const rndf::Zone &_aZone)
    {
      return _aZone.Id() == _zoneId;
    });

  bool found = it != this->dataPtr->zones.end();
  if (found)
    _zone = *it;

  return found;
}

//////////////////////////////////////////////////
bool RNDF::UpdateZone(const rndf::Zone &_zone)
{
  auto it = std::find(this->dataPtr->zones.begin(),
    this->dataPtr->zones.end(), _zone);

  bool found = it != this->dataPtr->zones.end();
  if (found)
    *it = _zone;

  return found;
}

//////////////////////////////////////////////////
bool RNDF::AddZone(const rndf::Zone &_newZone)
{
  // Validate the zone.
  if (!_newZone.Valid())
  {
    std::cerr << "[RNDF::AddZone() Invalid zone ["
              << _newZone.Id() << "]" << std::endl;
    return false;
  }

  // Check whether the zone already exists.
  if (std::find(this->dataPtr->zones.begin(), this->dataPtr->zones.end(),
    _newZone) != this->dataPtr->zones.end())
  {
    std::cerr << "[RNDF::AddZone() error: Existing zone" << std::endl;
    return false;
  }

  this->dataPtr->zones.push_back(_newZone);
  assert(this->NumZones() == this->dataPtr->zones.size());
  return true;
}

//////////////////////////////////////////////////
bool RNDF::RemoveZone(const int _zoneId)
{
  rndf::Zone zone(_zoneId);
  return (this->dataPtr->zones.erase(std::remove(
    this->dataPtr->zones.begin(), this->dataPtr->zones.end(), zone),
      this->dataPtr->zones.end()) != this->dataPtr->zones.end());
}

//////////////////////////////////////////////////
std::string RNDF::Version() const
{
  return this->dataPtr->header.Version();
}

//////////////////////////////////////////////////
void RNDF::SetVersion(const std::string &_version) const
{
  this->dataPtr->header.SetVersion(_version);
}

//////////////////////////////////////////////////
std::string RNDF::Date() const
{
  return this->dataPtr->header.Date();
}

//////////////////////////////////////////////////
void RNDF::SetDate(const std::string &_newDate) const
{
  this->dataPtr->header.SetDate(_newDate);
}

//////////////////////////////////////////////////
bool RNDF::Valid() const
{
  if (this->Name().empty() || this->NumSegments() <= 0)
    return false;

  for (auto i = 0u; i < this->NumSegments(); ++i)
  {
    int expectedSegmentId = i + 1;
    const rndf::Segment &s = this->Segments().at(i);
    if (!s.Valid() || s.Id() != expectedSegmentId)
      return false;
  }

  for (auto i = 0u; i < this->NumZones(); ++i)
  {
    size_t expectedZoneId = this->NumSegments() + i + 1;
    const rndf::Zone &z = this->Zones().at(i);
    if (!z.Valid() || static_cast<size_t>(z.Id()) != expectedZoneId)
      return false;
  }

  return true;
}

//////////////////////////////////////////////////
void RNDF::UpdateCache()
{
  for (auto &segment : this->Segments())
    for (auto &lane : segment.Lanes())
      for (auto &wp : lane.Waypoints())
      {
        rndf::UniqueId id(segment.Id(), lane.Id(), wp.Id());
        rndf::RNDFNode node(id);
        node.SetSegment(&segment);
        node.SetLane(&lane);
        node.SetWaypoint(&wp);
        this->dataPtr->cache[id.String()] = node;
      }

  for (auto &zone : this->Zones())
  {
    for (auto &wp : zone.Perimeter().Points())
    {
      rndf::UniqueId id(zone.Id(), 0, wp.Id());
      rndf::RNDFNode node(id);
      node.SetZone(&zone);
      node.SetWaypoint(&wp);
      this->dataPtr->cache[id.String()] = node;
    }
    for (auto &spot : zone.Spots())
    {
      for (auto &wp : spot.Waypoints())
      {
        rndf::UniqueId id(zone.Id(), spot.Id(), wp.Id());
        rndf::RNDFNode node(id);
        node.SetZone(&zone);
        node.SetWaypoint(&wp);
        this->dataPtr->cache[id.String()] = node;
      }
    }
  }
}

//////////////////////////////////////////////////
RNDFNode *RNDF::Info(const rndf::UniqueId &_id) const
{
  if (this->dataPtr->cache.find(_id.String()) == this->dataPtr->cache.end())
    return nullptr;

  return &(this->dataPtr->cache[_id.String()]);
}
