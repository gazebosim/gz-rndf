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
#include <iostream>
#include <string>
#include <vector>

#include <fstream>

#include "ignition/rndf/Lane.hh"
#include "ignition/rndf/ParserUtils.hh"
#include "ignition/rndf/Segment.hh"

using namespace ignition;
using namespace rndf;

namespace ignition
{
  namespace rndf
  {
    /// \internal
    /// \brief Private data for SegmentHeader class.
    class SegmentHeaderPrivate
    {
      /// \brief Default constructor.
      public: SegmentHeaderPrivate() = default;

      /// \brief Destructor.
      public: virtual ~SegmentHeaderPrivate() = default;

      /// \brief Segment name.
      public: std::string name = "";
    };

    /// \internal
    /// \brief Private data for Segment class.
    class SegmentPrivate
    {
      /// \brief Constructor.
      /// \param[in] _id Segment Id.
      public: explicit SegmentPrivate(const int _id)
        : id(_id)
      {
      }

      /// \brief Destructor.
      public: virtual ~SegmentPrivate() = default;

      /// \brief Segment identifier. E.g.: 1
      public: int id = -1;

      /// \brief Collection of lanes.
      public: std::vector<rndf::Lane> lanes;

      /// Below are the optional segment header members.
      public: SegmentHeader header;
    };
  }
}

//////////////////////////////////////////////////
SegmentHeader::SegmentHeader()
{
  this->dataPtr.reset(new SegmentHeaderPrivate());
}

//////////////////////////////////////////////////
bool SegmentHeader::Load(std::ifstream &_rndfFile, const int _segmentId,
  int &_lineNumber)
{
  auto oldPos = _rndfFile.tellg();
  int oldLineNumber = _lineNumber;

  std::string lineread;
  nextRealLine(_rndfFile, lineread, _lineNumber);

  auto tokens = split(lineread, " ");
  if (tokens.size() == 2 && tokens.at(0) == "lane")
  {
    // Restore the file position and line number.
    // ParseHeader() shouldn't have any effect.
    _rndfFile.seekg(oldPos);
    _lineNumber = oldLineNumber;
    return true;
  }

  if (tokens.size() != 2 || tokens.at(0) != "segment_name")
  {
    // Invalid or header element.
    std::cerr << "[Line " << _lineNumber << "]: Unable to parse segment header "
              << "element" << std::endl;
    std::cerr << " \"" << lineread << "\"" << std::endl;
    return false;
  }

  assert(tokens.size() == 2);

  this->SetName(tokens.at(1));
  return true;
}

//////////////////////////////////////////////////
std::string SegmentHeader::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
void SegmentHeader::SetName(const std::string &_name) const
{
  this->dataPtr->name = _name;
}

//////////////////////////////////////////////////
Segment::Segment()
{
  this->dataPtr.reset(new SegmentPrivate(-1));
}

//////////////////////////////////////////////////
Segment::Segment(const int _id)
  : Segment()
{
  if (_id <= 0)
    return;

  this->SetId(_id);
}

//////////////////////////////////////////////////
Segment::Segment(const Segment &_other)
  : Segment()
{
  *this = _other;
}

//////////////////////////////////////////////////
Segment::~Segment()
{
}

//////////////////////////////////////////////////
bool Segment::Load(std::ifstream &_rndfFile, int &_lineNumber)
{
  int segmentId;
  if (!parsePositive(_rndfFile, "segment", segmentId, _lineNumber))
    return false;

  int numLanes;
  if (!parsePositive(_rndfFile, "num_lanes", numLanes, _lineNumber))
    return false;

  // Parse optional segment header (containing the segment name).
  SegmentHeader header;
  if (!header.Load(_rndfFile, segmentId, _lineNumber))
    return false;

  std::vector<rndf::Lane> lanes;
  for (auto i = 0; i < numLanes; ++i)
  {
    // Parse a lane.
    rndf::Lane lane;
    if (!lane.Load(_rndfFile, segmentId, _lineNumber))
      return false;

    // Check that all lanes are consecutive.
    if (lane.Id() != i + 1)
    {
      std::cerr << "[Line " << _lineNumber << "]: Found non-consecutive lane "
                << "Id [" << lane.Id() << "]" << std::endl;
      return false;
    }

    lanes.push_back(lane);
  }

  // Parse "end_segment".
  if (!parseDelimiter(_rndfFile, "end_segment", _lineNumber))
    return false;

  // Populate the segment.
  this->SetId(segmentId);
  this->Lanes() = lanes;
  this->SetName(header.Name());

  return true;
}

//////////////////////////////////////////////////
int Segment::Id() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
bool Segment::SetId(const int _id)
{
  bool valid = _id > 0;
  if (valid)
    this->dataPtr->id = _id;
  return valid;
}

//////////////////////////////////////////////////
unsigned int Segment::NumLanes() const
{
  return this->dataPtr->lanes.size();
}

//////////////////////////////////////////////////
std::vector<Lane> &Segment::Lanes()
{
  return this->dataPtr->lanes;
}

//////////////////////////////////////////////////
const std::vector<Lane> &Segment::Lanes() const
{
  return this->dataPtr->lanes;
}

//////////////////////////////////////////////////
bool Segment::Lane(const int _laneId, rndf::Lane &_lane) const
{
  auto it = std::find_if(this->dataPtr->lanes.begin(),
    this->dataPtr->lanes.end(),
    [_laneId](const rndf::Lane &_aLane)
    {
      return _aLane.Id() == _laneId;
    });

  bool found = it != this->dataPtr->lanes.end();
  if (found)
    _lane = *it;

  return found;
}

//////////////////////////////////////////////////
bool Segment::UpdateLane(const rndf::Lane &_lane)
{
  auto it = std::find(this->dataPtr->lanes.begin(),
    this->dataPtr->lanes.end(), _lane);

  bool found = it != this->dataPtr->lanes.end();
  if (found)
    *it = _lane;

  return found;
}

//////////////////////////////////////////////////
bool Segment::AddLane(const rndf::Lane &_newLane)
{
  // Validate the lane.
  if (!_newLane.Valid())
  {
    std::cerr << "[Segment::AddLane() Invalid lane Id ["
              << _newLane.Id() << "]" << std::endl;
    return false;
  }

  // Check whether the lane already exists.
  if (std::find(this->dataPtr->lanes.begin(), this->dataPtr->lanes.end(),
    _newLane) != this->dataPtr->lanes.end())
  {
    std::cerr << "[Segment::AddLane() error: Existing lane" << std::endl;
    return false;
  }

  this->dataPtr->lanes.push_back(_newLane);
  assert(this->NumLanes() == this->dataPtr->lanes.size());
  return true;
}

//////////////////////////////////////////////////
bool Segment::RemoveLane(const int _laneId)
{
  rndf::Lane lane(_laneId);
  return (this->dataPtr->lanes.erase(std::remove(
    this->dataPtr->lanes.begin(), this->dataPtr->lanes.end(), lane),
      this->dataPtr->lanes.end()) != this->dataPtr->lanes.end());
}

//////////////////////////////////////////////////
std::string Segment::Name() const
{
  return this->dataPtr->header.Name();
}

//////////////////////////////////////////////////
void Segment::SetName(const std::string &_name) const
{
  this->dataPtr->header.SetName(_name);
}

//////////////////////////////////////////////////
bool Segment::Valid() const
{
  if (this->Id() <= 0 || this->Lanes().size() <= 0)
    return false;

  for (auto i = 0u; i < this->NumLanes(); ++i)
  {
    int expectedLaneId = i + 1;
    const rndf::Lane &l = this->Lanes().at(i);
    if (!l.Valid() || l.Id() != expectedLaneId)
      return false;
  }

  return true;
}

//////////////////////////////////////////////////
bool Segment::operator==(const Segment &_other) const
{
  return this->Id() == _other.Id();
}

//////////////////////////////////////////////////
bool Segment::operator!=(const Segment &_other) const
{
  return !(*this == _other);
}

//////////////////////////////////////////////////
Segment &Segment::operator=(const Segment &_other)
{
  this->SetId(_other.Id());
  this->Lanes() = _other.Lanes();
  this->SetName(_other.Name());
  return *this;
}
