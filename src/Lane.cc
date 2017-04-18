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
#include <vector>
#include <ignition/math/SphericalCoordinates.hh>

#include "ignition/rndf/Checkpoint.hh"
#include "ignition/rndf/Exit.hh"
#include "ignition/rndf/Lane.hh"
#include "ignition/rndf/ParserUtils.hh"
#include "ignition/rndf/Waypoint.hh"

using namespace ignition;
using namespace rndf;

namespace ignition
{
  namespace rndf
  {
    /// \internal
    /// \brief Private data for ZoneHeader class.
    class LaneHeaderPrivate
    {
      /// \brief Default constructor.
      public: LaneHeaderPrivate() = default;

      /// \brief Destructor.
      public: virtual ~LaneHeaderPrivate() = default;

      /// \brief Lane width in meters (non-negative).
      double width = 0.0;

      /// \brief Left boundary type.
      Marking leftBoundary = Marking::UNDEFINED;

      /// \brief Right boundary type.
      Marking rightBoundary = Marking::UNDEFINED;

      /// \brief Collection of waypoints that are checkpoints.
      std::vector<Checkpoint> checkpoints;

      /// \brief Collection of waypoints that are stops. We just store the
      /// waypoint Id.
      std::vector<int> stops;

      /// \brief Collection of exits.
      std::vector<Exit> exits;
    };

    /// \internal
    /// \brief Private data for Lane class.
    class LanePrivate
    {
      /// \brief Constructor.
      /// \param[in] _id Lane Id.
      public: explicit LanePrivate(const int _id)
        : id(_id)
      {
      }

      /// \brief Destructor.
      public: virtual ~LanePrivate() = default;

      /// \brief Lane identifier. E.g.: 1
      public: int id = -1;

      /// \brief Collection of waypoints.
      public: std::vector<Waypoint> waypoints;

      /// Below are the optional lane header members.
      LaneHeader header;
    };
  }
}

//////////////////////////////////////////////////
LaneHeader::LaneHeader()
{
  this->dataPtr.reset(new LaneHeaderPrivate());
}

//////////////////////////////////////////////////
bool LaneHeader::Load(std::ifstream &_rndfFile, const int _segmentId,
  const int _laneId, int &_lineNumber)
{
  double width = 0;
  Marking leftBoundary = Marking::UNDEFINED;
  Marking rightBoundary = Marking::UNDEFINED;
  std::vector<rndf::Checkpoint> checkpoints;
  std::vector<int> stops;
  std::vector<rndf::Exit> exits;

  // The width and left/right boundary options can only appear once.
  bool widthFound = false;
  bool leftBoundaryFound = false;
  bool rightBoundaryFound = false;
  bool done = false;

  do
  {
    auto oldPos = _rndfFile.tellg();
    int oldLineNumber = _lineNumber;

    std::string lineread;
    nextRealLine(_rndfFile, lineread, _lineNumber);

    auto tokens = split(lineread, " ");

    if ((tokens.size() < 2)                                  ||
       (tokens[0] == "lane_width"     && widthFound)         ||
       (tokens[0] == "left_boundary"  && leftBoundaryFound)  ||
       (tokens[0] == "right_boundary" && rightBoundaryFound))
    {
      // Invalid or repeated header element.
      std::cerr << "[Line " << _lineNumber << "]: Unable to parse lane header "
                << "element." << std::endl;
      std::cerr << " \"" << lineread << "\"" << std::endl;
      return false;
    }

    if (tokens[0] == "lane_width")
    {
      int widthFeet;
      if (!parseNonNegative(lineread, "lane_width", widthFeet))
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse "
                  << "lane width element" << std::endl;
        std::cerr << " \"" << lineread << "\"" << std::endl;
        return false;
      }

      // Convert from feet to meters.
      width = widthFeet * 0.3048;
      widthFound = true;
    }
    else if (tokens[0] == "left_boundary")
    {
      if (!parseBoundary(lineread, leftBoundary))
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse "
                  << "lane boundary element" << std::endl;
        std::cerr << " \"" << lineread << "\"" << std::endl;
        return false;
      }

      leftBoundaryFound = true;
    }
    else if (tokens[0] == "right_boundary")
    {
      if (!parseBoundary(lineread, rightBoundary))
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse "
                  << "lane boundary element" << std::endl;
        std::cerr << " \"" << lineread << "\"" << std::endl;
        return false;
      }

      rightBoundaryFound = true;
    }
    else if (tokens[0] == "checkpoint")
    {
      rndf::Checkpoint checkpoint;
      if (!parseCheckpoint(lineread, _segmentId, _laneId, checkpoint))
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse "
                  << "lane checkpoint element" << std::endl;
        std::cerr << " \"" << lineread << "\"" << std::endl;
        return false;
      }

      checkpoints.push_back(checkpoint);
    }
    else if (tokens[0] == "stop")
    {
      rndf::UniqueId stop;
      if (!parseStop(lineread, _segmentId, _laneId, stop))
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse "
                  << "lane stop element" << std::endl;
        std::cerr << " \"" << lineread << "\"" << std::endl;
        return false;
      }

      stops.push_back(stop.Z());
    }
    else if (tokens[0] == "exit")
    {
      rndf::Exit exit;
      if (!parseExit(lineread, _segmentId, _laneId, exit))
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse "
                  << "lane exit element" << std::endl;
        std::cerr << " \"" << lineread << "\"" << std::endl;
        return false;
      }

      exits.push_back(exit);
    }
    else
    {
      // This is the end of the header and the start of the waypoint section.
      // Restore the file position and line number.
      // ParseHeader() shouldn't have any effect.
      _rndfFile.seekg(oldPos);
      _lineNumber = oldLineNumber;
      done = true;
    }
  } while (!done);

  // Populate all fields.
  this->SetWidth(width);
  this->SetLeftBoundary(leftBoundary);
  this->SetRightBoundary(rightBoundary);
  this->Checkpoints() = checkpoints;
  this->Stops() = stops;
  this->Exits() = exits;

  return true;
}

//////////////////////////////////////////////////
double LaneHeader::Width() const
{
  return this->dataPtr->width;
}

//////////////////////////////////////////////////
bool LaneHeader::SetWidth(const double _newWidth)
{
  bool valid = _newWidth >= 0;
  if (!valid)
  {
    std::cerr << "LaneHeader::SetWidth() Invalid lane width [" << _newWidth
              << "]" << std::endl;
    return false;
  }

  this->dataPtr->width = _newWidth;
  return true;
}

//////////////////////////////////////////////////
Marking LaneHeader::LeftBoundary() const
{
  return this->dataPtr->leftBoundary;
}

//////////////////////////////////////////////////
void LaneHeader::SetLeftBoundary(const Marking &_boundary)
{
  this->dataPtr->leftBoundary = _boundary;
}

//////////////////////////////////////////////////
Marking LaneHeader::RightBoundary() const
{
  return this->dataPtr->rightBoundary;
}

//////////////////////////////////////////////////
void LaneHeader::SetRightBoundary(const Marking &_boundary)
{
  this->dataPtr->rightBoundary = _boundary;
}

//////////////////////////////////////////////////
size_t LaneHeader::NumCheckpoints() const
{
  return this->dataPtr->checkpoints.size();
}

//////////////////////////////////////////////////
std::vector<rndf::Checkpoint> &LaneHeader::Checkpoints()
{
  return this->dataPtr->checkpoints;
}

//////////////////////////////////////////////////
const std::vector<rndf::Checkpoint> &LaneHeader::Checkpoints() const
{
  return this->dataPtr->checkpoints;
}

//////////////////////////////////////////////////
bool LaneHeader::Checkpoint(const int _cpId, rndf::Checkpoint &_cp) const
{
  auto it = std::find_if(this->dataPtr->checkpoints.begin(),
    this->dataPtr->checkpoints.end(),
    [_cpId](const rndf::Checkpoint &_checkpoint)
    {
      return _checkpoint.CheckpointId() == _cpId;
    });

  bool found = it != this->dataPtr->checkpoints.end();
  if (found)
    _cp = *it;

  return found;
}

//////////////////////////////////////////////////
bool LaneHeader::UpdateCheckpoint(const rndf::Checkpoint &_cp)
{
  auto it = std::find(this->dataPtr->checkpoints.begin(),
    this->dataPtr->checkpoints.end(), _cp);

  bool found = it != this->dataPtr->checkpoints.end();
  if (found)
    *it = _cp;

  return found;
}

//////////////////////////////////////////////////
bool LaneHeader::AddCheckpoint(const rndf::Checkpoint &_newCheckpoint)
{
  // Validate the checkpoint.
  if (!_newCheckpoint.Valid())
  {
    std::cerr << "[Lane::AddCheckpoint() Invalid checkpoint: "
              << "checkpointId [" << _newCheckpoint.CheckpointId() << "], "
              << "waypointId [" << _newCheckpoint.WaypointId() << "]"
              << std::endl;
    return false;
  }

  // Check whether the checkpoint already exists.
  if (std::find(this->dataPtr->checkpoints.begin(),
        this->dataPtr->checkpoints.end(), _newCheckpoint) !=
          this->dataPtr->checkpoints.end())
  {
    std::cerr << "[Lane::AddCheckpoint() error: Existing checkpoint"
              << std::endl;
    return false;
  }

  this->dataPtr->checkpoints.push_back(_newCheckpoint);
  assert(this->NumCheckpoints() == this->dataPtr->checkpoints.size());
  return true;
}

//////////////////////////////////////////////////
bool LaneHeader::RemoveCheckpoint(const int _cpId)
{
  rndf::Checkpoint cp(_cpId, 1);
  return (this->dataPtr->checkpoints.erase(std::remove(
    this->dataPtr->checkpoints.begin(), this->dataPtr->checkpoints.end(), cp),
      this->dataPtr->checkpoints.end()) != this->dataPtr->checkpoints.end());
}

//////////////////////////////////////////////////
size_t LaneHeader::NumStops() const
{
  return this->dataPtr->stops.size();
}

//////////////////////////////////////////////////
std::vector<int> &LaneHeader::Stops()
{
  return this->dataPtr->stops;
}

//////////////////////////////////////////////////
const std::vector<int> &LaneHeader::Stops() const
{
  return this->dataPtr->stops;
}

//////////////////////////////////////////////////
bool LaneHeader::AddStop(const int _waypointId)
{
  // Validate the waypoint Id.
  if (_waypointId <= 0)
  {
    std::cerr << "[Lane::AddStop() Invalid waypoint Id: [" << _waypointId
              << "]" << std::endl;
    return false;
  }

  // Check whether the stop already exists.
  if (std::find(this->dataPtr->stops.begin(),
        this->dataPtr->stops.end(), _waypointId) != this->dataPtr->stops.end())
  {
    std::cerr << "[Lane::AddStop() error: Existing waypoint" << std::endl;
    return false;
  }

  this->dataPtr->stops.push_back(_waypointId);
  assert(this->NumStops() == this->dataPtr->stops.size());
  return true;
}

//////////////////////////////////////////////////
bool LaneHeader::RemoveStop(const int _waypointId)
{
  return (this->dataPtr->stops.erase(std::remove(
    this->dataPtr->stops.begin(), this->dataPtr->stops.end(), _waypointId),
      this->dataPtr->stops.end()) != this->dataPtr->stops.end());
}

//////////////////////////////////////////////////
size_t LaneHeader::NumExits() const
{
  return this->dataPtr->exits.size();
}

//////////////////////////////////////////////////
std::vector<Exit> &LaneHeader::Exits()
{
  return this->dataPtr->exits;
}

//////////////////////////////////////////////////
const std::vector<Exit> &LaneHeader::Exits() const
{
  return this->dataPtr->exits;
}

//////////////////////////////////////////////////
bool LaneHeader::AddExit(const Exit &_newExit)
{
  // Validate the exit unique Id.
  if (!_newExit.ExitId().Valid())
  {
    std::cerr << "[Lane::AddExit() Invalid exit Id: [" << _newExit.ExitId()
              << "]" << std::endl;
    return false;
  }

  // Validate the entry unique Id.
  if (!_newExit.EntryId().Valid())
  {
    std::cerr << "[Lane::AddExit() Invalid entry Id: [" << _newExit.EntryId()
              << "]" << std::endl;
    return false;
  }

  // Check whether the exit already exists.
  if (std::find(this->dataPtr->exits.begin(),
        this->dataPtr->exits.end(), _newExit) != this->dataPtr->exits.end())
  {
    std::cerr << "[Lane::AddExit() error: Existing exit" << std::endl;
    return false;
  }

  this->dataPtr->exits.push_back(_newExit);
  assert(this->NumExits() == this->dataPtr->exits.size());
  return true;
}

//////////////////////////////////////////////////
bool LaneHeader::RemoveExit(const Exit &_exit)
{
  return (this->dataPtr->exits.erase(std::remove(
    this->dataPtr->exits.begin(), this->dataPtr->exits.end(), _exit),
      this->dataPtr->exits.end()) != this->dataPtr->exits.end());
}

//////////////////////////////////////////////////
Lane::Lane()
{
  this->dataPtr.reset(new LanePrivate(-1));
}

//////////////////////////////////////////////////
Lane::Lane(const int _id)
  : Lane()
{
  if (_id <= 0)
    return;

  this->SetId(_id);
}

//////////////////////////////////////////////////
Lane::Lane(const Lane &_other)
  : Lane(_other.Id())
{
  *this = _other;
}

//////////////////////////////////////////////////
Lane::~Lane()
{
}

//////////////////////////////////////////////////
bool Lane::Load(std::ifstream &_rndfFile, const int _segmentId,
  int &_lineNumber)
{
  std::string lineread;

  nextRealLine(_rndfFile, lineread, _lineNumber);

  // Parse the "lane ID" .
  auto tokens = split(lineread, " ");
  if (tokens.size() != 2 || tokens.at(0) != "lane")
  {
    std::cerr << "[Line " << _lineNumber << "]: Unable to parse lane element"
              << std::endl;
    std::cerr << " \"" << lineread << "\"" << std::endl;
    return false;
  }

  auto laneIdTokens = split(tokens.at(1), ".");
  if (laneIdTokens.size() != 2 ||
      laneIdTokens.at(0) != std::to_string(_segmentId))
  {
    std::cerr << "[Line " << _lineNumber << "]: Unable to parse lane element"
              << std::endl;
    std::cerr << " \"" << lineread << "\"" << std::endl;
    return false;
  }

  std::string::size_type sz;
  int laneId;

  try
  {
    laneId = std::stoi(laneIdTokens.at(1), &sz);
  }
  catch(...)
  {
    std::cerr << "[Line " << _lineNumber << "]: Unable to parse lane element"
              << std::endl;
    std::cerr << " \"" << lineread << "\"" << std::endl;
    return false;
  }

  if (laneId <= 0 || laneId > 32768 || sz != laneIdTokens.at(1).size())
  {
    std::cerr << "[Line " << _lineNumber << "]: Out of range value ["
              << laneId << "]" << std::endl;
    std::cerr << " \"" << lineread << "\"" << std::endl;
    return false;
  }

  // Parse "num_waypoints".
  int numWaypoints;
  if (!parsePositive(_rndfFile, "num_waypoints", numWaypoints, _lineNumber))
    return false;

  // Parse optional lane header.
  LaneHeader header;
  if (!header.Load(_rndfFile, _segmentId, laneId, _lineNumber))
    return false;

  // Parse waypoints.
  std::vector<rndf::Waypoint> waypoints;
  for (auto i = 0; i < numWaypoints; ++i)
  {
    rndf::Waypoint waypoint;
    if (!waypoint.Load(_rndfFile, _segmentId, laneId, _lineNumber))
      return false;

    if (waypoint.Id() != i + 1)
    {
      std::cerr << "[Line " << _lineNumber << "]: Found non-consecutive "
                << "waypoint Id [" << waypoint.Id() << "]" << std::endl;
      return false;
    }

    waypoints.push_back(waypoint);
  }

  // Parse "end_lane".
  if (!parseDelimiter(_rndfFile, "end_lane", _lineNumber))
    return false;

  // Populate the lane.
  this->SetId(laneId);
  this->Waypoints() = waypoints;
  this->SetWidth(header.Width());
  this->SetLeftBoundary(header.LeftBoundary());
  this->SetRightBoundary(header.RightBoundary());
  this->Checkpoints() = header.Checkpoints();
  this->Stops() = header.Stops();
  this->Exits() = header.Exits();

  return true;
}

//////////////////////////////////////////////////
int Lane::Id() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
bool Lane::SetId(const int _id)
{
  bool valid = _id > 0;
  if (valid)
    this->dataPtr->id = _id;
  return valid;
}

//////////////////////////////////////////////////
size_t Lane::NumWaypoints() const
{
  return this->dataPtr->waypoints.size();
}

//////////////////////////////////////////////////
std::vector<rndf::Waypoint> &Lane::Waypoints()
{
  return this->dataPtr->waypoints;
}

//////////////////////////////////////////////////
const std::vector<rndf::Waypoint> &Lane::Waypoints() const
{
  return this->dataPtr->waypoints;
}

//////////////////////////////////////////////////
bool Lane::Waypoint(const int _wpId, rndf::Waypoint &_wp) const
{
  auto it = std::find_if(this->dataPtr->waypoints.begin(),
    this->dataPtr->waypoints.end(),
    [_wpId](const rndf::Waypoint &_waypoint)
    {
      return _waypoint.Id() == _wpId;
    });

  bool found = it != this->dataPtr->waypoints.end();
  if (found)
    _wp = *it;

  return found;
}

//////////////////////////////////////////////////
bool Lane::UpdateWaypoint(const rndf::Waypoint &_wp)
{
  auto it = std::find(this->dataPtr->waypoints.begin(),
    this->dataPtr->waypoints.end(), _wp);

  bool found = it != this->dataPtr->waypoints.end();
  if (found)
    *it = _wp;

  return found;
}

//////////////////////////////////////////////////
bool Lane::AddWaypoint(const rndf::Waypoint &_newWaypoint)
{
  // Validate the waypoint.
  if (!_newWaypoint.Valid())
  {
    std::cerr << "[Lane::Addwaypoint() Invalid waypoint Id ["
              << _newWaypoint.Id() << "]" << std::endl;
    return false;
  }

  // Check whether the waypoint already exists.
  if (std::find(this->dataPtr->waypoints.begin(),
        this->dataPtr->waypoints.end(), _newWaypoint) !=
          this->dataPtr->waypoints.end())
  {
    std::cerr << "[Lane::AddWaypoint() error: Existing waypoint" << std::endl;
    return false;
  }

  this->dataPtr->waypoints.push_back(_newWaypoint);
  assert(this->NumWaypoints() == this->dataPtr->waypoints.size());
  return true;
}

//////////////////////////////////////////////////
bool Lane::RemoveWaypoint(const int _wpId)
{
  rndf::Waypoint wp(_wpId, ignition::math::SphericalCoordinates());
  return (this->dataPtr->waypoints.erase(std::remove(
    this->dataPtr->waypoints.begin(), this->dataPtr->waypoints.end(), wp),
      this->dataPtr->waypoints.end()) != this->dataPtr->waypoints.end());
}

//////////////////////////////////////////////////
double Lane::Width() const
{
  return this->dataPtr->header.Width();
}

//////////////////////////////////////////////////
bool Lane::SetWidth(const double _newWidth)
{
  return this->dataPtr->header.SetWidth(_newWidth);
}

//////////////////////////////////////////////////
Marking Lane::LeftBoundary() const
{
  return this->dataPtr->header.LeftBoundary();
}

//////////////////////////////////////////////////
void Lane::SetLeftBoundary(const Marking &_boundary)
{
  this->dataPtr->header.SetLeftBoundary(_boundary);
}

//////////////////////////////////////////////////
Marking Lane::RightBoundary() const
{
  return this->dataPtr->header.RightBoundary();
}

//////////////////////////////////////////////////
void Lane::SetRightBoundary(const Marking &_boundary)
{
  return this->dataPtr->header.SetRightBoundary(_boundary);
}

//////////////////////////////////////////////////
size_t Lane::NumCheckpoints() const
{
  return this->dataPtr->header.NumCheckpoints();
}

//////////////////////////////////////////////////
std::vector<rndf::Checkpoint> &Lane::Checkpoints()
{
  return this->dataPtr->header.Checkpoints();
}

//////////////////////////////////////////////////
const std::vector<rndf::Checkpoint> &Lane::Checkpoints() const
{
  return this->dataPtr->header.Checkpoints();
}

//////////////////////////////////////////////////
bool Lane::Checkpoint(const int _cpId, rndf::Checkpoint &_cp) const
{
  return this->dataPtr->header.Checkpoint(_cpId, _cp);
}

//////////////////////////////////////////////////
bool Lane::UpdateCheckpoint(const rndf::Checkpoint &_cp)
{
  return this->dataPtr->header.UpdateCheckpoint(_cp);
}

//////////////////////////////////////////////////
bool Lane::AddCheckpoint(const rndf::Checkpoint &_newCheckpoint)
{
  return this->dataPtr->header.AddCheckpoint(_newCheckpoint);
}

//////////////////////////////////////////////////
bool Lane::RemoveCheckpoint(const int _cpId)
{
  return this->dataPtr->header.RemoveCheckpoint(_cpId);
}

//////////////////////////////////////////////////
size_t Lane::NumStops() const
{
  return this->dataPtr->header.NumStops();
}

//////////////////////////////////////////////////
std::vector<int> &Lane::Stops()
{
  return this->dataPtr->header.Stops();
}

//////////////////////////////////////////////////
const std::vector<int> &Lane::Stops() const
{
  return this->dataPtr->header.Stops();
}

//////////////////////////////////////////////////
bool Lane::AddStop(const int _waypointId)
{
  return this->dataPtr->header.AddStop(_waypointId);
}

//////////////////////////////////////////////////
bool Lane::RemoveStop(const int _waypointId)
{
  return this->dataPtr->header.RemoveStop(_waypointId);
}

//////////////////////////////////////////////////
size_t Lane::NumExits() const
{
  return this->dataPtr->header.NumExits();
}

//////////////////////////////////////////////////
std::vector<Exit> &Lane::Exits()
{
  return this->dataPtr->header.Exits();
}

//////////////////////////////////////////////////
const std::vector<Exit> &Lane::Exits() const
{
  return this->dataPtr->header.Exits();
}

//////////////////////////////////////////////////
bool Lane::AddExit(const Exit &_newExit)
{
  return this->dataPtr->header.AddExit(_newExit);
}

//////////////////////////////////////////////////
bool Lane::RemoveExit(const Exit &_exit)
{
  return this->dataPtr->header.RemoveExit(_exit);
}

//////////////////////////////////////////////////
bool Lane::Valid() const
{
  if (this->Id() <= 0 || this->NumWaypoints() <= 0)
    return false;

  // All waypoints must be valid and consecutive.
  for (auto i = 0u; i < this->NumWaypoints(); ++i)
  {
    int expectedWaypointId = i + 1;
    const rndf::Waypoint &w = this->Waypoints().at(i);
    if (!w.Valid() || w.Id() != expectedWaypointId)
      return false;
  }

  for (auto const &c : this->Checkpoints())
  {
    if (!c.Valid())
      return false;
  }

  for (auto &s : this->Stops())
  {
    if (s <= 0)
      return false;
  }

  for (auto const &e : this->Exits())
  {
    if (!e.Valid())
      return false;
  }

  return true;
}

//////////////////////////////////////////////////
bool Lane::operator==(const Lane &_other) const
{
  return this->Id() == _other.Id();
}

//////////////////////////////////////////////////
bool Lane::operator!=(const Lane &_other) const
{
  return !(*this == _other);
}

//////////////////////////////////////////////////
Lane &Lane::operator=(const Lane &_other)
{
  this->SetId(_other.Id());
  this->Waypoints() = _other.Waypoints();
  this->SetWidth(_other.Width());
  this->SetLeftBoundary(_other.LeftBoundary());
  this->SetRightBoundary(_other.RightBoundary());
  this->Checkpoints() = _other.Checkpoints();
  this->Stops() = _other.Stops();
  this->Exits() = _other.Exits();
  return *this;
}
