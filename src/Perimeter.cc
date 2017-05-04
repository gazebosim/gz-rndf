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
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <ignition/math/SphericalCoordinates.hh>

#include "ignition/rndf/Exit.hh"
#include "ignition/rndf/ParserUtils.hh"
#include "ignition/rndf/Perimeter.hh"
#include "ignition/rndf/Waypoint.hh"

using namespace ignition;
using namespace rndf;

namespace ignition
{
  namespace rndf
  {
    /// \internal
    /// \brief Private data for PerimeterHeader class.
    class PerimeterHeaderPrivate
    {
      /// \brief Default constructor.
      public: PerimeterHeaderPrivate() = default;

      /// \brief Destructor.
      public: virtual ~PerimeterHeaderPrivate() = default;

      /// \brief Collection of exits.
      public: std::vector<Exit> exits;
    };

    /// \internal
    /// \brief Private data for Perimeter class.
    class PerimeterPrivate
    {
      /// \brief Constructor.
      public: PerimeterPrivate() = default;

      /// \brief Destructor.
      public: virtual ~PerimeterPrivate() = default;

      /// \brief Collection of points.
      public: std::vector<Waypoint> points;

      /// Below are the optional perimeter header members.
      public: PerimeterHeader header;
    };
  }
}

//////////////////////////////////////////////////
PerimeterHeader::PerimeterHeader()
{
  this->dataPtr.reset(new PerimeterHeaderPrivate());
}

//////////////////////////////////////////////////
PerimeterHeader::~PerimeterHeader()
{
}

//////////////////////////////////////////////////
bool PerimeterHeader::Load(std::ifstream &_rndfFile, const int _zoneId,
  const int _perimeterId, const std::string &_lineread, int &_lineNumber,
  std::vector<ExitCacheEntry> &_exitCache)
{
  auto oldPos = _rndfFile.tellg();
  int oldLineNumber = _lineNumber;

  // We should leave if we don't find the "exit" element.
  rndf::Exit exit;
  while (exit.Load(_rndfFile, _zoneId, _perimeterId, _lineNumber))
  {
    this->AddExit(exit);

    // Add the exit to the cache.
    _exitCache.push_back({exit.ExitId().String(), exit.EntryId().String(),
      _lineNumber, _lineread});

    oldPos = _rndfFile.tellg();
    oldLineNumber = _lineNumber;
  }

  // Restore the file position and line number.
  // The next nextRealLine() call should point to the next element that is not
  // part of the header.
  _rndfFile.seekg(oldPos);
  _lineNumber = oldLineNumber;
  return true;
}

//////////////////////////////////////////////////
size_t PerimeterHeader::NumExits() const
{
  return this->dataPtr->exits.size();
}

//////////////////////////////////////////////////
std::vector<Exit> &PerimeterHeader::Exits()
{
  return this->dataPtr->exits;
}

//////////////////////////////////////////////////
bool PerimeterHeader::AddExit(const Exit &_newExit)
{
  // Validate the exit unique Id.
  if (!_newExit.Valid())
  {
    std::cerr << "PerimeterHeader::AddExit() Invalid exit [("
              << _newExit.ExitId() << ")(" << _newExit.EntryId()
              << ")]" << std::endl;
    return false;
  }

  // Check whether the exit already exists.
  if (std::find(this->dataPtr->exits.begin(),
        this->dataPtr->exits.end(), _newExit) != this->dataPtr->exits.end())
  {
    std::cerr << "PerimeterHeader::AddExit() error: Existing exit" << std::endl;
    return false;
  }

  this->dataPtr->exits.push_back(_newExit);
  assert(this->NumExits() == this->dataPtr->exits.size());
  return true;
}

//////////////////////////////////////////////////
bool PerimeterHeader::RemoveExit(const Exit &_exit)
{
  return (this->dataPtr->exits.erase(std::remove(
    this->dataPtr->exits.begin(), this->dataPtr->exits.end(), _exit),
      this->dataPtr->exits.end()) != this->dataPtr->exits.end());
}

//////////////////////////////////////////////////
Perimeter::Perimeter()
  : dataPtr(new PerimeterPrivate())
{
}

//////////////////////////////////////////////////
Perimeter::Perimeter(const Perimeter &_other)
  : Perimeter()
{
  *this = _other;
}

//////////////////////////////////////////////////
Perimeter::~Perimeter()
{
}

//////////////////////////////////////////////////
bool Perimeter::Load(std::ifstream &_rndfFile, const int _zoneId,
  int &_lineNumber, std::vector<ExitCacheEntry> &_exitCache,
  std::vector<std::string> &_waypointCache)
{
  std::string lineread;
  nextRealLine(_rndfFile, lineread, _lineNumber);

  // Parse the "perimeter Id" .
  auto tokens = split(lineread, " ");
  if (tokens.size() != 2 || tokens.at(0) != "perimeter")
  {
    std::cerr << "[Line " << _lineNumber << "]: Unable to parse perimeter "
              << "element" << std::endl;
    std::cerr << " \"" << lineread << "\"" << std::endl;
    return false;
  }

  auto perimeterIdTokens = split(tokens.at(1), ".");
  if (perimeterIdTokens.size() != 2                      ||
      perimeterIdTokens.at(0) != std::to_string(_zoneId) ||
      perimeterIdTokens.at(1) != "0")
  {
    std::cerr << "[Line " << _lineNumber << "]: Unable to parse perimeter "
              << "element" << std::endl;
    std::cerr << " \"" << lineread << "\"" << std::endl;
    return false;
  }

  // Parse "num_perimeterpoints".
  int numPoints;
  if (!parsePositive(_rndfFile, "num_perimeterpoints", numPoints, _lineNumber))
    return false;

  // Parse optional perimeter header.
  PerimeterHeader header;
  header.Load(_rndfFile, _zoneId, 0, lineread, _lineNumber, _exitCache);

  // Parse the perimeter points.
  std::vector<rndf::Waypoint> perimeterPoints;
  for (auto i = 0; i < numPoints; ++i)
  {
    rndf::Waypoint waypoint;
    if (!waypoint.Load(_rndfFile, _zoneId, 0, _lineNumber))
      return false;

    if (waypoint.Id() != i + 1)
    {
      std::cerr << "[Line " << _lineNumber << "]: Found non-consecutive "
                << "waypoint Id [" << waypoint.Id() << "]" << std::endl;
      return false;
    }

    // Set the exit flags if needed.
    for (auto const &exit : header.Exits())
    {
      if (exit.ExitId() == UniqueId(_zoneId, 0, waypoint.Id()))
      {
        waypoint.SetExit(true);
        break;
      }
    }

    perimeterPoints.push_back(waypoint);

    std::string wpStr(std::to_string(_zoneId) + ".0." +
      std::to_string(waypoint.Id()));
    _waypointCache.push_back(wpStr);
  }

  // Parse "end_perimeter".
  if (!parseDelimiter(_rndfFile, "end_perimeter", _lineNumber))
    return false;

  // Populate the perimeter.
  this->Points() = perimeterPoints;
  this->Exits() = header.Exits();

  return true;
}

//////////////////////////////////////////////////
size_t Perimeter::NumPoints() const
{
  return this->dataPtr->points.size();
}

//////////////////////////////////////////////////
std::vector<rndf::Waypoint> &Perimeter::Points()
{
  return this->dataPtr->points;
}

//////////////////////////////////////////////////
const std::vector<rndf::Waypoint> &Perimeter::Points() const
{
  return this->dataPtr->points;
}

//////////////////////////////////////////////////
bool Perimeter::Point(const int _wpId, rndf::Waypoint &_wp) const
{
  auto it = std::find_if(this->dataPtr->points.begin(),
    this->dataPtr->points.end(),
    [_wpId](const rndf::Waypoint &_waypoint)
    {
      return _waypoint.Id() == _wpId;
    });

  bool found = it != this->dataPtr->points.end();
  if (found)
    _wp = *it;

  return found;
}

//////////////////////////////////////////////////
bool Perimeter::UpdatePoint(const rndf::Waypoint &_wp)
{
  auto it = std::find(this->dataPtr->points.begin(),
    this->dataPtr->points.end(), _wp);

  bool found = it != this->dataPtr->points.end();
  if (found)
    *it = _wp;

  return found;
}

//////////////////////////////////////////////////
bool Perimeter::AddPoint(const rndf::Waypoint &_newWaypoint)
{
  // Validate the waypoint.
  if (!_newWaypoint.Valid())
  {
    std::cerr << "[Perimeter::AddPoint() Invalid point Id ["
              << _newWaypoint.Id() << "]" << std::endl;
    return false;
  }

  // Check whether the point already exists.
  if (std::find(this->dataPtr->points.begin(),
        this->dataPtr->points.end(), _newWaypoint) !=
          this->dataPtr->points.end())
  {
    std::cerr << "[Perimeter::AddPoint() error: Existing point" << std::endl;
    return false;
  }

  this->dataPtr->points.push_back(_newWaypoint);
  assert(this->NumPoints() == this->dataPtr->points.size());
  return true;
}

//////////////////////////////////////////////////
bool Perimeter::RemovePoint(const int _wpId)
{
  rndf::Waypoint wp(_wpId, ignition::math::SphericalCoordinates());
  return (this->dataPtr->points.erase(std::remove(
    this->dataPtr->points.begin(), this->dataPtr->points.end(), wp),
      this->dataPtr->points.end()) != this->dataPtr->points.end());
}

//////////////////////////////////////////////////
size_t Perimeter::NumExits() const
{
  return this->dataPtr->header.NumExits();
}

//////////////////////////////////////////////////
std::vector<Exit> &Perimeter::Exits()
{
  return this->dataPtr->header.Exits();
}

//////////////////////////////////////////////////
const std::vector<Exit> &Perimeter::Exits() const
{
  return this->dataPtr->header.Exits();
}

//////////////////////////////////////////////////
bool Perimeter::AddExit(const Exit &_newExit)
{
  return this->dataPtr->header.AddExit(_newExit);
}

//////////////////////////////////////////////////
bool Perimeter::RemoveExit(const Exit &_exit)
{
  return this->dataPtr->header.RemoveExit(_exit);
}

//////////////////////////////////////////////////
bool Perimeter::Valid() const
{
  if (this->NumPoints() <= 0)
    return false;

  // All points must be valid and consecutive.
  for (auto i = 0u; i < this->NumPoints(); ++i)
  {
    int expectedPerimeterPointId = i + 1;
    const rndf::Waypoint &w = this->Points().at(i);
    if (!w.Valid() || w.Id() != expectedPerimeterPointId)
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
bool Perimeter::operator==(const Perimeter &_other) const
{
  if ((this->Points().size() != _other.Points().size()) ||
      (this->Exits().size() != _other.Exits().size()))
  {
    return false;
  }

  for (auto const &point : this->Points())
  {
    if (std::find(_other.Points().begin(), _other.Points().end(), point) ==
          _other.Points().end())
    {
      return false;
    }
  }

  for (auto const &exit : this->Exits())
  {
    if (std::find(_other.Exits().begin(), _other.Exits().end(), exit) ==
          _other.Exits().end())
    {
      return false;
    }
  }

  return true;
}

//////////////////////////////////////////////////
bool Perimeter::operator!=(const Perimeter &_other) const
{
  return !(*this == _other);
}

//////////////////////////////////////////////////
Perimeter &Perimeter::operator=(const Perimeter &_other)
{
  this->Points() = _other.Points();
  this->Exits() = _other.Exits();
  return *this;
}
