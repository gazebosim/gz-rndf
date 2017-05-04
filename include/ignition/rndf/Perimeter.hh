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

#ifndef IGNITION_RNDF_PERIMETER_HH_
#define IGNITION_RNDF_PERIMETER_HH_

#include <iosfwd>
#include <memory>
#include <string>
#include <vector>

#include "ignition/rndf/Helpers.hh"

namespace ignition
{
  namespace rndf
  {
    // Forward declarations.
    class Exit;
    class ExitCacheEntry;
    class PerimeterHeaderPrivate;
    class PerimeterPrivate;
    class Waypoint;

    /// \internal
    /// \brief An internal private perimeter header class.
    class PerimeterHeader
    {
      /// \brief Default constructor.
      public: PerimeterHeader();

      /// \brief Destructor.
      public: virtual ~PerimeterHeader();

      ///////////
      /// Parsing
      ///////////

      /// \brief Load a perimeter header from an input stream coming from a
      /// text file. The expected format is the one specified on the RNDF spec.
      /// \param[in, out] _rndfFile Input file stream.
      /// \param[in] _zoneId The zone Id in which the spot is located.
      /// \param[in] _perimeterId The perimeter Id.
      /// \param[in, out] _lineNumber Line number pointed by the stream position
      /// indicator.
      /// \return True if a perimeter header block was found and parsed or
      /// false otherwise (e.g.: EoF or incorrect format found).
      public: bool Load(std::ifstream &_rndfFile,
                        const int _zoneId,
                        const int _perimeterId,
                        const std::string &_lineread,
                        int &_lineNumber,
                        std::vector<ExitCacheEntry> &_exitCache);

      /////////
      /// Exits
      /////////

      /// \brief Get the number of exits stored.
      /// \return The number of exits in the current lane.
      public: size_t NumExits() const;

      /// \brief Get a mutable reference to the vector of exits.
      /// \return A mutable reference to the vector of exits.
      public: std::vector<Exit> &Exits();

      /// \brief Add a new exit.
      /// \param[in] _newExit The exit to add.
      /// \return True when the exit was successfully added or
      /// false otherwise (e.g. if the exit  was already existing or invalid).
      public: bool AddExit(const Exit &_newExit);

      /// \brief Remove an existing exit.
      /// \param[in] _exit The exit to be removed.
      /// \return True when the exit was successfully deleted
      /// or false otherwise (e.g. if the exit was not found or invalid).
      public: bool RemoveExit(const Exit &_exit);

      /// \brief Smart pointer to private data.
      private: std::unique_ptr<PerimeterHeaderPrivate> dataPtr;
    };

    /// \brief Abstraction for representing a perimeter as a collection of
    /// perimeter points. A perimeter point can be tagged as "exit" if it's
    /// posible to leave the zone (represented by the perimeter) and jump to a
    /// different segment.
    class IGNITION_RNDF_VISIBLE Perimeter
    {
      /// \brief Default constructor.
      /// \sa Valid.
      public: Perimeter();

      /// \brief Copy constructor.
      /// \param[in] _other Other segment to copy from.
      /// \sa Valid.
      public: Perimeter(const Perimeter &_other);

      /// \brief Destructor.
      public: virtual ~Perimeter();

      ///////////
      /// Parsing
      ///////////

      /// \brief Load a perimeter from an input stream coming from a text file.
      /// The expected format is the one specified on the RNDF spec.
      /// \param[in, out] _rndfFile Input file stream.
      /// \param[in] _zoneId The zone Id in which the perimeter is located.
      /// \param[in, out] _lineNumber Line number pointed by the stream position
      /// indicator.
      /// \return True if a perimeter block was found and parsed or false
      /// otherwise (e.g.: EoF or incorrect format found).
      public: bool Load(std::ifstream &_rndfFile,
                        const int _zoneId,
                        int &_lineNumber,
                        std::vector<ExitCacheEntry> &_exitCache,
                        std::vector<std::string> &_waypointCache);

      ////////////////////
      /// Perimeter points
      ////////////////////

      /// \brief Get the number of perimeter points stored.
      /// \return The number of perimeter points in the current perimeter.
      public: size_t NumPoints() const;

      /// \brief Get a mutable reference to the vector of perimeter points.
      /// \return A mutable reference to the vector of perimeter points.
      public: std::vector<rndf::Waypoint> &Points();

      /// \brief Get the vector of perimeter points.
      /// \return \return The vector of perimeter points.
      public: const std::vector<rndf::Waypoint> &Points() const;

      /// \brief Get the details of one of the points with Id _wpId.
      /// \param[in] _wpId The point Id.
      /// \param[out] _wp The waypoint requested.
      /// \return True if the point was found or false otherwise.
      public: bool Point(const int _wpId,
                         rndf::Waypoint &_wp) const;

      /// \brief Update an existing point.
      /// \param[in] _wp The updated waypoint.
      /// \return True if the point was found and updated or false otherwise.
      public: bool UpdatePoint(const rndf::Waypoint &_wp);

      /// \brief Add a new perimeter point.
      /// \param[in] _newWaypoint A new waypoint to be added.
      /// \return True when the waypoint was successfully added to the list or
      /// false otherwise (e.g. if the Id of the waypoint was already existing
      /// or invalid).
      public: bool AddPoint(const rndf::Waypoint &_newWaypoint);

      /// \brief Remove an existing perimeter point.
      /// \param[in] _wpId The waypoint Id to be removed.
      /// \return True when the waypoint was successfully deleted from the list
      /// or false otherwise (e.g. if the Id of the waypoint was not found
      /// or invalid).
      public: bool RemovePoint(const int _wpId);

      /////////
      /// Exits
      /////////

      /// \brief Get the number of exits stored.
      /// \return The number of exits in the current lane.
      public: size_t NumExits() const;

      /// \brief Get a mutable reference to the vector of exits.
      /// \return A mutable reference to the vector of exits.
      public: std::vector<Exit> &Exits();

      /// \brief Get the vector of stops. The elements are waypoint Ids.
      /// \return The vector of stops.
      public: const std::vector<Exit> &Exits() const;

      /// \brief Add a new exit.
      /// \param[in] _newExit The exit to add.
      /// \return True when the exit was successfully added or
      /// false otherwise (e.g. if the exit  was already existing or invalid).
      public: bool AddExit(const Exit &_newExit);

      /// \brief Remove an existing exit.
      /// \param[in] _exit The exit to be removed.
      /// \return True when the exit was successfully deleted
      /// or false otherwise (e.g. if the exit was not found or invalid).
      public: bool RemoveExit(const Exit &_exit);

      //////////////
      /// Validation
      //////////////

      /// \return True if the parking spot is valid.
      public: bool Valid() const;

      /////////////
      /// Operators
      /////////////

      /// \brief Equality operator, result = this == _other
      /// \param[in] _other Perimeter to check for equality.
      /// \return true if this == _other
      public: bool operator==(const Perimeter &_other) const;

      /// \brief Inequality.
      /// \param[in] _other Perimeter to check for inequality.
      /// \return true if this != _other
      public: bool operator!=(const Perimeter &_other) const;

      /// \brief Assignment operator.
      /// \param[in] _other The new perimeter.
      /// \return A reference to this instance.
      public: Perimeter &operator=(const Perimeter &_other);

      /// \internal
      /// \brief Smart pointer to private data.
      private: std::unique_ptr<PerimeterPrivate> dataPtr;
    };
  }
}
#endif
