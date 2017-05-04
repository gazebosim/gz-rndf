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

#ifndef IGNITION_RNDF_SEGMENT_HH_
#define IGNITION_RNDF_SEGMENT_HH_

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
    class ExitCacheEntry;
    class Lane;
    class SegmentHeaderPrivate;
    class SegmentPrivate;

    // \internal
    /// \brief An internal private segment header class.
    class SegmentHeader
    {
      /// \brief Default constructor.
      public: SegmentHeader();

      /// \brief Destructor.
      public: virtual ~SegmentHeader();

      ///////////
      /// Parsing
      ///////////

      /// \brief Load a segment header from an input stream coming from a
      /// text file. The expected format is the one specified on the RNDF spec.
      /// \param[in, out] _rndfFile Input file stream.
      /// \param[in, out] _lineNumber Line number pointed by the stream position
      /// indicator.
      /// \return True if a segment header block was found and parsed or
      /// false otherwise (e.g.: EoF or incorrect format found).
      public: bool Load(std::ifstream &_rndfFile,
                        int &_lineNumber);

      ////////
      /// Name
      ////////

      /// \brief Get the segment name. E.g.: "Wisconsin_Ave".
      /// \return The segment name.
      public: std::string Name() const;

      /// \brief Set the segment name. E.g.: "Wisconsin_Ave".
      /// \param[in] _name The new name.
      public: void SetName(const std::string &_name) const;

      /// \brief Smart pointer to private data.
      private: std::unique_ptr<SegmentHeaderPrivate> dataPtr;
    };

    /// \brief Abstraction for representing road segments. A road network is
    /// composed by one or more segments, each of which comprises one or more
    /// lanes.
    class IGNITION_RNDF_VISIBLE Segment
    {
      /// \brief Default constructor.
      /// \sa Valid.
      public: Segment();

      /// \brief Constructor.
      /// \param[in] _id Segment Id (a positive number).
      /// \sa Valid.
      public: explicit Segment(const int _id);

      /// \brief Copy constructor.
      /// \param[in] _other Other segment to copy from.
      /// \sa Valid.
      public: Segment(const Segment &_other);

      /// \brief Destructor.
      public: virtual ~Segment();

      ///////////
      /// Parsing
      ///////////

      /// \brief Load a segment from an input stream coming from a text file.
      /// The expected format is the one specified on the RNDF spec.
      /// \param[in, out] _rndfFile Input file stream.
      /// \param[in, out] _lineNumber Line number pointed by the stream position
      /// indicator.
      /// \param[in, out] _exitCache Cache of exits parsed.
      /// \param[in, out] _waypointCache Cache of waypoints parsed.
      /// \return True if a segment block was found and parsed or false
      /// otherwise (e.g.: EoF or incorrect format found).
      public: bool Load(std::ifstream &_rndfFile,
                        int &_lineNumber,
                        std::vector<ExitCacheEntry> &_exitCache,
                        std::vector<std::string> &_waypointCache);

      ///////
      /// Id
      ///////

      /// \brief Get the unique identifier of the segment.
      /// \return The segment Id.
      public: int Id() const;

      /// \brief Set the identifier of the segment.
      /// \param[in] _id New unique Id.
      /// \return True if the operation succeed or false otherwise
      /// (e.g.: if the id is not valid).
      /// \sa Valid.
      public: bool SetId(const int _id);

      /////////
      /// Lanes
      /////////

      /// \brief Get the number of lanes stored.
      /// \return The number of lanes in this segment.
      public: size_t NumLanes() const;

      /// \brief Get a mutable reference to the vector of lanes.
      /// \return A mutable reference to the vector of lanes.
      public: std::vector<rndf::Lane> &Lanes();

      /// \brief Get the vector of lanes.
      /// \return \return The vector of lanes.
      public: const std::vector<rndf::Lane> &Lanes() const;

      /// \brief Get the details of one of the lane with Id _laneId.
      /// \param[in] _laneId The lane Id.
      /// \param[out] _lane The lane requested.
      /// \return True if the lane was found or false otherwise.
      public: bool Lane(const int _laneId,
                        rndf::Lane &_lane) const;

      /// \brief Update an existing lane.
      /// \param[in] _lane The updated lane.
      /// \return True if the lane was found and updated or false otherwise.
      public: bool UpdateLane(const rndf::Lane &_lane);

      /// \brief Add a new lane.
      /// \param[in] _newLane A new lane to be added.
      /// \return True when the lane was successfully added to the list or
      /// false otherwise (e.g. if the Id of the lane was already existing
      /// or invalid).
      public: bool AddLane(const rndf::Lane &_newLane);

      /// \brief Remove an existing lane.
      /// \param[in] _laneId The lane Id to be removed.
      /// \return True when the lane was successfully deleted from the list
      /// or false otherwise (e.g. if the Id of the lane was not found
      /// or invalid).
      public: bool RemoveLane(const int _laneId);

      ////////
      /// Name
      ////////

      /// \brief Get the segment name. E.g.: "Wisconsin_Ave".
      /// \return The segment name.
      public: std::string Name() const;

      /// \brief Set the segment name. E.g.: "Wisconsin_Ave".
      /// \param[in] _name The new name.
      public: void SetName(const std::string &_name) const;

      //////////////
      /// Validation
      //////////////

      /// \return True if the segment is valid.
      public: bool Valid() const;

      /////////////
      /// Operators
      /////////////

      /// \brief Equality operator, result = this == _other
      /// \param[in] _other Segment to check for equality.
      /// \return true if this == _other
      public: bool operator==(const Segment &_other) const;

      /// \brief Inequality.
      /// \param[in] _other Segment to check for inequality.
      /// \return true if this != _other
      public: bool operator!=(const Segment &_other) const;

      /// \brief Assignment operator.
      /// \param[in] _other The new segment.
      /// \return A reference to this instance.
      public: Segment &operator=(const Segment &_other);

      /// \internal
      /// \brief Smart pointer to private data.
      private: std::unique_ptr<SegmentPrivate> dataPtr;
    };
  }
}
#endif
