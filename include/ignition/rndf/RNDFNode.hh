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

#ifndef IGNITION_RNDF_RNDFNODE_HH_
#define IGNITION_RNDF_RNDFNODE_HH_

#include <memory>

#include "ignition/rndf/Helpers.hh"

#ifdef _WIN32
// Disable warning C4251 which is triggered by
// std::unique_ptr
#pragma warning(push)
#pragma warning(disable: 4251)
#endif

namespace ignition
{
  namespace rndf
  {
    // Forward declarations.
    class Lane;
    class RNDFNodePrivate;
    class Segment;
    class UniqueId;
    class Waypoint;
    class Zone;

    // \internal
    /// \brief An RNDF node class. Stores all the information associated with a
    /// given a unique Id .
    class IGNITION_RNDF_VISIBLE RNDFNode
    {
      /// \brief Default constructor.
      /// \sa Valid.
      public: RNDFNode();

      /// \brief Default constructor.
      /// \param[in] _id A unique Id.
      public: explicit RNDFNode(const rndf::UniqueId &_id);

      /// \brief Copy constructor.
      /// \param[in] _other Other RNDFNode.
      public: explicit RNDFNode(const RNDFNode &_other);

      /// \brief Destructor.
      public: virtual ~RNDFNode();

      /// \brief Get the Unique Id of the RNDF node.
      /// \return The Unique Id.
      public: rndf::UniqueId &UniqueId() const;

      /// \brief Get the pointer to the segment where the waypoint is contained.
      /// \return Pointer to the segment or nullptr if not possible (e.g. if
      /// the waypoint belongs to a zone).
      public: rndf::Segment *Segment() const;

      /// \brief Get the pointer to the lane where the waypoint is contained.
      /// \return Pointer to the lane or nullptr if not possible (e.g. if
      /// the waypoint belongs to a zone).
      public: rndf::Lane *Lane() const;

      /// \brief Get the pointer to the zone where the waypoint is contained.
      /// \return Pointer to the zone or nullptr if not possible (e.g. if
      /// the waypoint belongs to a segment).
      public: rndf::Zone *Zone() const;

      /// \brief Get the pointer to the waypoint with the stored unique Id.
      /// \return Pointer to the waypoint or nullptr if not possible (e.g. if
      /// the Id passed in the constructor was incorrect).
      public: rndf::Waypoint *Waypoint() const;

      /// \brief Set the unique unique Id.
      /// \param[in] _id Unique Id of the node.
      public: void SetUniqueId(const rndf::UniqueId &_id);

      /// \brief Set the pointer to the segment that contains the waypoint.
      /// \param[in] _segment Pointer to the segment that contains the waypoint
      /// or nullpr if there's no segment (e.g. ig the waypoint belongs to a
      /// zone).
      public: void SetSegment(rndf::Segment *_segment);

      /// \brief Set the pointer to the lane that contains the waypoint.
      /// \param[in] _lane Pointer to the lane that contains the waypoint
      /// or nullpr if there's no lane (e.g. ig the waypoint belongs to a
      /// zone).
      public: void SetLane(rndf::Lane *_lane);

      /// \brief Set the pointer to the zone that contains the waypoint.
      /// \param[in] _zone Pointer to the zone that contains the waypoint
      /// or nullpr if there's no zone (e.g. ig the waypoint belongs to a
      /// segment).
      public: void SetZone(rndf::Zone *_zone);

      /// \brief Set the pointer to the waypoint associated with the given
      /// unique Id passed in the constructor.
      /// \param[in] _segment Pointer to the waypoint with the given unique
      /// Id passed in the constructor or null not possible (e.g. if the Id
      // pased in the constructor was incorrect).
      public: void SetWaypoint(rndf::Waypoint *_waypoint);

      /////////////
      /// Operators
      /////////////

      /// \brief Equality operator, result = this == _other
      /// \param[in] _other RNDFNode to check for equality.
      /// \return true if this == _other
      public: bool operator==(const RNDFNode &_other) const;

      /// \brief Inequality.
      /// \param[in] _other RNDFNode to check for inequality.
      /// \return true if this != _other
      public: bool operator!=(const RNDFNode &_other) const;

      /// \brief Assignment operator.
      /// \param[in] _other The new RNDFNode.
      /// \return A reference to this instance.
      public: RNDFNode &operator=(const RNDFNode &_other);

      /// \internal
      /// \brief Smart pointer to private data.
      private: std::unique_ptr<RNDFNodePrivate> dataPtr;
    };
  }
}

#ifdef _WIN32
#pragma warning(pop)
#endif

#endif
