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

#include "ignition/rndf/Lane.hh"
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
    /// \brief Private data for RNDFNode class.
    class RNDFNodePrivate
    {
      /// \brief Constructor.
      public: explicit RNDFNodePrivate(const rndf::UniqueId &_id)
        : uniqueId(_id)
      {
      }

      /// \brief Destructor.
      public: virtual ~RNDFNodePrivate() = default;

      /// \brief The segment containing the waypoint.
      public: Segment *segment = nullptr;

      /// \brief The lane containing the waypoint.
      public: Lane *lane = nullptr;

      /// \brief The zone containing the waypoint.
      public: Zone *zone = nullptr;

      /// \brief The pointer to the waypoint that matches _id.
      public: Waypoint *waypoint = nullptr;

      /// \brief The unique Id.
      public: UniqueId uniqueId;
    };
  }
}

//////////////////////////////////////////////////
RNDFNode::RNDFNode()
  : dataPtr(new RNDFNodePrivate(rndf::UniqueId()))
{
}

//////////////////////////////////////////////////
RNDFNode::RNDFNode(const rndf::UniqueId &_id)
  : dataPtr(new RNDFNodePrivate(_id))
{
}

//////////////////////////////////////////////////
RNDFNode::RNDFNode(const rndf::RNDFNode &_other)
  : RNDFNode(_other.UniqueId())
{
  *this = _other;
}

//////////////////////////////////////////////////
RNDFNode::~RNDFNode()
{
}

//////////////////////////////////////////////////
UniqueId &RNDFNode::UniqueId() const
{
  return this->dataPtr->uniqueId;
}

//////////////////////////////////////////////////
Segment *RNDFNode::Segment() const
{
  return this->dataPtr->segment;
}

//////////////////////////////////////////////////
Lane *RNDFNode::Lane() const
{
  return this->dataPtr->lane;
}

//////////////////////////////////////////////////
Zone *RNDFNode::Zone() const
{
  return this->dataPtr->zone;
}

//////////////////////////////////////////////////
Waypoint *RNDFNode::Waypoint() const
{
  return this->dataPtr->waypoint;
}

//////////////////////////////////////////////////
void RNDFNode::SetUniqueId(const rndf::UniqueId &_id)
{
  this->dataPtr->uniqueId = _id;
}

//////////////////////////////////////////////////
void RNDFNode::SetSegment(rndf::Segment *_segment)
{
  this->dataPtr->segment = _segment;
}

//////////////////////////////////////////////////
void RNDFNode::SetLane(rndf::Lane *_lane)
{
  this->dataPtr->lane = _lane;
}

//////////////////////////////////////////////////
void RNDFNode::SetZone(rndf::Zone *_zone)
{
  this->dataPtr->zone = _zone;
}

//////////////////////////////////////////////////
void RNDFNode::SetWaypoint(rndf::Waypoint *_waypoint)
{
  this->dataPtr->waypoint = _waypoint;
}

//////////////////////////////////////////////////
bool RNDFNode::operator==(const RNDFNode &_other) const
{
  return this->UniqueId() == _other.UniqueId();
}

//////////////////////////////////////////////////
bool RNDFNode::operator!=(const RNDFNode &_other) const
{
  return !(*this == _other);
}

//////////////////////////////////////////////////
RNDFNode &RNDFNode::operator=(const RNDFNode &_other)
{
  this->SetUniqueId(_other.UniqueId());
  this->SetSegment(_other.Segment());
  this->SetLane(_other.Lane());
  this->SetZone(_other.Zone());
  this->SetWaypoint(_other.Waypoint());
  return *this;
}
