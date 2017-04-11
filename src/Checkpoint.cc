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
#include "ignition/rndf/Checkpoint.hh"

using namespace ignition;
using namespace rndf;

namespace ignition
{
  namespace rndf
  {
    /// \internal
    /// \brief Private data for Checkpoint class.
    class CheckpointPrivate
    {
      /// \brief Constructor.
      /// \param[in] _checkpointId Checkpoint Id.
      /// \param[in] _waypointId Waypoint Id.
      public: explicit CheckpointPrivate(const int _checkpointId,
                                         const int _waypointId)
        : checkpointId(_checkpointId),
          waypointId(_waypointId)
      {
      }

      /// \brief Destructor.
      public: virtual ~CheckpointPrivate() = default;

      /// \brief Checkpoint identifier. E.g.: 1
      public: int checkpointId = -1;

      /// \brief Waypoint identifier. E.g.: 1
      public: int waypointId = -1;
    };
  }
}

//////////////////////////////////////////////////
Checkpoint::Checkpoint()
{
  this->dataPtr.reset(new CheckpointPrivate(-1, -1));
}

//////////////////////////////////////////////////
Checkpoint::Checkpoint(const int _checkpointId, const int _waypointId)
  : Checkpoint()
{
  if (_checkpointId <= 0 || _waypointId <= 0)
    return;

  this->SetCheckpointId(_checkpointId);
  this->SetWaypointId(_waypointId);
}

//////////////////////////////////////////////////
Checkpoint::Checkpoint(const Checkpoint &_other)
  : Checkpoint(_other.CheckpointId(), _other.WaypointId())
{
  *this = _other;
}

//////////////////////////////////////////////////
Checkpoint::~Checkpoint()
{
}

//////////////////////////////////////////////////
int Checkpoint::CheckpointId() const
{
  return this->dataPtr->checkpointId;
}

//////////////////////////////////////////////////
bool Checkpoint::SetCheckpointId(const int _id)
{
  bool valid = _id > 0;
  if (valid)
    this->dataPtr->checkpointId = _id;
  return valid;
}

//////////////////////////////////////////////////
int Checkpoint::WaypointId() const
{
  return this->dataPtr->waypointId;
}

//////////////////////////////////////////////////
bool Checkpoint::SetWaypointId(const int _id)
{
  bool valid = _id > 0;
  if (valid)
    this->dataPtr->waypointId = _id;
  return valid;
}

//////////////////////////////////////////////////
bool Checkpoint::Valid() const
{
  return this->CheckpointId() > 0 && this->WaypointId() > 0;
}

//////////////////////////////////////////////////
bool Checkpoint::operator==(const Checkpoint &_other) const
{
  return this->CheckpointId() == _other.CheckpointId();
}

//////////////////////////////////////////////////
bool Checkpoint::operator!=(const Checkpoint &_other) const
{
  return !(*this == _other);
}

//////////////////////////////////////////////////
Checkpoint &Checkpoint::operator=(const Checkpoint &_other)
{
  this->SetCheckpointId(_other.CheckpointId());
  this->SetWaypointId(_other.WaypointId());
  return *this;
}
