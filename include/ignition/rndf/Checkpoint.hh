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

#ifndef IGNITION_RNDF_CHECKPOINT_HH_
#define IGNITION_RNDF_CHECKPOINT_HH_

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
    class CheckpointPrivate;

    /// \brief A checkpoint is a waypoint that has to be visited.
    /// It also has its own Id.
    class IGNITION_RNDF_VISIBLE Checkpoint
    {
      /// \brief Default constructor.
      public: Checkpoint();

      /// \brief Constructor.
      /// \param[in] _checkpointId Checkpoint Id (a positive number).
      /// \param[in] _waypointId The waypoint Id associated to the checkpoint.
      /// \sa Valid.
      public: explicit Checkpoint(const int _checkpointId,
                                  const int _waypointId);

      /// \brief Copy constructor.
      /// \param[in] _other Other checkpoint.
      public: Checkpoint(const Checkpoint &_other);

      /// \brief Destructor.
      public: virtual ~Checkpoint();

      /// \brief Get the checkpoint Id.
      /// \return The checkpoint Id.
      public: int CheckpointId() const;

      /// \brief Set the identifier of the checkpoint.
      /// \param[in] _id New unique Id.
      /// \return True if the operation succeed or false otherwise (e.g.: if the
      /// id is not valid).
      /// \sa Valid.
      public: bool SetCheckpointId(const int _id);

      /// \brief Get the waypoint Id.
      /// \return The waypoint Id.
      public: int WaypointId() const;

      /// \brief Set the identifier of the waypoint.
      /// \param[in] _id New unique Id.
      /// \return True if the operation succeed or false otherwise
      /// (e.g.: if the id is not valid).
      /// \sa Valid.
      public: bool SetWaypointId(const int _id);

      /// \return True if the checkpoint is valid.
      public: bool Valid() const;

      /// \brief Equality operator, result = this == _other
      /// \param[in] _other Checkpoint to check for equality
      /// \return true if this == _other
      public: bool operator==(const Checkpoint &_other) const;

      /// \brief Inequality
      /// \param[in] _other Checkpoint to check for inequality
      /// \return true if this != _other
      public: bool operator!=(const Checkpoint &_other) const;

      /// \brief Assignment operator.
      /// \param[in] _other The new checkpoint.
      /// \return A reference to this instance.
      public: Checkpoint &operator=(const Checkpoint &_other);

      /// \internal
      /// \brief Smart pointer to private data.
      private: std::unique_ptr<CheckpointPrivate> dataPtr;
    };
  }
}

#ifdef _WIN32
#pragma warning(pop)
#endif

#endif
