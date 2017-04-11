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

#ifndef IGNITION_RNDF_LANE_HH_
#define IGNITION_RNDF_LANE_HH_

#include <iosfwd>
#include <memory>
#include <vector>

#include "ignition/rndf/Helpers.hh"

namespace ignition
{
  namespace rndf
  {
    // Forward declarations.
    class Checkpoint;
    class Exit;
    class LaneHeaderPrivate;
    class LanePrivate;
    class Waypoint;

    /// \def Scope Different options for the lane boundaries.
    enum class Marking
    {
      /// \brief Double yellow type.
      DOUBLE_YELLOW,
      /// \brief Solid yellow type.
      SOLID_YELLOW,
      /// \brief Solid white type.
      SOLID_WHITE,
      /// \brief Broken white,
      BROKEN_WHITE,
      /// \brief Undefined.
      UNDEFINED,
    };

    /// \internal
    /// \brief An internal private lane header class.
    class LaneHeader
    {
      /// \brief Default constructor.
      public: LaneHeader();

      /// \brief Destructor.
      public: ~LaneHeader() = default;

      ///////////
      /// Parsing
      ///////////

      /// \brief Load a lane header from an input stream coming from a
      /// text file. The expected format is the one specified on the RNDF spec.
      /// \param[in, out] _rndfFile Input file stream.
      /// \param[in] _segmentId The expected zone Id.
      /// \param[in] _laneId The expected lane Id.
      /// \param[in, out] _lineNumber Line number pointed by the stream position
      /// indicator.
      /// \return True if a lane header block was found and parsed or
      /// false otherwise (e.g.: EoF or incorrect format found).
      public: bool Load(std::ifstream &_rndfFile,
                        const int _segmentId,
                        const int _laneId,
                        int &_lineNumber);

      /////////
      /// Width
      /////////

      /// \brief Get the lane width in meters.
      /// \return Return the lane width in meters.
      public: double Width() const;

      /// \brief Set the lane width.
      /// \param[in] _newWidth The new width in meters.
      public: bool SetWidth(const double _newWidth);

      //////////////
      /// Boundaries
      //////////////

      /// \brief Get the left boundary type.
      /// \return The left boundary type.
      public: Marking LeftBoundary() const;

      /// \brief Set the new left boundary type.
      /// \param[in] _boundary The new left boundary type.
      public: void SetLeftBoundary(const Marking &_boundary);

      /// \brief Get the right boundary type.
      /// \return The right boundary type.
      public: Marking RightBoundary() const;

      /// \brief Set the new right boundary type.
      /// \param[in] _boundary The new right boundary type.
      public: void SetRightBoundary(const Marking &_boundary);

      ///////////////
      /// Checkpoints
      ///////////////

      /// \brief Get the number of checkpoints stored.
      /// \return The number of checkpoints in the current lane.
      public: unsigned int NumCheckpoints() const;

      /// \brief Get a mutable reference to the vector of checkpoints;
      /// \return A mutable reference to the vector of checkpoints.
      public: std::vector<rndf::Checkpoint> &Checkpoints();

      /// \brief Get the vector of checkpoints;
      /// \return The vector of checkpoints.
      public: const std::vector<rndf::Checkpoint> &Checkpoints() const;

      /// \brief Get the details of one of the checkpoints with Id _cpId.
      /// \param[in] _cpId The checkpoint Id.
      /// \param[out] _cp The checkpoint requested.
      /// \return True if the checkpoint was found or false otherwise.
      public: bool Checkpoint(const int _cpId,
                              rndf::Checkpoint &_cp) const;

      /// \brief Update an existing checkpoint.
      /// \param[in] _cp The updated checkpoint.
      /// \return True if the checkpoint was found and updated or false
      /// otherwise.
      public: bool UpdateCheckpoint(const rndf::Checkpoint &_cp);

      /// \brief Add a new checkpoint.
      /// \param[in] _newCheckpoint A new checkpoint to be added.
      /// \return True when the checkpoint was successfully added to the list or
      /// false otherwise (e.g. if the Id of the checkpoint was already existing
      /// or invalid).
      public: bool AddCheckpoint(const rndf::Checkpoint &_newCheckpoint);

      /// \brief Remove an existing checkpoint.
      /// \param[in] _cpId The checkpoint Id to be removed.
      /// \return True when the checkpoint was successfully deleted
      /// or false otherwise (e.g. if the Id of the checkpoint was not found
      /// or invalid).
      public: bool RemoveCheckpoint(const int _cpId);

      /////////
      /// Stops
      /////////

      /// \brief Get the number of stops stored.
      /// \return The number of stops in the current lane.
      public: unsigned int NumStops() const;

      /// \brief Get a mutable reference to the vector of stops. The elements
      /// are waypoint Ids.
      /// \return A mutable reference to the vector of stops.
      public: std::vector<int> &Stops();

      /// \brief Get the vector of stops. The elements are waypoint Ids.
      /// \return The vector of stops.
      public: const std::vector<int> &Stops() const;

      /// \brief Add a new stop.
      /// \param[in] _waypointId The Id of a new waypoint considered a stop.
      /// \return True when the stop was successfully added to the list or
      /// false otherwise (e.g. if the Id of the waypoint was already existing
      /// or invalid).
      public: bool AddStop(const int _waypointId);

      /// \brief Remove an existing stop.
      /// \param[in] _waypointId The waypoint Id (of a stop sign) to be removed.
      /// \return True when the waypoint was successfully deleted
      /// or false otherwise (e.g. if the Id of the waypoint was not found
      /// or invalid).
      public: bool RemoveStop(const int _waypointId);

      /////////
      /// Exits
      /////////

      /// \brief Get the number of exits stored.
      /// \return The number of exits in the current lane.
      public: unsigned int NumExits() const;

      /// \brief Get a mutable reference to the vector of exits.
      /// \return A mutable reference to the vector of exits.
      public: std::vector<Exit> &Exits();

      /// \brief Get the vector of stops.
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

      /// \brief Smart pointer to private data.
      private: std::unique_ptr<LaneHeaderPrivate> dataPtr;
    };

    /// \brief A class that represents a road lane composed by a set of
    /// waypoints.
    class IGNITION_RNDF_VISIBLE Lane
    {
      /// \brief Default constructor.
      /// \sa Valid.
      public: Lane();

      /// \brief Constructor.
      /// \param[in] _id Lane Id (a positive number).
      /// \sa Valid.
      public: explicit Lane(const int _id);

      /// \brief Copy constructor.
      /// \param[in] _other Other lane.
      /// \sa Valid.
      public: explicit Lane(const Lane &_other);

      /// \brief Destructor.
      public: virtual ~Lane();

      ///////////
      /// Parsing
      ///////////

      /// \brief Load a lane from an input stream coming from a text file.
      /// The expected format is the one specified on the RNDF spec.
      /// \param[in, out] _rndfFile Input file stream.
      /// \param[in, out] _segmentId Expected segment Id.
      /// \param[in, out] _lineNumber Line number pointed by the stream position
      /// indicator.
      /// \return True if a lane block was found and parsed or
      /// false otherwise (e.g.: EoF or incorrect format found).
      public: bool Load(std::ifstream &_rndfFile,
                        const int _segmentId,
                        int &_lineNumber);

      ///////
      /// Id
      ///////

      /// \brief Get the unique identifier of the lane.
      /// \return The lane Id.
      public: int Id() const;

      /// \brief Set the identifier of the lane.
      /// \param[in] _id New unique Id.
      /// \return True if the operation succeed or false otherwise
      /// (e.g.: if the id is not valid).
      /// \sa Valid.
      public: bool SetId(const int _id);

      /////////////
      /// Waypoints
      /////////////

      /// \brief Get the number of waypoints stored.
      /// \return The number of waypoints in the current lane.
      public: unsigned int NumWaypoints() const;

      /// \brief Get a mutable reference to the vector of waypoints.
      /// \return A mutable reference to the vector of waypoints.
      public: std::vector<rndf::Waypoint> &Waypoints();

      /// \brief Get the vector of waypoints.
      /// \return \return The vector of waypoints.
      public: const std::vector<rndf::Waypoint> &Waypoints() const;

      /// \brief Get the details of one of the waypoints with Id _wpId.
      /// \param[in] _wpId The waypoint Id.
      /// \param[out] _wp The waypoint requested.
      /// \return True if the waypoint was found or false otherwise.
      public: bool Waypoint(const int _wpId,
                            rndf::Waypoint &_wp) const;

      /// \brief Update an existing waypoint.
      /// \param[in] _wp The updated waypoint.
      /// \return True if the waypoint was found and updated or false otherwise.
      public: bool UpdateWaypoint(const rndf::Waypoint &_wp);

      /// \brief Add a new waypoint.
      /// \param[in] _newWaypoint A new waypoint to be added.
      /// \return True when the waypoint was successfully added to the list or
      /// false otherwise (e.g. if the Id of the waypoint was already existing
      /// or invalid).
      public: bool AddWaypoint(const rndf::Waypoint &_newWaypoint);

      /// \brief Remove an existing waypoint.
      /// \param[in] _wpId The waypoint Id to be removed.
      /// \return True when the waypoint was successfully deleted from the list
      /// or false otherwise (e.g. if the Id of the waypoint was not found
      /// or invalid).
      public: bool RemoveWaypoint(const int _wpId);

      /////////
      /// Width
      /////////

      /// \brief Get the lane width in meters.
      /// \return Return the lane width in meters.
      public: double Width() const;

      /// \brief Set the lane width.
      /// \param[in] _newWidth The new width in meters.
      public: bool SetWidth(const double _newWidth);

      //////////////
      /// Boundaries
      //////////////

      /// \brief Get the left boundary type.
      /// \return The left boundary type.
      public: Marking LeftBoundary() const;

      /// \brief Set the new left boundary type.
      /// \param[in] _boundary The new left boundary type.
      public: void SetLeftBoundary(const Marking &_boundary);

      /// \brief Get the right boundary type.
      /// \return The right boundary type.
      public: Marking RightBoundary() const;

      /// \brief Set the new right boundary type.
      /// \param[in] _boundary The new right boundary type.
      public: void SetRightBoundary(const Marking &_boundary);

      ///////////////
      /// Checkpoints
      ///////////////

      /// \brief Get the number of checkpoints stored.
      /// \return The number of checkpoints in the current lane.
      public: unsigned int NumCheckpoints() const;

      /// \brief Get a mutable reference to the vector of checkpoints;
      /// \return A mutable reference to the vector of checkpoints.
      public: std::vector<rndf::Checkpoint> &Checkpoints();

      /// \brief Get the vector of checkpoints;
      /// \return The vector of checkpoints.
      public: const std::vector<rndf::Checkpoint> &Checkpoints() const;

      /// \brief Get the details of one of the checkpoints with Id _cpId.
      /// \param[in] _cpId The checkpoint Id.
      /// \param[out] _cp The checkpoint requested.
      /// \return True if the checkpoint was found or false otherwise.
      public: bool Checkpoint(const int _cpId, rndf::Checkpoint &_cp) const;

      /// \brief Update an existing checkpoint.
      /// \param[in] _cp The updated checkpoint.
      /// \return True if the checkpoint was found and updated or false
      /// otherwise.
      public: bool UpdateCheckpoint(const rndf::Checkpoint &_cp);

      /// \brief Add a new checkpoint.
      /// \param[in] _newCheckpoint A new checkpoint to be added.
      /// \return True when the checkpoint was successfully added to the list or
      /// false otherwise (e.g. if the Id of the checkpoint was already existing
      /// or invalid).
      public: bool AddCheckpoint(const rndf::Checkpoint &_newCheckpoint);

      /// \brief Remove an existing checkpoint.
      /// \param[in] _cpId The checkpoint Id to be removed.
      /// \return True when the checkpoint was successfully deleted
      /// or false otherwise (e.g. if the Id of the checkpoint was not found
      /// or invalid).
      public: bool RemoveCheckpoint(const int _cpId);

      /////////
      /// Stops
      /////////

      /// \brief Get the number of stops stored.
      /// \return The number of stops in the current lane.
      public: unsigned int NumStops() const;

      /// \brief Get a mutable reference to the vector of stops. The elements
      /// are waypoint Ids.
      /// \return A mutable reference to the vector of stops.
      public: std::vector<int> &Stops();

      /// \brief Get the vector of stops. The elements are waypoint Ids.
      /// \return The vector of stops.
      public: const std::vector<int> &Stops() const;

      /// \brief Add a new stop.
      /// \param[in] _waypointId The Id of a new waypoint considered a stop.
      /// \return True when the stop was successfully added to the list or
      /// false otherwise (e.g. if the Id of the waypoint was already existing
      /// or invalid).
      public: bool AddStop(const int _waypointId);

      /// \brief Remove an existing stop.
      /// \param[in] _waypointId The waypoint Id (of a stop sign) to be removed.
      /// \return True when the waypoint was successfully deleted
      /// or false otherwise (e.g. if the Id of the waypoint was not found
      /// or invalid).
      public: bool RemoveStop(const int _waypointId);

      /////////
      /// Exits
      /////////

      /// \brief Get the number of exits stored.
      /// \return The number of exits in the current lane.
      public: unsigned int NumExits() const;

      /// \brief Get a mutable reference to the vector of exits.
      /// \return A mutable reference to the vector of exits.
      public: std::vector<Exit> &Exits();

      /// \brief Get the vector of stops.
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

      /// \return True if the lane is valid.
      public: bool Valid() const;

      /////////////
      /// Operators
      /////////////

      /// \brief Equality operator, result = this == _other
      /// \param[in] _other Lane to check for equality.
      /// \return true if this == _other
      public: bool operator==(const Lane &_other) const;

      /// \brief Inequality.
      /// \param[in] _other Lane to check for inequality.
      /// \return true if this != _other
      public: bool operator!=(const Lane &_other) const;

      /// \brief Assignment operator.
      /// \param[in] _other The new lane.
      /// \return A reference to this instance.
      public: Lane &operator=(const Lane &_other);

      /// \internal
      /// \brief Smart pointer to private data.
      private: std::unique_ptr<LanePrivate> dataPtr;
    };
  }
}
#endif
