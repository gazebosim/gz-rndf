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

#ifndef IGNITION_RNDF_PARKINGSPOT_HH_
#define IGNITION_RNDF_PARKINGSPOT_HH_

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
    class ParkingSpotPrivate;
    class ParkingSpotHeaderPrivate;
    class Waypoint;

    /// \internal
    /// \brief An internal private spot header class.
    class ParkingSpotHeader
    {
      /// \brief Default constructor.
      public: ParkingSpotHeader();

      /// \brief Destructor.
      public: virtual ~ParkingSpotHeader();

      ///////////
      /// Parsing
      ///////////

      /// \brief Load a parking spot header from an input stream coming from a
      /// text file. The expected format is the one specified on the RNDF spec.
      /// \param[in, out] _rndfFile Input file stream.
      /// \param[in] _zoneId The zone Id in which the spot is located.
      /// \param[in] _spotId The spot Id.
      /// \param[in, out] _lineNumber Line number pointed by the stream position
      /// indicator.
      /// \return True if a parking spot header block was found and parsed or
      /// false otherwise (e.g.: EoF or incorrect format found).
      public: bool Load(std::ifstream &_rndfFile,
                        const int _zoneId,
                        const int _spotId,
                        int &_lineNumber);

      /////////
      /// Width
      /////////

      /// \brief Get the parking spot width in meters.
      /// \return Return the parking spot width in meters.
      public: double Width() const;

      /// \brief Set the parking spot width.
      /// \param[in] _newWidth The new width in meters.
      /// \return True if the new width was set or false otherwise
      /// (invalid value).
      public: bool SetWidth(const double _newWidth);

      //////////////
      /// Checkpoint
      //////////////

      /// \brief Get a mutable reference to the checkpoint.
      /// \return A mutable reference to the checkpoint.
      public: rndf::Checkpoint &Checkpoint();

      /// \brief Get the checkpoint.
      /// \return The checkpoint.
      public: const rndf::Checkpoint &Checkpoint() const;

      /// \brief Smart pointer to private data.
      private: std::unique_ptr<ParkingSpotHeaderPrivate> dataPtr;
    };

    /// \brief An abstraction for representing a parking spot within a zone.
    class IGNITION_RNDF_VISIBLE ParkingSpot
    {
      /// \brief Default constructor.
      public: ParkingSpot();

      /// \brief Constructor.
      /// \param[in] _spotId Parking spot Id (a positive number).
      public: explicit ParkingSpot(const int _spotId);

      /// \brief Copy constructor.
      /// \param[in] _other Other parking spot.
      public: ParkingSpot(const ParkingSpot &_other);

      /// \brief Destructor.
      public: virtual ~ParkingSpot();

      ///////////
      /// Parsing
      ///////////

      /// \brief Load a parking spot from an input stream coming from a text
      /// file. The expected format is the one specified on the RNDF spec.
      /// \param[in, out] _rndfFile Input file stream.
      /// \param[in] _zoneId The zone Id in which the spot is located.
      /// \param[in, out] _lineNumber Line number pointed by the stream position
      /// indicator.
      /// \return True if a parking spot block was found and parsed or false
      /// otherwise (e.g.: EoF or incorrect format found).
      public: bool Load(std::ifstream &_rndfFile,
                        const int _zoneId,
                        int &_lineNumber);

      ///////
      /// Id
      ///////

      /// \brief Get the Id.
      /// \return The parking spot Id.
      public: int Id() const;

      /// \brief Set the identifier of the parking spot.
      /// \param[in] _id New unique Id.
      /// \return True if the operation succeed or false otherwise
      /// (e.g.: if the id is not valid).
      /// \sa Valid.
      public: bool SetId(const int _id);

      /////////////
      /// Waypoints
      /////////////

      /// \brief Get the number of waypoints stored.
      /// \return The number of waypoints in the current parking spot.
      public: size_t NumWaypoints() const;

      /// \brief Get a mutable reference to the vector of waypoints;
      /// \return A mutable reference to the vector of waypoints.
      public: std::vector<rndf::Waypoint> &Waypoints();

      /// \brief Get the vector of waypoints;
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

      /// \brief Get the parking spot width in meters.
      /// \return Return the parking spot width in meters.
      public: double Width() const;

      /// \brief Set the parking spot width.
      /// \param[in] _newWidth The new width in meters.
      /// \return True if the new width was set of false otherwise (invalid
      /// value).
      public: bool SetWidth(const double _newWidth);

      //////////////
      /// Checkpoint
      //////////////

      /// \brief Get a mutable reference to the checkpoint.
      /// \return A mutable reference to the checkpoint.
      public: rndf::Checkpoint &Checkpoint();

      /// \brief Get the checkpoint.
      /// \return The checkpoint.
      public: const rndf::Checkpoint &Checkpoint() const;

      //////////////
      /// Validation
      //////////////

      /// \return True if the parking spot is valid.
      public: bool Valid() const;

      /////////////
      /// Operators
      /////////////

      /// \brief Equality operator, result = this == _other
      /// \param[in] _other Parking spot to check for equality.
      /// \return true if this == _other
      public: bool operator==(const ParkingSpot &_other) const;

      /// \brief Inequality.
      /// \param[in] _other Parking spot to check for inequality.
      /// \return true if this != _other
      public: bool operator!=(const ParkingSpot &_other) const;

      /// \brief Assignment operator.
      /// \param[in] _other The new parking spot.
      /// \return A reference to this instance.
      public: ParkingSpot &operator=(const ParkingSpot &_other);

      /// \internal
      /// \brief Smart pointer to private data.
      private: std::unique_ptr<ParkingSpotPrivate> dataPtr;
    };
  }
}
#endif
