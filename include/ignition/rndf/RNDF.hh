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

#ifndef IGNITION_RNDF_RNDF_HH_
#define IGNITION_RNDF_RNDF_HH_

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
    class RNDFHeaderPrivate;
    class RNDFNode;
    class RNDFPrivate;
    class Segment;
    class UniqueId;
    class Zone;

    // \internal
    /// \brief An internal private RNDF header class.
    class RNDFHeader
    {
      /// \brief Default constructor.
      public: RNDFHeader();

      /// \brief Destructor.
      public: ~RNDFHeader() = default;

      ///////////
      /// Parsing
      ///////////

      /// \brief Load a RNDF header from an input stream coming from a
      /// text file. The expected format is the one specified on the RNDF spec.
      /// \param[in, out] _rndfFile Input file stream.
      /// \param[in, out] _lineNumber Line number pointed by the stream position
      /// indicator.
      /// \return True if a RNDF header block was found and parsed or
      /// false otherwise (e.g.: EoF or incorrect format found).
      public: bool Load(std::ifstream &_rndfFile,
                        int &_lineNumber);

      ///////////
      /// Version
      ///////////

      /// \brief Get the format version. E.g.: "2.3.6".
      /// \return The format version.
      public: std::string Version() const;

      /// \brief Set the format version.
      /// \param[in] _version The new version.
      public: void SetVersion(const std::string &_version) const;

      ////////
      /// Date
      ////////

      /// \brief Get the creation date.
      /// \return The creation date.
      public: std::string Date() const;

      /// \brief Set the creation date.
      /// \param[in] _newDate The new creation date.
      public: void SetDate(const std::string &_newDate) const;

      /// \brief Smart pointer to private data.
      private: std::unique_ptr<RNDFHeaderPrivate> dataPtr;
    };

    /// \brief An abstraction to represent a Route Network Definition File
    /// (RNDF). Please, refer to the specification for more details.
    /// \reference http://www.grandchallenge.org/grandchallenge/docs/RNDF_MDF_Formats_031407.pdf
    class IGNITION_RNDF_VISIBLE RNDF
    {
      /// \brief Default constructor.
      public: RNDF();

      /// \brief Constructor.
      /// \param[in] _filepath Path to an existing RNDF file.
      public: explicit RNDF(const std::string &_filepath);

      /// \brief Destructor.
      public: virtual ~RNDF();

      ///////////
      /// Parsing
      ///////////

      /// \brief Load a RNDF from an input stream coming from a text file.
      /// The expected format is the one specified on the RNDF spec.
      /// \param[in, out] _filePath Path to RNDF file.
      /// \return True if the entire RNDF was correctly parsed or false
      /// otherwise (e.g.: EoF or incorrect format found).
      public: bool Load(const std::string &_filePath);

      ////////
      /// Name
      ////////

      /// \brief Get the segment name.
      /// \return The segment name.
      public: std::string Name() const;

      /// \brief Set the segment name.
      /// \param[in] _name The new name.
      public: void SetName(const std::string &_name);

      ////////////
      /// Segments
      ////////////

      /// \brief Get the number of segments stored.
      /// \return The number of segments in this RNDF.
      public: unsigned int NumSegments() const;

      /// \brief Get a mutable reference to the vector of segments.
      /// \return A mutable reference to the vector of segments.
      public: std::vector<rndf::Segment> &Segments();

      /// \brief Get the vector of segments.
      /// \return \return The vector of segments.
      public: const std::vector<rndf::Segment> &Segments() const;

      /// \brief Get the details of one of the segments with Id _segmentId.
      /// \param[in] _segmentId The segment Id.
      /// \param[out] _segment The segment requested.
      /// \return True if the segment was found or false otherwise.
      public: bool Segment(const int _segmentId,
                           rndf::Segment &_segment) const;

      /// \brief Update an existing segment.
      /// \param[in] _segment The updated segment.
      /// \return True if the segment was found and updated or false otherwise.
      public: bool UpdateSegment(const rndf::Segment &_segment);

      /// \brief Add a new segment.
      /// \param[in] _newSegment A new segment to be added.
      /// \return True when the segment was successfully added to the list or
      /// false otherwise (e.g. if the Id of the segment was already existing
      /// or invalid).
      public: bool AddSegment(const rndf::Segment &_newSegment);

      /// \brief Remove an existing segment.
      /// \param[in] _segmentId The segment Id to be removed.
      /// \return True when the segment was successfully deleted from the list
      /// or false otherwise (e.g. if the Id of the segment was not found
      /// or invalid).
      public: bool RemoveSegment(const int _segmentId);

      /////////
      /// Zones
      /////////

      /// \brief Get the number of zones stored.
      /// \return The number of zones in this RNDF.
      public: unsigned int NumZones() const;

      /// \brief Get a mutable reference to the vector of zones.
      /// \return A mutable reference to the vector of zones.
      public: std::vector<rndf::Zone> &Zones();

      /// \brief Get the vector of zones.
      /// \return \return The vector of zones.
      public: const std::vector<rndf::Zone> &Zones() const;

      /// \brief Get the details of one of the zones with Id _zoneId.
      /// \param[in] _zoneId The zone Id.
      /// \param[out] _zone The zone requested.
      /// \return True if the zone was found or false otherwise.
      public: bool Zone(const int _zoneId,
                        rndf::Zone &_zone) const;

      /// \brief Update an existing zone.
      /// \param[in] _zone The updated zone.
      /// \return True if the zone was found and updated or false otherwise.
      public: bool UpdateZone(const rndf::Zone &_zone);

      /// \brief Add a new zone.
      /// \param[in] _newZone A new zone to be added.
      /// \return True when the zone was successfully added to the list or
      /// false otherwise (e.g. if the Id of the zone was already existing
      /// or invalid).
      public: bool AddZone(const rndf::Zone &_newZone);

      /// \brief Remove an existing zone.
      /// \param[in] _zoneId The zone Id to be removed.
      /// \return True when the zone was successfully deleted from the list
      /// or false otherwise (e.g. if the Id of the zone was not found
      /// or invalid).
      public: bool RemoveZone(const int _zoneId);

      ///////////
      /// Version
      ///////////

      /// \brief Get the format version. E.g.: "2.3.6".
      /// \return The format version.
      public: std::string Version() const;

      /// \brief Set the format version.
      /// \param[in] _version The new version.
      public: void SetVersion(const std::string &_version) const;

      ////////
      /// Date
      ////////

      /// \brief Get the creation date.
      /// \return The creation date.
      public: std::string Date() const;

      /// \brief Set the creation date.
      /// \param[in] _newDate The new creation date.
      public: void SetDate(const std::string &_newDate) const;

      //////////////
      /// Validation
      //////////////

      /// \brief Whether the current RNDF object is valid or not.
      /// \return True if the RNDF is valid.
      public: bool Valid() const;

      /////////
      /// Utils
      /////////

      /// \brief Get a pointer to the associated RNDF node given a unique Id.
      /// The RNDFNode object contains the metada associated to the id.
      /// \param[in] _id The Unique Id to check.
      /// \return A pointer to the RNDFnode.
      public: RNDFNode *Info(const rndf::UniqueId &_id) const;

      /// \brief Populates the "cache" member variable linking all unique Ids
      /// with their metadata (RNDFNode).
      private: void UpdateCache();

      /// \internal
      /// \brief Smart pointer to private data.
      private: std::unique_ptr<RNDFPrivate> dataPtr;
    };
  }
}
#endif
