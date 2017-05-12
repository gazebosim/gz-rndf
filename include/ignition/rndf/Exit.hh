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

#ifndef IGNITION_RNDF_EXIT_HH_
#define IGNITION_RNDF_EXIT_HH_

#include <iosfwd>
#include <string>

#include "ignition/rndf/Helpers.hh"
#include "ignition/rndf/UniqueId.hh"

namespace ignition
{
  namespace rndf
  {
    /// \brief An exit clas that shows how to go from an exit waypoint to
    /// an entry waypoint. The waypoints are represented with their unique Id.
    class IGNITION_RNDF_VISIBLE Exit
    {
      /// \brief Default constructor.
      public: Exit() = default;

      /// \brief Constructor.
      /// \param[in] _exit The unique ID of the exit waypoint.
      /// \param[in] _entry The unique ID of the entry waypoint.
      /// \sa Valid.
      public: explicit Exit(const UniqueId &_exit,
                            const UniqueId &_entry);

      /// \brief Copy constructor.
      /// \param[in] _other Other Exit.
      public: Exit(const Exit &_other);

      /// \brief Destructor.
      public: virtual ~Exit();

      ///////////
      /// Parsing
      ///////////

      /// \brief Load an exit from an input stream coming from a text file.
      /// The expected format is the one specified on the RNDF spec.
      /// \param[in, out] _rndfFile Input file stream.
      /// \param[in] _x The expected "x" value from an x.y.z Id.
      /// \param[in] _y The expected "y" value from an x.y.z Id.
      /// \param[in, out] _lineNumber Line number pointed by the stream position
      /// indicator.
      /// \param[out] _lineRead Entire text line used to parse the exit.
      /// \return True if a zone block was found and parsed or
      /// false otherwise (e.g.: EoF or incorrect format found).
      public: bool Load(std::ifstream &_rndfFile,
                        const int _x,
                        const int _y,
                        int &_lineNumber,
                        std::string &_lineRead);

      //////////
      /// ExitId
      //////////

      /// \brief Get the unique Id of the exit waypoint.
      /// \return The unique Id of the exit waypoint.
      public: const UniqueId &ExitId() const;

      /// \brief Get a mutable reference to the unique Id of the exit waypoint.
      /// \return A mutable reference to the unique Id of the exit waypoint.
      public: UniqueId &ExitId();

      ///////////
      /// EntryId
      ///////////

      /// \brief Get the unique Id of the entry waypoint.
      /// \return The unique Id of the entry waypoint.
      public: const UniqueId &EntryId() const;

      /// \brief Get a mutable reference to the unique Id of the entry waypoint.
      /// \return A mutable reference to the unique Id of the entry waypoint.
      public: UniqueId &EntryId();

      //////////////
      /// Validation
      //////////////

      /// \return True if the exit is valid.
      public: bool Valid() const;

      /////////////
      /// Operators
      /////////////

      /// \brief Equality operator, result = this == _other
      /// \param[in] _other Exit to check for equality.
      /// \return true if this == _other
      public: bool operator==(const Exit &_other) const;

      /// \brief Inequality.
      /// \param[in] _other Exit to check for inequality.
      /// \return true if this != _other
      public: bool operator!=(const Exit &_other) const;

      /// \brief Assignment operator.
      /// \param[in] _other The new Exit.
      /// \return A reference to this instance.
      public: Exit &operator=(const Exit &_other);

      /// \brief The unique Id of the exit waypoint.
      private: UniqueId exit;

      /// \brief The unique Id of the entry waypoint.
      private: UniqueId entry;
    };
  }
}
#endif
