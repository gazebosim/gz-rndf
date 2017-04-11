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

#ifndef IGNITION_RNDF_UNIQUEID_HH_
#define IGNITION_RNDF_UNIQUEID_HH_

#include <iostream>
#include <string>
#include "ignition/rndf/Helpers.hh"

namespace ignition
{
  namespace rndf
  {
    /// \brief A unique id of the form x.y.z, where x and z are positive
    /// numbers and y is a non-negative number (perimeter Ids are always 0).
    /// The maximum allowed value is 32768.
    class IGNITION_RNDF_VISIBLE UniqueId
    {
      /// \brief Default constructor.
      public: UniqueId();

      /// \brief Constructor.
      /// \param[in] _x A positive number.
      /// \param[in] _y A non-negative number.
      /// \param[in] _z A positive number.
      public: explicit UniqueId(const int _x,
                                const int _y,
                                const int _z);

      /// \brief Constructor.
      /// \param[in] _id With format x.y.z
      public: explicit UniqueId(const std::string &_id);

      /// \brief Copy constructor.
      /// \param[in] _other Other UniqueId.
      public: UniqueId(const UniqueId &_other);

      /// \brief Destructor.
      public: virtual ~UniqueId();

      /// \brief Get 'x' value.
      /// \return The 'x' value.
      public: int X() const;

      /// \brief Set the 'x' value.
      /// \param[in] _x New 'x' value.
      /// \return True if the operation succeed or false otherwise
      /// (e.g.: if the 'x' is not valid).
      /// \sa Valid.
      public: bool SetX(const int _x);

      /// \brief Get the 'y' value.
      /// \return The 'y' value.
      public: int Y() const;

      /// \brief Set the 'y' value.
      /// \param[in] _y New 'y' value.
      /// \return True if the operation succeed or false otherwise
      /// (e.g.: if the 'y' is not valid).
      /// \sa Valid.
      public: bool SetY(const int _y);

      /// \brief Get the 'z' value.
      /// \return The 'z' value.
      public: int Z() const;

      /// \brief Set the 'z' value.
      /// \param[in] _z New 'z' value.
      /// \return True if the operation succeed or false otherwise
      /// (e.g.: if the 'z' is not valid).
      /// \sa Valid.
      public: bool SetZ(const int _z);

      /// \brief Whether the object is valid or not.
      /// \return True if the unique Id is valid.
      public: bool Valid() const;

      /// \brief Convert to string.
      /// \return A string representation of the unique Id.
      public: std::string String() const;

      /// \brief Equality operator, result = this == _other
      /// \param[in] _other UniqueId to check for equality.
      /// \return true if this == _other
      public: bool operator==(const UniqueId &_other) const;

      /// \brief Inequality
      /// \param[in] _other UniqueId to check for inequality.
      /// \return true if this != _other
      public: bool operator!=(const UniqueId &_other) const;

      /// \brief Assignment operator.
      /// \param[in] _other The new UniqueId.
      /// \return A reference to this instance.
      public: UniqueId &operator=(const UniqueId &_other);

      /// \brief Stream insertion operator.
      /// \param[out] _out The output stream.
      /// \param[in] _id UniqueId to write to the stream.
      public: friend std::ostream &operator<<(std::ostream &_out,
                                              const UniqueId &_id)
      {
        _out << _id.X() << "." << _id.Y() << "." << _id.Z();
        return _out;
      }

      /// \brief The 'x' value.
      private: int x;

      /// \brief The 'y' value.
      private: int y;

      /// \brief The 'z' value.
      private: int z;
    };
  }
}
#endif
