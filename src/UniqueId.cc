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

#include <array>
#include <iostream>
#include <sstream>
#include <string>

#include "ignition/rndf/ParserUtils.hh"
#include "ignition/rndf/UniqueId.hh"

using namespace ignition;
using namespace rndf;

//////////////////////////////////////////////////
UniqueId::UniqueId()
  : x(-1),
    y(-1),
    z(-1)
{
}

//////////////////////////////////////////////////
UniqueId::UniqueId(const int _x, const int _y, const int _z)
  : UniqueId()
{
  if ((_x <= 0) || (_y < 0) || (_z <= 0))
    return;

  this->SetX(_x);
  this->SetY(_y);
  this->SetZ(_z);
}

//////////////////////////////////////////////////
UniqueId::UniqueId(const std::string &_id)
  : UniqueId()
{
  auto tokens = split(_id, ".");
  if (tokens.size() != 3)
  {
    std::cerr << "Unable to parse uniqueId [" << _id << "]" << std::endl;
    return;
  }

  std::array<std::string::size_type, 3> sz;
  std::array<int, 3> data;
  const std::array<int, 3> kMin = {1, 0, 1};
  try
  {
    for (int i = 0; i < 3; ++i)
      data[i] = std::stod(tokens[i], &(sz[i]));
  } catch (...)
  {
    std::cerr << "Unable to parse uniqueId [" << _id << "]" << std::endl;
    return;
  }

  // Sanity check.
  for (int i = 0; i < 3; ++i)
  {
    if (data[i] < kMin[i]             ||
        data[i] > 32768               ||
        sz[i] != tokens.at(i).size())
    {
      std::cerr << "Unable to parse uniqueId [" << _id << "]" << std::endl;
      return;
    }
  }

  this->SetX(data.at(0));
  this->SetY(data.at(1));
  this->SetZ(data.at(2));
}

//////////////////////////////////////////////////
UniqueId::UniqueId(const UniqueId &_other)
  : UniqueId(_other.X(), _other.Y(), _other.Z())
{
}

//////////////////////////////////////////////////
UniqueId::~UniqueId()
{
}

//////////////////////////////////////////////////
int UniqueId::X() const
{
  return this->x;
}

//////////////////////////////////////////////////
bool UniqueId::SetX(const int _x)
{
  bool valid = _x > 0;
  if (valid)
    this->x = _x;
  return valid;
}

//////////////////////////////////////////////////
int UniqueId::Y() const
{
  return this->y;
}

//////////////////////////////////////////////////
bool UniqueId::SetY(const int _y)
{
  // We allow 0 here because a perimeter Id is always 0.
  bool valid = _y >= 0;
  if (valid)
    this->y = _y;
  return valid;
}

//////////////////////////////////////////////////
int UniqueId::Z() const
{
  return this->z;
}

//////////////////////////////////////////////////
bool UniqueId::SetZ(const int _z)
{
  bool valid = _z > 0;
  if (valid)
    this->z = _z;
  return valid;
}

//////////////////////////////////////////////////
std::string UniqueId::String() const
{
  std::ostringstream stream;
  stream << *this;
  return stream.str();
}

//////////////////////////////////////////////////
bool UniqueId::Valid() const
{
  return this->X() > 0 && this->Y() >= 0 && this->Z() > 0;
}

//////////////////////////////////////////////////
bool UniqueId::operator==(const UniqueId &_other) const
{
  return this->X() == _other.X() &&
         this->Y() == _other.Y() &&
         this->Z() == _other.Z();
}

//////////////////////////////////////////////////
bool UniqueId::operator!=(const UniqueId &_other) const
{
  return !(*this == _other);
}

//////////////////////////////////////////////////
UniqueId &UniqueId::operator=(const UniqueId &_other)
{
  this->SetX(_other.X());
  this->SetY(_other.Y());
  this->SetZ(_other.Z());
  return *this;
}
