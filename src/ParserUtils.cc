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

#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "ignition/rndf/Checkpoint.hh"
#include "ignition/rndf/Exit.hh"
#include "ignition/rndf/Lane.hh"
#include "ignition/rndf/ParserUtils.hh"
#include "ignition/rndf/UniqueId.hh"
#include "ignition/rndf/Waypoint.hh"

#ifdef _WIN32
  const auto& ignition_rndf_strtok = strtok_s;
  const auto& ignition_rndf_strdup = _strdup;
#else
  const auto& ignition_rndf_strtok = strtok_r;
  const auto& ignition_rndf_strdup = strdup;
#endif

namespace ignition
{
  namespace rndf
  {
    //////////////////////////////////////////////////
    void trimWhitespaces(std::string &_str)
    {
      // Remove comments.
      auto commentStart = _str.find("/*");
      if (commentStart != std::string::npos)
      {
        auto commentEnd = _str.rfind("*/");
        if (commentEnd != std::string::npos)
          _str.erase(commentStart, commentEnd - commentStart + 2);
      }

      // Remove consecutive whitespaces leaving only one.
      auto new_end = std::unique(_str.begin(), _str.end(),
        [](char lhs, char rhs)
        {
          return isspace(lhs) && isspace(rhs);
        });
      _str.erase(new_end, _str.end());

      // Remove leading and trailing whitespaces.
      if (!_str.empty() && isspace(_str.front()))
        _str.erase(0, 1);

      if (!_str.empty() && isspace(_str.back()))
        _str.pop_back();

      // Replace all spaces with ' '.
      std::replace_if(_str.begin(), _str.end(), ::isspace, ' ');
    }

    /////////////////////////////////////////////////
    std::vector<std::string> split(const std::string &_str,
        const std::string &_delim)
    {
      std::vector<std::string> tokens;
      char *saveptr;
      char *str = ignition_rndf_strdup(_str.c_str());

      auto token = ignition_rndf_strtok(str, _delim.c_str(), &saveptr);

      while (token)
      {
        tokens.push_back(token);
        token = ignition_rndf_strtok(nullptr, _delim.c_str(), &saveptr);
      }

      free(str);
      return tokens;
    }

    //////////////////////////////////////////////////
    void nextRealLine(std::ifstream &_rndfFile, std::string &_line,
      int &_lineNumber)
    {
      while (std::getline(_rndfFile, _line))
      {
        ++_lineNumber;

        trimWhitespaces(_line);

        // Ignore blank lines.
        if (!_line.empty())
          break;
      }
    }

    //////////////////////////////////////////////////
    bool parseString(std::ifstream &_rndfFile, const std::string &_delimiter,
      std::string &_value, int &_lineNumber)
    {
      std::string lineread;
      nextRealLine(_rndfFile, lineread, _lineNumber);

      auto tokens = split(lineread, " ");
      if (tokens.size() != 2                                     ||
          tokens.at(0) != _delimiter                             ||
          tokens.at(1).find_first_of("*\\") != std::string::npos ||
          tokens.at(1).size() > 128)
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse "
                  << _delimiter << " element" << std::endl;
        std::cerr << " \"" << lineread << "\"" << std::endl;
        return false;
      }

      _value = tokens.at(1);
      return true;
    }

    //////////////////////////////////////////////////
    bool parseDelimiter(std::ifstream &_rndfFile, const std::string &_delimiter,
      int &_lineNumber)
    {
      if (_rndfFile.eof())
        return false;

      std::string lineread;
      nextRealLine(_rndfFile, lineread, _lineNumber);

      if (lineread != _delimiter)
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse delimiter ["
                  << _delimiter << "]" << std::endl;
        std::cerr << " \"" << lineread << "\"" << std::endl;
        return false;
      }

      return true;
    }

    //////////////////////////////////////////////////
    bool parsePositive(std::ifstream &_rndfFile, const std::string &_delimiter,
      int &_value, int &_lineNumber)
    {
      std::string lineread;
      nextRealLine(_rndfFile, lineread, _lineNumber);

      auto start = lineread.find(_delimiter + " ");
      if (start != 0)
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse delimiter ["
                  << _delimiter << "]" << std::endl;
        std::cerr << " \"" << lineread << "\"" << std::endl;
        return false;
      }

      lineread.erase(0, _delimiter.size() + 1);

      std::string::size_type sz;
      try
      {
        _value = std::stoi(lineread, &sz);
      }
      catch(...)
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse "
                  << _value << "as a positive number" << std::endl;
        std::cerr << " \"" << lineread << "\"" << std::endl;
        return false;
      }

      if (_value <= 0 || _value > 32768 || sz != lineread.size())
      {
        std::cerr << "[Line " << _lineNumber << "]: Out of range value ["
                  << _value << "]" << std::endl;
        std::cerr << " \"" << lineread << "\"" << std::endl;
        return false;
      }

      return true;
    }

    //////////////////////////////////////////////////
    bool parseNonNegative(std::ifstream &_rndfFile,
      const std::string &_delimiter, int &_value, int &_lineNumber)
    {
      std::string lineread;
      nextRealLine(_rndfFile, lineread, _lineNumber);

      if (!parseNonNegative(lineread, _delimiter, _value))
      {
        std::cerr << "[Line " << _lineNumber << "]: Unable to parse "
                  << "non-negative value" << std::endl;
        std::cerr << " \"" << lineread << "\"" << std::endl;
        return false;
      }

      return true;
    }

    //////////////////////////////////////////////////
    bool parseNonNegative(const std::string &_input,
      const std::string &_delimiter, int &_value)
    {
      std::string input = _input;
      trimWhitespaces(input);

      auto start = input.find(_delimiter + " ");
      if (start != 0)
        return false;

      input.erase(0, _delimiter.size() + 1);

      std::string::size_type sz;
      try
      {
        _value = std::stoi(input, &sz);
      }
      catch(...)
      {
        return false;
      }

      if (_value < 0 || _value > 32768 || sz != input.size())
        return false;

      return true;
    }

    //////////////////////////////////////////////////
    bool parsePositive(const std::string &_input,
      const std::string &_delimiter, int &_value)
    {
      bool res = parseNonNegative(_input, _delimiter, _value);
      return res && _value > 0;
    }

    //////////////////////////////////////////////////
    bool parseBoundary(const std::string &_input, Marking &_boundary)
    {
      _boundary = Marking::UNDEFINED;
      std::string input = _input;
      trimWhitespaces(input);

      if (input == "left_boundary double_yellow" ||
          input == "right_boundary double_yellow")
      {
        _boundary = Marking::DOUBLE_YELLOW;
      }
      else if (input == "left_boundary solid_yellow" ||
               input == "right_boundary solid_yellow")
      {
        _boundary = Marking::SOLID_YELLOW;
      }
      else if (input == "left_boundary solid_white" ||
               input == "right_boundary solid_white")
      {
        _boundary = Marking::SOLID_WHITE;
      }
      else if (input == "left_boundary broken_white" ||
               input == "right_boundary broken_white")
      {
        _boundary = Marking::BROKEN_WHITE;
      }
      else
        return false;

      return true;
    }

    //////////////////////////////////////////////////
    bool parseCheckpoint(const std::string &_input, const int _segmentId,
      const int _laneId, Checkpoint &_checkpoint)
    {
      std::string input = _input;
      trimWhitespaces(input);

      auto tokens = split(input, " ");
      if (tokens.size() != 3)
        return false;

      if (tokens.at(0) != "checkpoint")
        return false;

      auto checkpointTokens = split(tokens.at(1), ".");
      if (checkpointTokens.size() != 3)
        return false;

      if (checkpointTokens.at(0) != std::to_string(_segmentId))
        return false;

      if (checkpointTokens.at(1) != std::to_string(_laneId))
        return false;

      std::string::size_type sz;
      int waypointId;
      try
      {
        waypointId = std::stoi(checkpointTokens.at(2), &sz);
      }
      catch(...)
      {
        return false;
      }

      if (waypointId <= 0 || waypointId > 32768 ||
          sz != checkpointTokens.at(2).size())
      {
        return false;
      }

      int checkpointId;
      try
      {
        checkpointId = std::stoi(tokens.at(2), &sz);
      }
      catch(...)
      {
        return false;
      }

      if (checkpointId <= 0 || checkpointId > 32768 ||
          sz != tokens.at(2).size())
      {
        return false;
      }

      _checkpoint.SetCheckpointId(checkpointId);
      _checkpoint.SetWaypointId(waypointId);
      return true;
    }

    //////////////////////////////////////////////////
    bool parseStop(const std::string &_input, const int _segmentId,
      const int _laneId, UniqueId &_stop)
    {
      std::string input = _input;
      trimWhitespaces(input);

      auto tokens = split(input, " ");
      if (tokens.size() != 2)
        return false;

      if (tokens.at(0) != "stop")
        return false;

      auto waypointTokens = split(tokens.at(1), ".");
      if (waypointTokens.size() != 3)
        return false;

      if (waypointTokens.at(0) != std::to_string(_segmentId))
        return false;

      if (waypointTokens.at(1) != std::to_string(_laneId))
        return false;

      std::string::size_type sz;
      int z;
      try
      {
        z = std::stoi(waypointTokens.at(2), &sz);
      }
      catch(...)
      {
        return false;
      }

      if (z <= 0 || z > 32768 || sz != waypointTokens.at(2).size())
        return false;

      _stop.SetX(_segmentId);
      _stop.SetY(_laneId);
      _stop.SetZ(z);
      return true;
    }

    //////////////////////////////////////////////////
    bool parseExit(const std::string &_input, const int _segmentId,
      const int _laneId, Exit &_exit)
    {
      std::string input = _input;
      trimWhitespaces(input);

      auto tokens = split(input, " ");
      if (tokens.size() != 3)
        return false;

      if (tokens.at(0) != "exit")
        return false;

      auto exitTokens = split(tokens.at(1), ".");
      if (exitTokens.size() != 3)
        return false;

      if (exitTokens.at(0) != std::to_string(_segmentId))
        return false;

      if (exitTokens.at(1) != std::to_string(_laneId))
        return false;

      std::string::size_type sz;
      int exitWaypointId;
      try
      {
        exitWaypointId = std::stoi(exitTokens.at(2), &sz);
      }
      catch(...)
      {
        return false;
      }

      if (exitWaypointId <= 0 || exitWaypointId > 32768 ||
          sz != exitTokens.at(2).size())
      {
        return false;
      }

      auto entryTokens = split(tokens.at(2), ".");
      if (entryTokens.size() != 3)
        return false;

      int x;
      try
      {
        x = std::stoi(entryTokens.at(0), &sz);
      }
      catch(...)
      {
        return false;
      }

      if (x <= 0 || x > 32768 || sz != entryTokens.at(0).size())
      {
        return false;
      }

      int y;
      try
      {
        y = std::stoi(entryTokens.at(1), &sz);
      }
      catch(...)
      {
        return false;
      }

      if (y < 0 || y > 32768 || sz != entryTokens.at(1).size())
      {
        return false;
      }

      int z;
      try
      {
        z = std::stoi(entryTokens.at(2), &sz);
      }
      catch(...)
      {
        return false;
      }

      if (z <= 0 || z > 32768 || sz != entryTokens.at(2).size())
      {
        return false;
      }

      UniqueId exitId(_segmentId, _laneId, exitWaypointId);
      UniqueId entryId(x, y, z);
      _exit.ExitId() = exitId;
      _exit.EntryId() = entryId;
      return true;
    }
  }
}
