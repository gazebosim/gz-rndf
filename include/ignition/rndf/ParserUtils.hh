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

#ifndef IGNITION_RNDF_PARSERUTILS_HH_
#define IGNITION_RNDF_PARSERUTILS_HH_

#include <iosfwd>
#include <string>
#include <vector>

#include "ignition/rndf/Helpers.hh"

namespace ignition
{
  namespace rndf
  {
    // Forward declarations.
    class Checkpoint;
    class Exit;
    class UniqueId;
    enum class Marking;

    /// \brief An entry that captures an exit parsed.
    struct ExitCacheEntry
    {
      /// \brief The exit Id.
      public: std::string exitId;

      /// \brief The entry Id.
      public: std::string entryId;

      /// \brief The line number in which the exit was located.
      public: int lineNumber;

      /// \brief The entire line containing the exit.
      public: std::string line;
    };

    /// \brief Remove comments, consecutive whitespaces (leaving onle one) and
    /// leading and trailing whitespaces.
    /// \param[in] _str Input string.
    IGNITION_RNDF_VISIBLE
    void trimWhitespaces(std::string &_str);

    /// \brief Splits a string into tokens.
    /// \param[in] _str Input string.
    /// \param[in] _delim Token delimiter.
    /// \return Vector of tokens.
    IGNITION_RNDF_VISIBLE
    std::vector<std::string> split(const std::string &_str,
                                   const std::string &_delim);

    /// \brief Consumes lines from an input stream coming from a text file.
    /// The function reads line by line until it finds a line containing
    /// parsable content or EoF. Blank lines or lines with just a comment aren't
    /// considered parsable lines, so they will be consumed by this function.
    /// \param[in, out] _rndfFile Input file stream.
    /// \param[out] _line First line found with parsable content.
    /// \param[in, out] _lineNumber Line number pointed by the stream position
    /// indicator.
    IGNITION_RNDF_VISIBLE
    void nextRealLine(std::ifstream &_rndfFile,
                      std::string &_line,
                      int &_lineNumber);

    /// \brief Checks if the next parsable line from an input stream coming from
    /// a text file matches the following expression:
    /// "<DELIMITER> <STRING> [<COMMENT>]".
    /// <DELIMITER> is a string such as "RNDF_name".
    /// <STRING> is a sequence of characters with a maximum length of 128, and
    /// do not contain any spaces, backslashes or '*'.
    /// <COMMENT> is an optional element delimited by "/*" and "*/" and is
    /// always placed at the end of the line.
    /// \param[in, out] _rndfFile Input file stream.
    /// \param[in] _delimiter The <DELIMITER>.
    /// \param[out] _value The parsed <STRING>.
    /// \param[in, out] _lineNumber Line number pointed by the stream position
    /// indicator.
    /// \return True if the next parsable line matched the expression or false
    /// otherwise.
    IGNITION_RNDF_VISIBLE
    bool parseString(std::ifstream &_rndfFile,
                     const std::string &_delimiter,
                     std::string &_value,
                     int &_lineNumber);

    /// \brief Checks if the next parsable line from an input stream coming from
    /// a text file matches the following expression:
    /// "<DELIMITER> [<COMMENT>]".
    /// <DELIMITER> is a string such as "RNDF_name".
    /// <COMMENT> is an optional element delimited by "/*" and "*/" and is
    /// always placed at the end of the line.
    /// \param[in, out] _rndfFile Input file stream.
    /// \param[in] _delimiter The <DELIMITER>.
    /// \param[in, out] _lineNumber Line number pointed by the stream position
    /// indicator.
    /// \return True if the next parsable line matched the expression or false
    /// otherwise.
    IGNITION_RNDF_VISIBLE
    bool parseDelimiter(std::ifstream &_rndfFile,
                        const std::string &_delimiter,
                        int &_lineNumber);

    /// \brief Checks if the next parsable line from an input stream coming from
    /// a text file matches the following expression:
    /// "<DELIMITER> <POSITIVE> [<COMMENT>]".
    /// <DELIMITER> is a string such as "RNDF_name".
    /// <POSITIVE> is an integer value between [1, 32768].
    /// <COMMENT> is an optional element delimited by "/*" and "*/" and is
    /// always placed at the end of the line.
    /// \param[in, out] _rndfFile Input file stream.
    /// \param[in] _delimiter The <DELIMITER>.
    /// \param[out] _value The parsed <POSITIVE>.
    /// \param[in, out] _lineNumber Line number pointed by the stream position
    /// indicator.
    /// \return True if the next parsable line matched the expression or false
    /// otherwise.
    IGNITION_RNDF_VISIBLE
    bool parsePositive(std::ifstream &_rndfFile,
                       const std::string &_delimiter,
                       int &_value,
                       int &_lineNumber);

    /// \brief Checks if the next parsable line from an input stream coming from
    /// a text file matches the following expression:
    /// "<DELIMITER> <NON_NEGATIVE> [<COMMENT>]".
    /// <DELIMITER> is a string such as "RNDF_name".
    /// <NON_NEGATIVE> is an integer value between [0, 32768].
    /// <COMMENT> is an optional element delimited by "/*" and "*/" and is
    /// always placed at the end of the line.
    /// \param[in, out] _rndfFile Input file stream.
    /// \param[in] _delimiter The <DELIMITER>.
    /// \param[out] _value The parsed <NON_NEGATIVE>.
    /// \param[in, out] _lineNumber Line number pointed by the stream position
    /// indicator.
    /// \return True if the next parsable line matched the expression or false
    /// otherwise.
    IGNITION_RNDF_VISIBLE
    bool parseNonNegative(std::ifstream &_rndfFile,
                         const std::string &_delimiter,
                         int &_value,
                         int &_lineNumber);

    /// \brief Checks if a string matches the following expression:
    /// "<DELIMITER> <NON_NEGATIVE> [<COMMENT>]".
    /// <DELIMITER> is a string such as "RNDF_name".
    /// <NON_NEGATIVE> is an integer value between [0, 32768].
    /// <COMMENT> is an optional element delimited by "/*" and "*/" and is
    /// always placed at the end of the line.
    /// \param[in] _input Input string.
    /// \param[in] _delimiter The <DELIMITER>.
    /// \param[out] _value The parsed <NON_NEGATIVE>.
    /// \return True if the input string matched the expression or false
    /// otherwise.
    IGNITION_RNDF_VISIBLE
    bool parseNonNegative(const std::string &_input,
                          const std::string &_delimiter,
                          int &_value);

    /// \brief Checks if a string matches the following expression:
    /// "<DELIMITER> <POSITIVE> [<COMMENT>]".
    /// <DELIMITER> is a string such as "RNDF_name".
    /// <POSITIVE> is a positive value between [1, 32768].
    /// <COMMENT> is an optional element delimited by "/*" and "*/" and is
    /// always placed at the end of the line.
    /// \param[in] _input Input string.
    /// \param[in] _delimiter The <DELIMITER>.
    /// \param[out] _value The parsed <POSITIVE>.
    /// \return True if the input string matched the expression or false
    /// otherwise.
    IGNITION_RNDF_VISIBLE
    bool parsePositive(const std::string &_input,
                       const std::string &_delimiter,
                       int &_value);

    /// \brief Checks if a string matches the following expression:
    /// "left_boundary <BOUNDARY> [<COMMENT>]" or.
    /// "right_boundary <BOUNDARY> [<COMMENT>]".
    /// <BOUNDARY> is a string with one of the following values (double_yellow,
    /// solid_yellow, solid_white, broken_white).
    /// <COMMENT> is an optional element delimited by "/*" and "*/" and is
    /// always placed at the end of the line.
    /// \param[in] _input Input string.
    /// \param[out] _boundary The parsed <BOUNDARY>.
    /// \return True if the input string matched the expression or false
    /// otherwise.
    IGNITION_RNDF_VISIBLE
    bool parseBoundary(const std::string &_input,
                       Marking &_boundary);

    /// \brief Checks if a string matches the following expression:
    /// "checkpoint <WAYPOINT_ID> <CHECKPOINT_ID> [<COMMENT>]".
    /// <WAYPOINT_ID> is a "x.y.z" sequence, where 'x', 'y' and 'z' are positive
    /// numbers.
    /// <CHECKPOINT_ID> is a positive number.
    /// <COMMENT> is an optional element delimited by "/*" and "*/" and is
    /// always placed at the end of the line.
    /// \param[in] _input Input string.
    /// \param[in] _segmentId The expected segment Id (the "x").
    /// \param[in] _laneId The expected lane Id (the "y").
    /// \param[out] _checkpoint A Checkpoint object created parsing
    ///  <WAYPOINT_ID> and <CHECKPOINT_ID>.
    /// \return True if the input string matched the expression or false
    /// otherwise.
    IGNITION_RNDF_VISIBLE
    bool parseCheckpoint(const std::string &_input,
                         const int _segmentId,
                         const int _laneId,
                         Checkpoint &_checkpoint);

    /// \brief Checks if a string matches the following expression:
    /// "stop <WAYPOINT_ID> [<COMMENT>]".
    /// <WAYPOINT_ID> is a "x.y.z" sequence, where 'x', 'y' and 'z' are positive
    /// numbers.
    /// <COMMENT> is an optional element delimited by "/*" and "*/" and is
    /// always placed at the end of the line.
    /// \param[in] _input Input string.
    /// \param[in] _segmentId The expected segment Id (the "x").
    /// \param[in] _laneId The expected lane Id (the "y").
    /// \param[out] _stop An uniqueId object created parsing <WAYPOINT_ID>.
    /// \return True if the input string matched the expression or false
    /// otherwise.
    IGNITION_RNDF_VISIBLE
    bool parseStop(const std::string &_input,
                   const int _segmentId,
                   const int _laneId,
                   UniqueId &_stop);

    /// \brief Checks if a string matches the following expression:
    /// "exit <EXIT_WAYPOINT_ID> <ENTRY_WAYPOINT_ID> [<COMMENT>]" or.
    /// "exit <EXIT_WAYPOINT_ID> <ENTRY_PERIMETERPOINT_ID> [<COMMENT>]".
    /// <EXIT_WAYPOINT_ID> is a "x.y.z" sequence, where 'x', 'y' and 'z' are
    /// positive numbers.
    /// <ENTRY_WAYPOINT_ID> and <ENTRY_PERIMETERPOINT_ID> are a "x.y.z"
    /// sequence, where 'x', 'y' and 'z' are positive numbers.
    /// <COMMENT> is an optional element delimited by "/*" and "*/" and is
    /// always placed at the end of the line.
    /// \param[in] _input Input string.
    /// \param[in] _segmentId The expected segment Id (the "x").
    /// \param[in] _laneId The expected lane Id (the "y").
    /// \param[out] _exit An Exit object created parsing <EXIT_WAYPOINT_ID> and
    /// <ENTRY_WAYPOINT_ID> or <ENTRY_PERIMETERPOINT_ID>.
    /// \return True if the input string matched the expression or false
    /// otherwise.
    IGNITION_RNDF_VISIBLE
    bool parseExit(const std::string &_input,
                   const int _segmentId,
                   const int _laneId,
                   Exit &_exit);
  }
}
#endif
