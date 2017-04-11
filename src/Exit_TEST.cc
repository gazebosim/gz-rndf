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
#include <string>
#include <tuple>
#include <vector>

#include "gtest/gtest.h"
#include "ignition/rndf/test_config.h"
#include "ignition/rndf/Exit.hh"
#include "ignition/rndf/UniqueId.hh"

using namespace ignition;
using namespace rndf;

// The fixture for testing Exit class.
class ExitTest : public testing::FileParserUtils
{
};

//////////////////////////////////////////////////
/// \brief Check accessors.
TEST(Exit, accessors)
{
  // Test invalid exits.
  {
    Exit exit;
    EXPECT_FALSE(exit.Valid());
  }
  {
    Exit exit(UniqueId(0, 1, 2), UniqueId(1, 2, 3));
    EXPECT_FALSE(exit.Valid());
  }
  {
    Exit exit(UniqueId(1, 2, 3), UniqueId(0, 1, 2));
    EXPECT_FALSE(exit.Valid());
  }

  // Test valid exit.
  {
    UniqueId exitId(1, 2, 3);
    UniqueId entryId(4, 5, 6);
    Exit exit(exitId, entryId);
    EXPECT_TRUE(exit.Valid());
    EXPECT_EQ(exit.ExitId(), exitId);
    EXPECT_EQ(exit.EntryId(), entryId);
  }
}

//////////////////////////////////////////////////
/// \brief Check [in]equality operators.
TEST(Exit, equality)
{
  UniqueId exitId1(1, 2, 3);
  UniqueId entryId1(4, 5, 6);
  Exit exit1(exitId1, entryId1);

  UniqueId exitId2(10, 20, 30);
  UniqueId entryId2(40, 50, 60);
  Exit exit2(exitId2, entryId2);

  Exit exit3(exitId1, entryId1);

  EXPECT_FALSE(exit1 == exit2);
  EXPECT_TRUE(exit1 != exit2);

  EXPECT_FALSE(exit1 != exit3);
  EXPECT_TRUE(exit1 == exit3);
}

//////////////////////////////////////////////////
/// \brief Check assignment operator.
TEST(Exit, assignment)
{
  UniqueId exitId1(1, 2, 3);
  UniqueId entryId1(4, 5, 6);
  Exit exit1(exitId1, entryId1);

  UniqueId exitId2(10, 20, 30);
  UniqueId entryId2(40, 50, 60);
  Exit exit2(exitId2, entryId2);
  EXPECT_NE(exit1, exit2);

  exit2 = exit1;
  EXPECT_EQ(exit1, exit2);
}

//////////////////////////////////////////////////
/// \brief Check loading an exit from a file.
TEST_F(ExitTest, load)
{
  // The first element is the content to be parsed.
  // The second element is the expected return value.
  // The third element is the test Id.
  // The forth element is the expected line value.
  std::vector<std::tuple<std::string, bool, int, int>> testCases =
  {
    std::make_tuple(""                              , false, 0, 1),
    std::make_tuple("\n\n"                          , false, 0, 3),
    // Missing exit.
    std::make_tuple(
      "\n\n"
      "xxx  68.0.30 2.1.1\n"
                                                    , false, 0, 3),
    // Invalid x (x.y.z) in the exit Id.
    std::make_tuple(
      "\n\n"
      "exit  0.0.30 2.1.1\n"
                                                    , false, 0, 3),
    // Invalid y (x.y.z) in the exit Id.
    std::make_tuple(
      "\n\n"
      "exit  68.-1.30 2.1.1\n"
                                                    , false, 0, 3),
    // Invalid z (x.y.z) in the exit Id.
    std::make_tuple(
      "\n\n"
      "exit  68.0.0 2.1.1\n"
                                                    , false, 0, 3),
    // Missing x (x.y.z) in the exit Id.
    std::make_tuple(
      "\n\n"
      "exit  0.30 2.1.1\n"
                                                    , false, 0, 3),
    // Missing y (x.y.z) in the exit Id.
    std::make_tuple(
      "\n\n"
      "exit  68.30 2.1.1\n"
                                                    , false, 0, 3),
    // Missing z (x.y.z) in the exit Id.
    std::make_tuple(
      "\n\n"
      "exit  68.0 2.1.1\n"
                                                    , false, 0, 3),
    // Invalid x (x.y.z) in the entry Id.
    std::make_tuple(
      "\n\n"
      "exit  68.0.30 0.1.1\n"
                                                    , false, 0, 3),
    // Invalid y (x.y.z) in the entry Id.
    std::make_tuple(
      "\n\n"
      "exit  68.0.30 2.-1.1\n"
                                                    , false, 0, 3),
    // Invalid z (x.y.z) in the entry Id.
    std::make_tuple(
      "\n\n"
      "exit  68.0.30 2.1.0\n"
                                                    , false, 0, 3),
    // Missing x (x.y.z) in the entry Id.
    std::make_tuple(
      "\n\n"
      "exit  68.0.30 1.1\n"
                                                    , false, 0, 3),
    // Missing y/z (x.y.z) in the entry Id.
    std::make_tuple(
      "\n\n"
      "exit  68.0.30 2.1\n"
                                                    , false, 0, 3),
    // Unexpected x (x.y.z) value.
    std::make_tuple(
      "\n\n"
      "exit  99.0.30 2.1.1\n"
                                                    , false, 0, 3),
    // Unexpected y (x.y.z) value.
    std::make_tuple(
      "\n\n"
      "exit  68.99.30 2.1.1\n"
                                                    , false, 0, 3),
    // No comment on exit.
    std::make_tuple(
      "\n/* comment */\n"
      "exit  68.0.30 2.1.1\n"
                                                    , true, 1, 3),
    // Comment on exit.
    std::make_tuple(
      "\n\n"
      "exit  68.0.30   2.1.1/*comment    */ \n"
                                                    , true, 12, 3),
  };

  for (auto const &testCase : testCases)
  {
    int line = 0;
    std::string content = std::get<0>(testCase);
    int testId = std::get<2>(testCase);

    // Expectations.
    bool expectedResult = std::get<1>(testCase);
    int expectedLine = std::get<3>(testCase);

    // Write the content of this test case into the test file.
    this->PopulateFile(content);
    std::ifstream f(this->fileName);

    // Leave this comment for knowing wich test case failed if needed.
    std::cout << "Testing [" << content << "]" << std::endl;

    // Check expectations.
    Exit exit;
    bool res;
    EXPECT_EQ(res = exit.Load(f, 68, 0, line), expectedResult);
    EXPECT_EQ(line, expectedLine);
    if (res)
    {
      switch (testId)
      {
        case 1: case 2:
        {
          UniqueId exitId(68, 0, 30);
          UniqueId entryId(2, 1, 1);
          EXPECT_EQ(exit.ExitId(), exitId);
          EXPECT_EQ(exit.EntryId(), entryId);
          break;
        }
        default:
          break;
      };
    }
  }
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
