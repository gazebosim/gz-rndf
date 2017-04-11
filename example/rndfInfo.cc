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

#include <ignition/rndf/RNDF.hh>

//////////////////////////////////////////////////
void usage()
{
  std::cerr << "Show some details of a RNDF file.\n\n"
            << " rndf_info <RNDF_file>\n\n"
            << std::endl;
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Sanity check.
  if (argc != 2)
  {
    usage();
    return -1;
  }

  // Parse the RNDF file.
  std::string fileName = argv[1];
  ignition::rndf::RNDF rndf(fileName);
  if (!rndf.Valid())
  {
    std::cerr << "File [" << fileName << "] is invalid" << std::endl;
    return -1;
  }

  // Show stats.
  std::cout << "Name:               [" << rndf.Name() << "]" << std::endl;
  if (!rndf.Version().empty())
    std::cout << "Version:            [" << rndf.Version() << "]" << std::endl;
  if (!rndf.Date().empty())
    std::cout << "Creation date:      [" << rndf.Date() << "]" << std::endl;
  std::cout << "Number of segments: " << rndf.NumSegments() << std::endl;
  std::cout << "Number of zones:    " << rndf.NumZones() << std::endl;
}
