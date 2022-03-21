/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

/*
 * Development of this module has been funded by the Monterey Bay Aquarium
 * Research Institute (MBARI) and the David and Lucile Packard Foundation
 */

#include <chrono>
#include <gtest/gtest.h>

#include "helper/LrauvTestFixture.hh"
#include "lrauv_command.pb.h"

#include <fstream>

/// \brief Loads the tethys "tethys_acoustic_comms.sdf" world.
/// This world has the robot start at a certain depth and 3 acoustic comms nodes
class LrauvTestCommsOrientation : public LrauvTestFixtureBase
{
  /// Documentation inherited
  protected: void SetUp() override
  {
    LrauvTestFixtureBase::SetUp("tethys_acoustic_comms.sdf");
  }
};

//////////////////////////////////////////////////
TEST_F(LrauvTestCommsOrientation, CheckCommsOrientation)
{

}