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

#ifndef TETHYS_CONTROLPANEL_HH_
#define TETHYS_CONTROLPANEL_HH_

#include <string>
#include <unordered_set>

#include <gz/gui/Plugin.hh>

#include <gz/transport/Node.hh>

#include <gz/sim/gui/GuiSystem.hh>

#include "lrauv_gazebo_plugins/msgs/lrauv_init.pb.h"

namespace tethys
{

/// \brief Control Panel for controlling the tethys using the GUI.
class SpawnPanel : public gz::sim::GuiSystem
{
  Q_OBJECT

  /// \brief Constructor
  public: SpawnPanel();

  /// \brief Destructor
  public: ~SpawnPanel();

  /// \brief Documentation inherited
  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  /// \brief Publish an init message to spawn / initialize a vehicle.
  public: Q_INVOKABLE void Spawn(
    double latitude, double longitude, double depth, int commsId, QString name,
    double heading = 0.0, double pitch = 0.0, double roll = 0.0);

  /// \brief Documentation inherited
  public: void Update(const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager &_ecm);

  /// \brief Transport node
  private: gz::transport::Node node;

  /// \brief Transport publisher
  private: gz::transport::Node::Publisher pub;

  /// \brief Topic used by WorldCommPlugin for spawn init messages.
  private: std::string initTopic{"/lrauv/init"};

  /// \brief The names of all the models
  private: std::unordered_set<std::string> modelNames;

  /// \brief The acoustic address of the models
  private: std::unordered_set<int> acousticIds;
};

}

#endif
