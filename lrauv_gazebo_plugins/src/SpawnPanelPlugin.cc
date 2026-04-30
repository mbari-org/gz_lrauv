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

#include "SpawnPanelPlugin.hh"
#include <cmath>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

#include <gz/gui/Application.hh>
#include <gz/gui/Conversions.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>

#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>

namespace tethys
{

SpawnPanel::SpawnPanel()
{
  gz::gui::App()->Engine()->rootContext()->setContextProperty(
    "SpawnPanel", this);
}

SpawnPanel::~SpawnPanel()
{

}

void SpawnPanel::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Spawn LRAUV Panel";

  if (_pluginElem)
  {
    auto initTopicElem = _pluginElem->FirstChildElement("init_topic");
    if (initTopicElem && initTopicElem->GetText())
      this->initTopic = initTopicElem->GetText();
  }

  this->pub = this->node.Advertise<lrauv_gazebo_plugins::msgs::LRAUVInit>(
      this->initTopic);
  if (!this->pub)
  {
    gzerr << "Unable to advertise spawn init topic [" << this->initTopic
          << "]." << std::endl;
  }

  gz::gui::App()->findChild<
    gz::gui::MainWindow *>()->installEventFilter(this);
}

void SpawnPanel::Spawn(
  double latitude, double longitude, double depth, int commsId, QString name,
  double heading, double pitch, double roll)
{
  if (!std::isfinite(latitude) || !std::isfinite(longitude) ||
      !std::isfinite(depth) || !std::isfinite(heading) ||
      !std::isfinite(pitch) || !std::isfinite(roll))
  {
    gzerr << "Spawn values must be finite numbers." << std::endl;
    return;
  }

  if (latitude < -90.0 || latitude > 90.0)
  {
    gzerr << "Latitude must be within [-90, 90] degrees." << std::endl;
    return;
  }

  if (longitude < -180.0 || longitude > 180.0)
  {
    gzerr << "Longitude must be within [-180, 180] degrees." << std::endl;
    return;
  }

  if (depth < 0.0)
  {
    gzerr << "Depth must be >= 0 meters (positive down)." << std::endl;
    return;
  }

  if (!this->pub.HasConnections())
  {
    gzerr << "No subscribers connected on spawn init topic ["
          << this->initTopic << "]." << std::endl;
    return;
  }

  if (this->acousticIds.count(commsId) > 0)
  {
    gzerr << "Comms ID [" << commsId << "] already exists.\n";
    return;
  }

  auto vehName = name.trimmed().toStdString();
  if (vehName.empty())
  {
    gzerr << "Model name cannot be empty." << std::endl;
    return;
  }

  if (this->modelNames.count(vehName) > 0)
  {
    gzerr << "Model name [" << vehName << "] already exists.\n";
    return;
  }

  lrauv_gazebo_plugins::msgs::LRAUVInit msg;
  msg.set_initlat_(latitude);
  msg.set_initlon_(longitude);
  msg.set_initz_(depth);
  msg.set_initpitch_(pitch);
  msg.set_initroll_(roll);
  msg.set_initheading_(heading);
  msg.set_acommsaddress_(commsId);
  msg.mutable_id_()->set_data(vehName);

  if (!this->pub.Publish(msg))
  {
    gzerr << "Failed to publish spawn init message on topic ["
          << this->initTopic << "]." << std::endl;
    return;
  }

  this->acousticIds.insert(commsId);
  this->modelNames.insert(vehName);
}

void SpawnPanel::Update(const gz::sim::UpdateInfo &,
  gz::sim::EntityComponentManager &_ecm)
{
}

}

// Register this plugin
GZ_ADD_PLUGIN(tethys::SpawnPanel,
                    gz::gui::Plugin)
