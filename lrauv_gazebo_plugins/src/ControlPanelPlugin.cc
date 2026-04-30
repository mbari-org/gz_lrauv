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

#include "ControlPanelPlugin.hh"
#include <algorithm>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

#include <gz/gui/Application.hh>
#include <gz/gui/Conversions.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>

#include <QMetaObject>

namespace tethys
{

ControlPanel::ControlPanel() : gz::gui::Plugin()
{
  gz::gui::App()->Engine()->rootContext()->setContextProperty(
    "ControlPanel", this);

  // Start neutral - these values should match the defaults on the QML
  lastCommand.set_buoyancyaction_(500.0 / (100 * 100 * 100));
  lastCommand.set_dropweightstate_(1);
  this->SetVehicle("tethys");
}

ControlPanel::~ControlPanel()
{

}

void ControlPanel::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Tethys Control Panel";

  gz::gui::App()->findChild<
    gz::gui::MainWindow *>()->installEventFilter(this);
}

void ControlPanel::ReleaseDropWeight()
{
  gzdbg << "release dropweight\n";
  lastCommand.set_dropweightstate_(0);
  this->pub.Publish(lastCommand);
}

void ControlPanel::SetVehicle(QString _name)
{
  auto vehicle = _name.trimmed().toStdString();
  if (vehicle.empty())
  {
    gzerr << "Vehicle name cannot be empty" << std::endl;
    return;
  }

  this->currentVehicle = vehicle;

  gzdbg << "Setting name as " << this->currentVehicle <<"\n";
  this->pub = node.Advertise<lrauv_gazebo_plugins::msgs::LRAUVCommand>(
    "/" + this->currentVehicle + "/command_topic"
  );

  const auto newStateTopic = "/" + this->currentVehicle + "/state_topic";
  if (this->feedbackSubscribed && newStateTopic == this->stateTopic)
    return;

  this->stateTopic = newStateTopic;
  if (!this->node.Subscribe(this->stateTopic, &ControlPanel::StateCallback, this))
  {
    gzerr << "Failed subscribing to state topic [" << this->stateTopic
          << "]" << std::endl;
  }
  else
  {
    this->feedbackSubscribed = true;
  }
}

void ControlPanel::SetRudder(qreal _angle)
{
  const auto cmd = std::clamp(static_cast<double>(_angle),
      -kRudderLimit, kRudderLimit);
  gzdbg << "Setting rudder angle to " << cmd << "\n";
  lastCommand.set_rudderangleaction_(cmd);
  this->pub.Publish(lastCommand);
}

void ControlPanel::SetElevator(qreal _angle)
{
  const auto cmd = std::clamp(static_cast<double>(_angle),
      -kElevatorLimit, kElevatorLimit);
  gzdbg << "Setting elevator angle to " << cmd << "\n";
  lastCommand.set_elevatorangleaction_(cmd);
  this->pub.Publish(lastCommand);
}

void ControlPanel::SetPitchMass(qreal _massPosition)
{
  const auto cmd = std::clamp(static_cast<double>(_massPosition),
      kMassMin, kMassMax);
  gzdbg << "Setting mass position to " << cmd << "\n";
  lastCommand.set_masspositionaction_(cmd);
  this->pub.Publish(lastCommand);
}


void ControlPanel::SetThruster(qreal _thrust)
{
  const auto cmd = std::clamp(static_cast<double>(_thrust),
      -kThrusterLimit, kThrusterLimit);
  gzdbg << "Setting thruster angular velocity to " << cmd << "\n";
  lastCommand.set_propomegaaction_(cmd);
  this->pub.Publish(lastCommand);
}

void ControlPanel::SetBuoyancyEngine(qreal _volume)
{
  const auto cmd = std::clamp(static_cast<double>(_volume),
      kBuoyancyMin, kBuoyancyMax);
  gzdbg << "Setting buoyancy engine to " << cmd << "\n";
  lastCommand.set_buoyancyaction_(cmd);
  this->pub.Publish(lastCommand);
}

double ControlPanel::RudderFeedback() const
{
  return this->rudderFeedback;
}

double ControlPanel::ElevatorFeedback() const
{
  return this->elevatorFeedback;
}

double ControlPanel::MassFeedback() const
{
  return this->massFeedback;
}

double ControlPanel::ThrusterFeedback() const
{
  return this->thrusterFeedback;
}

double ControlPanel::BuoyancyFeedback() const
{
  return this->buoyancyFeedback;
}

void ControlPanel::StateCallback(
  const lrauv_gazebo_plugins::msgs::LRAUVState &_msg)
{
  const double rudder = _msg.rudderangle_();
  const double elevator = _msg.elevatorangle_();
  const double mass = _msg.massposition_();
  const double thruster = _msg.propomega_();
  const double buoyancy = _msg.buoyancyposition_();

  QMetaObject::invokeMethod(this,
    [this, rudder, elevator, mass, thruster, buoyancy]()
    {
      this->rudderFeedback = rudder;
      this->elevatorFeedback = elevator;
      this->massFeedback = mass;
      this->thrusterFeedback = thruster;
      this->buoyancyFeedback = buoyancy;
      emit this->FeedbackUpdated();
    },
    Qt::QueuedConnection);
}
}

// Register this plugin
GZ_ADD_PLUGIN(tethys::ControlPanel,
                    gz::gui::Plugin)
