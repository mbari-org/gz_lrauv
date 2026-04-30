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

#include <gz/gui/Plugin.hh>

#include <gz/transport/Node.hh>

#include "lrauv_gazebo_plugins/msgs/lrauv_command.pb.h"
#include "lrauv_gazebo_plugins/msgs/lrauv_state.pb.h"

namespace tethys
{

/// \brief Control Panel for controlling the tethys using the GUI.
class ControlPanel : public gz::gui::Plugin
{
  Q_OBJECT

  Q_PROPERTY(double rudderFeedback READ RudderFeedback NOTIFY FeedbackUpdated)
  Q_PROPERTY(double elevatorFeedback READ ElevatorFeedback NOTIFY FeedbackUpdated)
  Q_PROPERTY(double massFeedback READ MassFeedback NOTIFY FeedbackUpdated)
  Q_PROPERTY(double thrusterFeedback READ ThrusterFeedback NOTIFY FeedbackUpdated)
  Q_PROPERTY(double buoyancyFeedback READ BuoyancyFeedback NOTIFY FeedbackUpdated)

  /// \brief Constructor
  public: ControlPanel();

  /// \brief Destructor
  public: ~ControlPanel();

  /// \brief Documentation inherited
  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  /// \brief Releases the drop weight
  public: Q_INVOKABLE void ReleaseDropWeight();

  /// \brief Sets the vehicle name that you want to control
  /// \param[in] _name The name of the vehicle in question.
  public: Q_INVOKABLE void SetVehicle(QString _name);

  /// \brief Sets the rudder rotation
  /// \param[in] _rudderAngle The rudder angle set point, in radians
  public: Q_INVOKABLE void SetRudder(qreal _rudderAngle);

  /// \brief Sets the elevator rotation
  /// \param[in] _elevatorAngle The elevator angle set point, in radians
  public: Q_INVOKABLE void SetElevator(qreal _elevatorAngle);

  /// \brief Sets the mass shifter position
  /// \param[in] _pitchmassPosition The mass shifter position, in meters
  public: Q_INVOKABLE void SetPitchMass(qreal _pitchmassPosition);

  /// \brief Sets the thruster thrust
  /// \param[in] _rudderAngle The thruster angular velocity, in rad/s
  public: Q_INVOKABLE void SetThruster(qreal _thrust);

  /// \brief Sets the buoyancy engine
  /// \param[in] _volume The buoyancy engine's volume, in cubic meters
  public: Q_INVOKABLE void SetBuoyancyEngine(qreal _volume);

  /// \brief Rudder feedback from actuator state topic [rad].
  public: double RudderFeedback() const;

  /// \brief Elevator feedback from actuator state topic [rad].
  public: double ElevatorFeedback() const;

  /// \brief Mass shifter feedback from actuator state topic [m].
  public: double MassFeedback() const;

  /// \brief Thruster feedback from actuator state topic [rad/s].
  public: double ThrusterFeedback() const;

  /// \brief Buoyancy bladder feedback from actuator state topic [m^3].
  public: double BuoyancyFeedback() const;

  /// \brief Signal emitted when feedback values are updated.
  Q_SIGNALS: void FeedbackUpdated();

  /// \brief Callback for actuator state feedback.
  private: void StateCallback(const lrauv_gazebo_plugins::msgs::LRAUVState &_msg);

  /// \brief Transport node
  private: gz::transport::Node node;

  /// \brief Transport publisher
  private: gz::transport::Node::Publisher pub;

  /// \brief LRAUVCommand for the last state
  private: lrauv_gazebo_plugins::msgs::LRAUVCommand lastCommand;

  /// \brief Topic currently used for actuator feedback subscription.
  private: std::string stateTopic;

  /// \brief Track current vehicle namespace for topic derivation.
  private: std::string currentVehicle;

  /// \brief True once at least one feedback topic subscription exists.
  private: bool feedbackSubscribed{false};

  /// \brief Latest actuator feedback values.
  private: double rudderFeedback{0.0};
  private: double elevatorFeedback{0.0};
  private: double massFeedback{0.0};
  private: double thrusterFeedback{0.0};
  private: double buoyancyFeedback{0.0005};

  /// \brief Hard limits for GUI-issued actuator commands.
  private: static constexpr double kRudderLimit = 0.26;
  private: static constexpr double kElevatorLimit = 0.26;
  private: static constexpr double kMassMin = -0.03;
  private: static constexpr double kMassMax = 0.03;
  private: static constexpr double kThrusterLimit = 30.0;
  private: static constexpr double kBuoyancyMin = 80e-6;
  private: static constexpr double kBuoyancyMax = 955e-6;
};

}

#endif
