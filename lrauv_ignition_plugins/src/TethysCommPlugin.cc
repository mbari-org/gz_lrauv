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

#include <chrono>

#include <ignition/msgs/double.pb.h>
#include <ignition/msgs/header.pb.h>
#include <ignition/msgs/time.pb.h>
#include <ignition/msgs/vector3d.pb.h>
#include <ignition/plugin/Register.hh>

#include "lrauv_command.pb.h"
#include "lrauv_state.pb.h"

#include "TethysCommPlugin.hh"

using namespace tethys_comm_plugin;

void AddAngularVelocityComponent(
  const ignition::gazebo::Entity &_entity,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  if (!_ecm.Component<ignition::gazebo::components::AngularVelocity>(
      _entity))
  {
    _ecm.CreateComponent(_entity,
      ignition::gazebo::components::AngularVelocity());
  }
    // Create an angular velocity component if one is not present.
  if (!_ecm.Component<ignition::gazebo::components::WorldAngularVelocity>(
      _entity))
  {
    _ecm.CreateComponent(_entity,
      ignition::gazebo::components::WorldAngularVelocity());
  }
}

void AddWorldPose (
  const ignition::gazebo::Entity &_entity,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  if (!_ecm.Component<ignition::gazebo::components::WorldPose>(
      _entity))
  {
    _ecm.CreateComponent(_entity,
      ignition::gazebo::components::WorldPose());
  }
  // Create an angular velocity component if one is not present.
  if (!_ecm.Component<ignition::gazebo::components::WorldPose>(
      _entity))
  {
    _ecm.CreateComponent(_entity,
      ignition::gazebo::components::WorldPose());
  }
}

TethysCommPlugin::TethysCommPlugin()
{
}

void TethysCommPlugin::Configure(
  const ignition::gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  ignition::gazebo::EntityComponentManager &_ecm,
  ignition::gazebo::EventManager &_eventMgr)
{
  ignmsg << "TethysCommPlugin::Configure" << std::endl;

  // Parse SDF parameters
  if (_sdf->HasElement("command_topic"))
  {
    this->commandTopic = _sdf->Get<std::string>("command_topic");
  }
  if (_sdf->HasElement("state_topic"))
  {
    this->stateTopic = _sdf->Get<std::string>("state_topic");
  }

  // Initialize transport
  if (!this->node.Subscribe(this->commandTopic,
      &TethysCommPlugin::CommandCallback, this))
  {
    ignerr << "Error subscribing to topic " << "[" << commandTopic << "]. "
      << std::endl;
    return;
  }

  this->statePub =
    this->node.Advertise<lrauv_ignition_plugins::msgs::LRAUVState>(
    this->stateTopic);
  if (!this->statePub)
  {
    ignerr << "Error advertising topic [" << stateTopic << "]"
      << std::endl;
  }

  SetupControlTopics();
  SetupEntities(_entity, _sdf, _ecm, _eventMgr);

  this->prevPubPrintTime = std::chrono::steady_clock::duration::zero();
  this->prevSubPrintTime = std::chrono::steady_clock::duration::zero();
}

void TethysCommPlugin::SetupControlTopics()
{
  this->rudderPub =
    this->node.Advertise<ignition::msgs::Double>(this->rudderTopic);
  if (!this->rudderPub)
  {
    ignerr << "Error advertising topic [" << rudderTopic << "]"
      << std::endl;
  }

  this->elevatorPub =
    this->node.Advertise<ignition::msgs::Double>(this->elevatorTopic);
  if (!this->elevatorPub)
  {
    ignerr << "Error advertising topic [" << elevatorTopic << "]"
      << std::endl;
  }

  this->thrusterPub =
    this->node.Advertise<ignition::msgs::Double>(this->thrusterTopic);
  if(!this->thrusterPub)
  {
    ignerr << "Error advertising topic [" << thrusterTopic << "]"
      << std::endl;
  }
}

void TethysCommPlugin::SetupEntities( 
  const ignition::gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  ignition::gazebo::EntityComponentManager &_ecm,
  ignition::gazebo::EventManager &_eventMgr)
{
  if (_sdf->HasElement("model_link"))
  {
    this->baseLinkName = _sdf->Get<std::string>("model_link");
  }

  if (_sdf->HasElement("propeller_link"))
  {
    this->thrusterLinkName = _sdf->Get<std::string>("propeller_link");
  }

  if (_sdf->HasElement("rudder_link"))
  {
    this->elevatorLinkName = _sdf->Get<std::string>("rudder_link");
  }

  if (_sdf->HasElement("elavator_link"))
  {
    this->thrusterLinkName = _sdf->Get<std::string>("elavator_link");
  }

  auto model = ignition::gazebo::Model(_entity);
  
  this->modelLink = model.LinkByName(_ecm, this->baseLinkName);
  this->rudderLink = model.LinkByName(_ecm, this->rudderLinkName);
  this->elevatorLink = model.LinkByName(_ecm, this->elevatorLinkName);
  this->thrusterLink = model.LinkByName(_ecm, this->thrusterLinkName);

  AddAngularVelocityComponent(this->thrusterLink, _ecm);
  AddWorldPose(this->modelLink, _ecm);
  AddWorldPose(this->rudderLink, _ecm);
  AddWorldPose(this->elevatorLink, _ecm);
}

void TethysCommPlugin::CommandCallback(
  const lrauv_ignition_plugins::msgs::LRAUVCommand &_msg)
{
  // Lazy timestamp conversion just for printing
  if (std::chrono::seconds(int(floor(_msg.time_()))) - this->prevSubPrintTime > std::chrono::milliseconds(1000))
  {
    igndbg << "Received command: " << std::endl
      << "  propOmegaAction_: " << _msg.propomegaaction_() << std::endl
      << "  rudderAngleAction_: " << _msg.rudderangleaction_() << std::endl
      << "  elevatorAngleAction_: " << _msg.elevatorangleaction_() << std::endl
      << "  massPositionAction_: " << _msg.masspositionaction_() << std::endl
      << "  buoyancyAction_: " << _msg.buoyancyaction_() << std::endl
      << "  density_: " << _msg.density_() << std::endl
      << "  dt_: " << _msg.dt_() << std::endl
      << "  time_: " << _msg.time_() << std::endl;
  
    this->prevSubPrintTime = std::chrono::seconds(int(floor(_msg.time_())));
  }

  // Rudder
  ignition::msgs::Double rudderAngMsg;
  rudderAngMsg.set_data(_msg.rudderangleaction_());
  this->rudderPub.Publish(rudderAngMsg);

  // Elevator
  ignition::msgs::Double elevatorAngMsg;
  elevatorAngMsg.set_data(_msg.elevatorangleaction_());
  this->elevatorPub.Publish(elevatorAngMsg);

  // Thruster
  ignition::msgs::Double thrusterMsg;
  
  // TODO(arjo):
  // Conversion from rpm-> force b/c thruster plugin takes force
  // Maybe we should change that?
  auto angVel = _msg.propomegaaction_()/(60*2*M_PI);
  auto force = -7.879*1000*0.0016*angVel*angVel;
  if (angVel < 0)
    force *=-1;
  thrusterMsg.set_data(force);
  this->thrusterPub.Publish(thrusterMsg);
}

void TethysCommPlugin::PreUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  // ignmsg << "TethysCommPlugin::PreUpdate" << std::endl;
}

void TethysCommPlugin::PostUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  const ignition::gazebo::EntityComponentManager &_ecm)
{
  // ignmsg << "TethysCommPlugin::PostUpdate" << std::endl;

  ignition::gazebo::Link baseLink(modelLink);
  // TODO: where is worldPose defined?
  auto model_pose = worldPose(modelLink, _ecm);

  // Publish state
  lrauv_ignition_plugins::msgs::LRAUVState stateMsg;

  stateMsg.mutable_header()->mutable_stamp()->set_sec(
    std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count());
  stateMsg.mutable_header()->mutable_stamp()->set_nsec(
    int(std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count())
    - stateMsg.header().stamp().sec() * 1000000000);

  auto rph = model_pose.Rot().Euler();
  ignition::msgs::Set(stateMsg.mutable_rph_(), rph);
  stateMsg.set_depth_(-model_pose.Pos().Z());
  stateMsg.set_speed_(baseLink.WorldLinearVelocity(_ecm)->Length());

  // TODO(anyone)
  // Follow up https://github.com/ignitionrobotics/ign-gazebo/pull/519
  auto latlon = sphericalCoords.SphericalFromLocalPosition(model_pose.Pos());
  stateMsg.set_latitudedeg_(latlon.X());
  stateMsg.set_longitudedeg_(latlon.Y());

  ignition::gazebo::Link propLink(thrusterLink);
  auto prop_omega = propLink.WorldAngularVelocity(_ecm)->Length();
  stateMsg.set_propomega_(prop_omega);

  this->statePub.Publish(stateMsg);

  if (_info.simTime - this->prevPubPrintTime > std::chrono::milliseconds(1000))
  {
    igndbg << "Published state at time: " << stateMsg.header().stamp().sec()
      << "." << stateMsg.header().stamp().nsec() << std::endl;

    this->prevPubPrintTime = _info.simTime;
  }
}

IGNITION_ADD_PLUGIN(
  tethys_comm_plugin::TethysCommPlugin,
  ignition::gazebo::System,
  tethys_comm_plugin::TethysCommPlugin::ISystemConfigure,
  tethys_comm_plugin::TethysCommPlugin::ISystemPreUpdate,
  tethys_comm_plugin::TethysCommPlugin::ISystemPostUpdate)
