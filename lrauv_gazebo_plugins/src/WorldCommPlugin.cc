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

#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/spherical_coordinates.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/TopicUtils.hh>

#include "lrauv_gazebo_plugins/lrauv_init.pb.h"

#include "WorldCommPlugin.hh"

using namespace tethys;

/////////////////////////////////////////////////
void WorldCommPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &_eventMgr)
{
  // Parse SDF parameters
  if (_sdf->HasElement("spawn_topic"))
  {
    this->spawnTopic = _sdf->Get<std::string>("spawn_topic");
  }

  // Initialize transport
  if (!this->node.Subscribe(this->spawnTopic,
      &WorldCommPlugin::SpawnCallback, this))
  {
    gzerr << "Error subscribing to topic " << "[" << this->spawnTopic << "]. "
      << std::endl;
    return;
  }
  gzmsg << "Listening to spawn messages on [" << this->spawnTopic << "]"
         << std::endl;

  std::string worldName;
  auto worldEntity = gz::sim::worldEntity(_entity, _ecm);
  if (gz::sim::kNullEntity != worldEntity)
  {
    gz::sim::World world(worldEntity);
    auto worldNameOpt = world.Name(_ecm);
    if (worldNameOpt)
    {
      worldName = worldNameOpt.value();
    }
    else
    {
      gzerr << "Failed to get name for world entity [" << worldEntity
             << "]" << std::endl;
    }
  }
  else
  {
    gzerr << "Failed to get world entity" << std::endl;
  }

  if (worldName.empty())
  {
    gzerr << "Failed to initialize plugin." << std::endl;
    return;
  }

  // Valid world name for services
  auto topicWorldName =
      gz::transport::TopicUtils::AsValidTopic(worldName);
  if (topicWorldName.empty())
  {
    gzerr << "Invalid world name ["
           << worldName << "]" << std::endl;
    return;
  }

  // Services
  this->createService = "/world/" + topicWorldName + "/create";
  this->performerService = "/world/" + topicWorldName + "/level/set_performer";
  this->setSphericalCoordsService = "/world/" + topicWorldName
    + "/set_spherical_coordinates";

  // We assume that the world origin spherical coordinates will either be set
  // through SDF, or through this plugin. This assumption is broken if a user
  // sets it manually.
  this->hasWorldLatLon =
      gz::sim::sphericalCoordinates(worldEntity, _ecm).has_value();
}

/////////////////////////////////////////////////
void WorldCommPlugin::ServiceResponse(const gz::msgs::Boolean &_rep,
  const bool _result)
{
  if (!_result || !_rep.data())
    gzerr << "Error requesting some service." << std::endl;
}

/////////////////////////////////////////////////
void WorldCommPlugin::SpawnCallback(
  const lrauv_gazebo_plugins::msgs::LRAUVInit &_msg)
{
  gzdbg << "Received spawn message: " << std::endl
    << _msg.DebugString() << std::endl;

  if (!_msg.has_id_())
  {
    gzerr << "Received empty ID, can't initialize vehicle." << std::endl;
    return;
  }

  auto lat = _msg.initlat_();
  auto lon = _msg.initlon_();
  auto ele = -_msg.initz_();

  // Center the world around the first vehicle spawned
  if (!this->hasWorldLatLon)
  {
    gzdbg << "Setting world origin coordinates to latitude [" << lat
           << "], longitude [" << lon << "], elevation [" << ele << "]"
           << std::endl;

    // Set spherical coordinates
    gz::msgs::SphericalCoordinates scReq;
    scReq.set_surface_model(gz::msgs::SphericalCoordinates::EARTH_WGS84);
    scReq.set_latitude_deg(lat);
    scReq.set_longitude_deg(lon);
    scReq.set_elevation(ele);

    // Use zero heading so world is always aligned with lat / lon,
    // rotate vehicle instead.
    scReq.set_heading_deg(0.0);

    if (!this->node.Request(this->setSphericalCoordsService, scReq,
        &WorldCommPlugin::ServiceResponse, this))
    {
      gzerr << "Failed to request service [" << this->setSphericalCoordsService
             << "]" << std::endl;
    }
    else
    {
      this->hasWorldLatLon = true;
    }
  }

  // Create vehicle
  gz::msgs::EntityFactory factoryReq;
  factoryReq.set_sdf(this->TethysSdfString(_msg));

  auto coords = factoryReq.mutable_spherical_coordinates();
  coords->set_surface_model(gz::msgs::SphericalCoordinates::EARTH_WGS84);
  coords->set_latitude_deg(lat);
  coords->set_longitude_deg(lon);
  coords->set_elevation(ele);
  gz::msgs::Set(factoryReq.mutable_pose()->mutable_orientation(),
      gz::math::Quaterniond(
      _msg.initroll_(), _msg.initpitch_(), _msg.initheading_()));

  // RPH command is in NED
  // X == R: about N
  // Y == P: about E
  // Z == H: about D

  // Gazebo takes ENU
  // X == R: about E
  // Y == P: about N
  // Z == Y: about U

  auto rotENU = gz::math::Quaterniond::EulerToQuaternion(
      // East: NED's pitch
      _msg.initpitch_(),
      // North: NED's roll
      _msg.initroll_(),
      // Up: NED's -yaw
      -_msg.initheading_());

  // The robot model is facing its own -X, so with zero ENU orientation it faces
  // West. We add an extra 90 degree yaw so zero means North, to conform with
  // NED.
  auto rotRobot = gz::math::Quaterniond(0.0, 0.0, -GZ_PI * 0.5) * rotENU;

  gz::msgs::Set(factoryReq.mutable_pose()->mutable_orientation(), rotRobot);

  // TODO(chapulina) Check what's up with all the errors
  if (!this->node.Request(this->createService, factoryReq,
      &WorldCommPlugin::ServiceResponse, this))
  {
    gzerr << "Failed to request service [" << this->createService
           << "]" << std::endl;
  }
  else
  {
    // Make spawned model a performer
    gz::msgs::StringMsg performerReq;
    performerReq.set_data(_msg.id_().data());
    if (!this->node.Request(this->performerService, performerReq,
        &WorldCommPlugin::ServiceResponse, this))
    {
      gzerr << "Failed to request service [" << this->performerService
             << "]" << std::endl;
    }
  }
}

/////////////////////////////////////////////////
std::string WorldCommPlugin::TethysSdfString(const lrauv_gazebo_plugins::msgs::LRAUVInit &_msg)
{
  const std::string _id = _msg.id_().data();
  const std::string _acommsAddress = std::to_string(_msg.acommsaddress_());

  const std::string sdfStr = R"(
  <sdf version="1.9">
  <model name=")" + _id + R"(">
    <include merge="true">

      <!--
          Without any extra pose offset, the model is facing West.
          For the controller, zero orientation means the robot is facing North.
          So we need to rotate it.
          Note that this pose is expressed in ENU.
      <pose degrees="true">0 0 0  0 0 -90</pose>
      -->

      <!-- rename included model to avoid frame collisions -->
      <name>tethys_equipped</name>

      <uri>tethys_equipped</uri>

      <experimental:params>

        <sensor element_id="base_link::salinity_sensor" action="modify">
          <topic>/model/)" + _id + R"(/salinity</topic>
        </sensor>

        <sensor element_id="base_link::temperature_sensor" action="modify">
          <topic>/model/)" + _id + R"(/temperature</topic>
        </sensor>

        <sensor element_id="base_link::chlorophyll_sensor" action="modify">
          <topic>/model/)" + _id + R"(/chlorophyll</topic>
        </sensor>

        <sensor element_id="base_link::current_sensor" action="modify">
          <topic>/model/)" + _id + R"(/current</topic>
        </sensor>

        <sensor element_id="base_link::sparton_ahrs_m2_imu" action="modify">
          <topic>/)" + _id + R"(/ahrs/imu</topic>
        </sensor>

        <sensor element_id="base_link::sparton_ahrs_m2_magnetometer" action="modify">
          <topic>/)" + _id + R"(/ahrs/magnetometer</topic>
        </sensor>

        <sensor element_id="base_link::teledyne_pathfinder_dvl" action="modify">
          <topic>/)" + _id + R"(/dvl/velocity</topic>
        </sensor>

        <plugin element_id="gz::sim::systems::Thruster" action="modify">
          <namespace>)" + _id + R"(</namespace>
        </plugin>

        <plugin element_id="tethys::TethysCommPlugin" action="modify">
          <namespace>)" + _id + R"(</namespace>
          <command_topic>)" + _id + R"(/command_topic</command_topic>
          <state_topic>)" + _id + R"(/state_topic</state_topic>
        </plugin>

        <plugin element_id="gz::sim::systems::BuoyancyEngine" action="modify">
          <namespace>)" + _id + R"(</namespace>
        </plugin>

        <plugin element_id="gz::sim::systems::DetachableJoint" action="modify">
          <topic>/model/)" + _id + R"(/drop_weight</topic>
        </plugin>

        <plugin element_id="gz::sim::systems::CommsEndpoint" action="modify">
          <address>)" + _acommsAddress + R"(</address>
          <topic>)" + _acommsAddress + R"(/rx</topic>
        </plugin>

        <plugin element_id="tethys::RangeBearingPlugin" action="modify">
          <address>)" + _acommsAddress + R"(</address>
          <namespace>)" + _id + R"(</namespace>
        </plugin>

      </experimental:params>
    </include>
  </model>
  </sdf>)";

  return sdfStr;
}

GZ_ADD_PLUGIN(
  tethys::WorldCommPlugin,
  gz::sim::System,
  tethys::WorldCommPlugin::ISystemConfigure)
