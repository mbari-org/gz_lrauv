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

import QtQuick 2.9
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import gz.gui 1.0

Flickable {
  id: mainView
  anchors.fill: parent
  Layout.minimumWidth: 400
  Layout.minimumHeight: 300
  clip: true
  flickableDirection: Flickable.VerticalFlick
  boundsBehavior: Flickable.StopAtBounds
  interactive: contentHeight > height
  contentWidth: mainLayout.width + 20
  contentHeight: mainLayout.implicitHeight + 12

  function syncFromFeedback() {
    if (!rudderControl.pressed && !rudderSpin.activeFocus)
      mainLayout.rudderValue = ControlPanel.rudderFeedback;

    if (!elevatorControl.pressed && !elevatorSpin.activeFocus)
      mainLayout.elevatorValue = ControlPanel.elevatorFeedback;

    if (!massControl.pressed && !massSpin.activeFocus)
      mainLayout.massValue = ControlPanel.massFeedback;

    if (!thrustControl.pressed && !thrusterSpin.activeFocus)
      mainLayout.thrusterValue = ControlPanel.thrusterFeedback;

    if (!buoyancyControl.pressed && !buoyancySpin.activeFocus)
    {
      var cc = ControlPanel.buoyancyFeedback * 1000000.0;
      mainLayout.buoyancyValue = Math.round(cc);
    }
  }

  Component.onCompleted: {
    syncFromFeedback();
  }

  Connections {
    target: ControlPanel
    onFeedbackUpdated: {
      mainView.syncFromFeedback();
    }
  }

  GridLayout {
    id: mainLayout
    columns: 5
    rowSpacing: 2
    columnSpacing: 2
    width: Math.max(mainView.width - 20, 380)
    height: implicitHeight
    x: 10

    // Rudder [rad]
    property double rudderValue: 0.0
    property double rudderMin: -0.26
    property double rudderMax: 0.26

    // Elevator [rad]
    property double elevatorValue: 0.0
    property double elevatorMin: -0.26
    property double elevatorMax: 0.26

    // Mass shifter [m]
    property double massValue: 0.0
    property double massMin: -0.03
    property double massMax: 0.03

    // Thruster [rad/s]
    property double thrusterValue: 0.0
    property double thrusterMin: -30.0
    property double thrusterMax: 30.0

    // Buoyancy [cc], mapped to m^3 in plugin command callback
    property double buoyancyValue: 500.0
    property double buoyancyMin: 80.0
    property double buoyancyMax: 955.0

    Label {
      text: "Vehicle name"
      Layout.columnSpan: 1
    }
    TextField {
      id: vehicleNameValue
      Layout.columnSpan: 4
      Layout.fillWidth: true
      text: "tethys"
      onEditingFinished: {
        ControlPanel.SetVehicle(text);
      }
    }

    Label {
      text: "Rudder (rad)"
      Layout.columnSpan: 1
    }
    GzSpinBox {
      id: rudderSpin
      Layout.columnSpan: 1
      value: mainLayout.rudderValue
      minimumValue: mainLayout.rudderMin
      maximumValue: mainLayout.rudderMax
      decimals: 2
      stepSize: 0.01
      onEditingFinished: {
        mainLayout.rudderValue = rudderSpin.value;
        ControlPanel.SetRudder(mainLayout.rudderValue);
      }
    }
    Label {
      text: mainLayout.rudderMin
      Layout.columnSpan: 1
      font.pixelSize: 9
      Layout.alignment: Qt.AlignRight
    }
    Slider {
      id: rudderControl
      Layout.columnSpan: 1
      Layout.fillWidth: true
      from: mainLayout.rudderMin
      to: mainLayout.rudderMax
      value: mainLayout.rudderValue
      onMoved: {
        mainLayout.rudderValue = Math.round(rudderControl.value * 100) / 100;
        ControlPanel.SetRudder(mainLayout.rudderValue);
      }
    }
    Label {
      text: mainLayout.rudderMax
      Layout.columnSpan: 1
      font.pixelSize: 9
    }

    Label {
      text: "Elevator (rad)"
      Layout.columnSpan: 1
    }
    GzSpinBox {
      id: elevatorSpin
      Layout.columnSpan: 1
      value: mainLayout.elevatorValue
      minimumValue: mainLayout.elevatorMin
      maximumValue: mainLayout.elevatorMax
      decimals: 2
      stepSize: 0.01
      onEditingFinished: {
        mainLayout.elevatorValue = elevatorSpin.value;
        ControlPanel.SetElevator(mainLayout.elevatorValue);
      }
    }
    Label {
      text: mainLayout.elevatorMin
      Layout.columnSpan: 1
      font.pixelSize: 9
      Layout.alignment: Qt.AlignRight
    }
    Slider {
      id: elevatorControl
      Layout.columnSpan: 1
      Layout.fillWidth: true
      from: mainLayout.elevatorMin
      to: mainLayout.elevatorMax
      value: mainLayout.elevatorValue
      onMoved: {
        mainLayout.elevatorValue = Math.round(elevatorControl.value * 100) / 100;
        ControlPanel.SetElevator(mainLayout.elevatorValue);
      }
    }
    Label {
      text: mainLayout.elevatorMax
      Layout.columnSpan: 1
      font.pixelSize: 9
    }

    Label {
      text: "Mass shifter (m)"
      Layout.columnSpan: 1
    }
    GzSpinBox {
      id: massSpin
      Layout.columnSpan: 1
      value: mainLayout.massValue
      minimumValue: mainLayout.massMin
      maximumValue: mainLayout.massMax
      decimals: 3
      stepSize: 0.001
      onEditingFinished: {
        mainLayout.massValue = massSpin.value;
        ControlPanel.SetPitchMass(mainLayout.massValue);
      }
    }
    Label {
      text: mainLayout.massMin
      Layout.columnSpan: 1
      font.pixelSize: 9
      Layout.alignment: Qt.AlignRight
    }
    Slider {
      id: massControl
      Layout.columnSpan: 1
      Layout.fillWidth: true
      from: mainLayout.massMin
      to: mainLayout.massMax
      value: mainLayout.massValue
      onMoved: {
        mainLayout.massValue = Math.round(massControl.value * 1000) / 1000;
        ControlPanel.SetPitchMass(mainLayout.massValue);
      }
    }
    Label {
      text: mainLayout.massMax
      Layout.columnSpan: 1
      font.pixelSize: 9
    }

    Label {
      text: "Thruster (rad/s)"
      Layout.columnSpan: 1
    }
    GzSpinBox {
      id: thrusterSpin
      Layout.columnSpan: 1
      value: mainLayout.thrusterValue
      minimumValue: mainLayout.thrusterMin
      maximumValue: mainLayout.thrusterMax
      decimals: 1
      stepSize: 1.0
      onEditingFinished: {
        mainLayout.thrusterValue = thrusterSpin.value;
        ControlPanel.SetThruster(mainLayout.thrusterValue);
      }
    }
    Label {
      text: mainLayout.thrusterMin
      Layout.columnSpan: 1
      font.pixelSize: 9
      Layout.alignment: Qt.AlignRight
    }
    Slider {
      id: thrustControl
      Layout.columnSpan: 1
      Layout.fillWidth: true
      from: mainLayout.thrusterMin
      to: mainLayout.thrusterMax
      value: mainLayout.thrusterValue
      onMoved: {
        mainLayout.thrusterValue = Math.round(thrustControl.value * 10) / 10;
        ControlPanel.SetThruster(mainLayout.thrusterValue);
      }
    }
    Label {
      text: mainLayout.thrusterMax
      Layout.columnSpan: 1
      font.pixelSize: 9
    }

    Label {
      text: "Buoyancy engine (cc)"
      Layout.columnSpan: 1
    }
    GzSpinBox {
      id: buoyancySpin
      Layout.columnSpan: 1
      value: mainLayout.buoyancyValue
      minimumValue: mainLayout.buoyancyMin
      maximumValue: mainLayout.buoyancyMax
      decimals: 0
      stepSize: 1.0
      onEditingFinished: {
        mainLayout.buoyancyValue = Math.round(buoyancySpin.value);
        var cmdM3 = mainLayout.buoyancyValue / 1000000.0;
        ControlPanel.SetBuoyancyEngine(cmdM3);
      }
    }
    Label {
      text: mainLayout.buoyancyMin
      Layout.columnSpan: 1
      font.pixelSize: 9
      Layout.alignment: Qt.AlignRight
    }
    Slider {
      id: buoyancyControl
      Layout.columnSpan: 1
      Layout.fillWidth: true
      from: mainLayout.buoyancyMin
      to: mainLayout.buoyancyMax
      value: mainLayout.buoyancyValue
      onMoved: {
        mainLayout.buoyancyValue = Math.round(buoyancyControl.value);
        var cmdM3 = mainLayout.buoyancyValue / 1000000.0;
        ControlPanel.SetBuoyancyEngine(cmdM3);
      }
    }
    Label {
      text: mainLayout.buoyancyMax
      Layout.columnSpan: 1
      font.pixelSize: 9
    }

    Label {
      text: "Drop weight"
      Layout.columnSpan: 1
    }
    Label {
      text: "Detached"
      Layout.columnSpan: 1
      font.pixelSize: 9
      Layout.alignment: Qt.AlignRight
    }
    Switch {
      id: dropWeightRelease
      Layout.columnSpan: 1
      Layout.fillWidth: true
      checked: true
      onToggled: {
        ControlPanel.ReleaseDropWeight();
        checkable = false;
      }
    }
    Label {
      text: "Attached"
      Layout.columnSpan: 2
      font.pixelSize: 9
    }

    Item {
      Layout.columnSpan: 5
      Layout.minimumHeight: 8
      width: 1
    }
  }
}
