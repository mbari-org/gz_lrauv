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
  Layout.minimumHeight: 420
  clip: true
  flickableDirection: Flickable.VerticalFlick
  boundsBehavior: Flickable.StopAtBounds
  interactive: contentHeight > height
  contentWidth: mainLayout.width + 20
  contentHeight: mainLayout.implicitHeight + 12

  GridLayout {
    id: mainLayout
    columns: 5
    rowSpacing: 2
    columnSpacing: 2
    width: Math.max(mainView.width - 20, 380)
    height: implicitHeight
    x: 10

    Label {
      text: "Vehicle name"
      Layout.columnSpan: 1
    }
    TextField {
      id: vehicleNameValue
      Layout.columnSpan: 4
      Layout.fillWidth: true
      text: "tethys"
    }

    Label {
      text: "Latitude (deg)"
      Layout.columnSpan: 1
    }
    TextField {
      id: latitudeValue
      Layout.columnSpan: 4
      Layout.fillWidth: true
      text: "36.693509"
    }

    Label {
      text: "Longitude (deg)"
      Layout.columnSpan: 1
    }
    TextField {
      id: longitudeValue
      Layout.columnSpan: 4
      Layout.fillWidth: true
      text: "-121.936568"
    }

    Label {
      text: "Depth (+down, m)"
      Layout.columnSpan: 1
    }
    TextField {
      id: depthValue
      Layout.columnSpan: 4
      Layout.fillWidth: true
      text: "0"
    }

    Label {
      text: "Comms ID"
      Layout.columnSpan: 1
    }
    GzSpinBox {
      id: commsValue
      Layout.columnSpan: 4
      value: 0
      minimumValue: 0
      maximumValue: 255
      decimals: 0
      stepSize: 1
    }

    Label {
      text: "Heading (rad)"
      Layout.columnSpan: 1
    }
    TextField {
      id: headingValue
      Layout.columnSpan: 4
      Layout.fillWidth: true
      text: "0"
    }

    Label {
      text: "Pitch (rad)"
      Layout.columnSpan: 1
    }
    TextField {
      id: pitchValue
      Layout.columnSpan: 4
      Layout.fillWidth: true
      text: "0"
    }

    Label {
      text: "Roll (rad)"
      Layout.columnSpan: 1
    }
    TextField {
      id: rollValue
      Layout.columnSpan: 4
      Layout.fillWidth: true
      text: "0"
    }

    Button {
      text: "Spawn!"
      Layout.columnSpan: 5
      onClicked: function() {
        SpawnPanel.Spawn(
          parseFloat(latitudeValue.text),
          parseFloat(longitudeValue.text),
          parseFloat(depthValue.text),
          parseInt(commsValue.value),
          vehicleNameValue.text,
          parseFloat(headingValue.text),
          parseFloat(pitchValue.text),
          parseFloat(rollValue.text)
        )
      }
    }

    Item {
      Layout.columnSpan: 5
      Layout.minimumHeight: 8
      width: 1
    }
  }
}
