import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Window {
    id: rootWindow
    width: 480
    height: 640
    visible: true
    title: qsTr('Joint Trajectory Publisher GUI')
    Column {
        id: rootColumn
        spacing: 10
        Row {
            spacing: rootColumn.spacing
            Text {
                text: 'robot_description name:'
            }
            Text {
                id: robotNameText
                text: 'Not recived robot name'
            }
        }
        Grid {
            columns: 2
            spacing: rootColumn.spacing
            Repeater {
                id: jointTrajectoryLabelRepeater
                JointTrajectory {}
            }
        }
        Switch {
            id: publishJointTrajectorySwitch
            text: 'Enabele publish joint trajectory'
        }
    }
    Timer {
        id: spinTimer
        interval: 50
        repeat: true
        running: true
    }
    Timer {
        id: publishTimer
        interval: 100
        repeat: true
        running: publishJointTrajectorySwitch.checked
    }
    Connections {
        target: spinTimer
        function onTriggered() {
            nodeSpinner.spinSome()
        }
    }
    Connections {
        target: rclNode
        function onRobotDescriptionUpdated(robotName) {
            robotNameText.text = robotName
        }
    }
    Connections {
        target: rclNode
        function onJointConfigurationChanged(jointConfigurations) {
            jointTrajectoryLabelRepeater.model = jointConfigurations
        }
    }
    Connections {
        target: publishTimer
        function onTriggered() {
            var destPositions = [jointTrajectoryLabelRepeater.model.length];
            for (var i = 0; i < jointTrajectoryLabelRepeater.model.length; i++) {
                destPositions[i] = jointTrajectoryLabelRepeater.itemAt(i).value;
            }
            rclNode.publishJointAngulerPositions(destPositions)
        }
    }
    Shortcuts {}
}
