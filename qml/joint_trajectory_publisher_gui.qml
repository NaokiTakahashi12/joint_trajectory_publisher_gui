import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Window {
    width: 480
    height: 640
    visible: true
    title: qsTr('Joint Trajectory Publisher GUI')
    Timer {
        id: spinTimer
        interval: 50
        repeat: true
        running: true
    }
    Connections {
        target: spinTimer
        function onTriggered() {
            nodeSpinner.spinSome()
        }
    }
    Shortcuts {}
}
