import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Column {
    property alias value: jointTrajectorySlider.value
    Text {
        height: 20
        text: modelData.name
        font.bold: true
    }
    Text {
        height: 20
        text: jointTrajectorySlider.value
    }
    Slider {
        height: 20
        id: jointTrajectorySlider
        from: modelData.lower
        to: modelData.upper
    }
}
