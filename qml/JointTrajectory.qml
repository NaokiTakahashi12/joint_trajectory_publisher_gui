import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    property string name
    RowLayout {
        Text {
            text: name
        }
        Slider {
            from: -1.57
            to: 1.57
        }
    }
}
