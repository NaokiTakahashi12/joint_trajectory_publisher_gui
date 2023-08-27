import QtQuick
import QtQuick.Controls

Item {
    Shortcut {
        sequences: ['Q']
        onActivated: {
            console.log('Quit shortcut activated')
            Qt.quit()
        }
    }
}
