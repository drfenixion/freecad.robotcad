import QtQuick 2.9
import QtQuick.Layouts 1.0

FocusScope {
    property alias label: label.text
    property alias text: ti.text
    implicitWidth: layout.implicitWidth
    implicitHeight: 30
    signal textvalChanged(string txt)

    id: root

    RowLayout {
        id: layout
        spacing: 45

        Text {
            id: label
            text: qsTr("text")
            height: ti.height
            font.pixelSize: ti.font.pixelSize
            color: plt.textColor
        }

        Rectangle {
            height: 30
            width: ti.implicitWidth < 70 ? 70 : ti.implicitWidth
            implicitWidth: ti.implicitWidth
            border.color: ti.activeFocus ? "#515dd0" : "#cccccc" // Change border color based on focus
            color: plt.textBackground
            radius: 4
            smooth: true

            TextInput {
                id: ti
                text: "none"
                color: plt.textColor
                height: parent.height
                font.pixelSize: 24
                focus: true

                onTextChanged: {
                    root.textvalChanged(text)
                }

                onActiveFocusChanged: {
                    if (!activeFocus && text === "") {
                        // If the TextInput loses focus and is empty, set focus back to it
                        ti.forceActiveFocus()
                    }
                }
            }
        }
    }
}
