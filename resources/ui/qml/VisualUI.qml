import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

GroupBox {
    title: "Visual Properties"
    Layout.fillWidth: true

    ColumnLayout {
        spacing: 5

        TextField {
            id: visualName
            placeholderText: "Visual Name"
            Layout.fillWidth: true
        }

        CheckBox {
            id: castShadowsCheckBox
            text: "Cast Shadows"
            checked: true
        }

        Slider {
            id: transparencySlider
            from: 0.0
            to: 1.0
            value: 0.0
            stepSize: 0.1
            Layout.fillWidth: true
        }

        Text {
            text: "Transparency: " + transparencySlider.value.toFixed(2)
        }

        TextField {
            id: laserRetro
            placeholderText: "Laser Retro"
            validator: DoubleValidator {}
            Layout.fillWidth: true
        }

        TextField {
            id: visibilityFlags
            placeholderText: "Visibility Flags"
            validator: IntValidator { bottom: 0 }
            Layout.fillWidth: true
        }

        // Include Pose UI for visual pose
        PoseUI {
            Layout.fillWidth: true
        }
    }
}
