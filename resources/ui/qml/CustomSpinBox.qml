import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: root
    width: layout.implicitWidth // Default width for the root item
    height: 40  // Default height for the root item

    // Custom properties
    property alias label: lbl.text  // Alias for the Label's text
    property string objectpropertyName: "none"   // Parameter for getter/setter
    property int minimumValue: 0  // Minimum value for SpinBox
    property int maximumValue: 100  // Maximum value for SpinBox
    // Layout to arrange Label and SpinBox horizontally
    RowLayout {
        id: layout
        anchors.fill: parent  // Fill the parent (root) item
        spacing: 7  // Spacing between Label and SpinBox

        // Label to display text
        Label {
            id: lbl
            text: "Default"  // Default text
            color: plt.textColor  // Text color from external property
            Layout.preferredWidth: 100  // Preferred width for the label
            Layout.alignment: Qt.AlignVCenter  // Center vertically
        }

        // SpinBox for numeric input
        SpinBox {
            id: spinBox
            from: minimumValue  // Set minimum value
            to: maximumValue  // Set maximum value
            stepSize: 1  // Step size for increment/decrement
            value: prop.getter(objectpropertyName)  // Get initial value from external property
            onValueModified: {
                prop.setter(objectpropertyName, value)  // Set value on modification
            }

            // Customize the text color of the SpinBox
            contentItem: TextInput {
                text: spinBox.textFromValue(spinBox.value, spinBox.locale)
                color: plt.textColor // Use custom text color
                font: spinBox.font  // Use the SpinBox's font
                horizontalAlignment: Qt.AlignHCenter  // Center text horizontally
                verticalAlignment: Qt.AlignVCenter  // Center text vertically
                readOnly: !spinBox.editable  // Make it read-only if SpinBox is not editable
                validator: spinBox.validator  // Use SpinBox's validator
                inputMethodHints: Qt.ImhFormattedNumbersOnly  // Restrict input to numbers
            }

            // Custom background for SpinBox
            background: Rectangle {
                implicitWidth: 150  // Default width for the SpinBox background
                implicitHeight: 30  // Default height for the SpinBox background
                color: plt.textBackground  // Background color from external property
                radius: 4  // Rounded corners
                border.color: spinBox.activeFocus ? Qt.lighter("#404ec5") : "gray"  // Border color based on focus
                border.width: 2  // Border width
            }

            // Layout properties for SpinBox
            Layout.fillWidth: true  // Allow SpinBox to expand horizontally
            Layout.alignment: Qt.AlignVCenter  // Center vertically
        }
    }
}
