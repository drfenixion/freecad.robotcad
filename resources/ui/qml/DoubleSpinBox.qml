import QtQuick 2.9
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.0

Rectangle{

    id: root
    property string textcolor:"white"
    property string backgroundColor: "black"
    property alias value: spinBox.realValue
    property real default_value:0.0
    property real min: 0
    property real max: 100
    property alias decimalPlaces: spinBox.decimals
    property alias suffix: spinBox.suffix
    property alias label: name.text
    signal spinBoxvalueChanged(real val)
    // Function to convert decimals to integers
    function decimalToInt(decimal) {
        return decimal * Math.pow(10, spinBox.decimals);
    }
    color: "transparent"
    height: name.height + 10
    width: layout.implicitWidth + 16

    RowLayout {
        id: layout
        anchors.fill: parent
        spacing: 8

        Text {
            id: name
            text: qsTr("Label")
            font.pointSize: 12
            color: root.textcolor
        }

        SpinBox {

            id: spinBox
            from: root.decimalToInt(root.min)
            value: root.decimalToInt(root.default_value)
            to: root.decimalToInt(root.max)
            stepSize: decimalFactor
            editable: true
            height: name.implicitHeight
            width: contentItem.paintedWidth + 20
            property string suffix
            property int decimals: 3
            property real realValue: value / decimalFactor
            readonly property int decimalFactor: Math.pow(10, decimals)

            contentItem: TextInput {
                id:ti
                z: 2
                text: spinBox.textFromValue(spinBox.value, spinBox.locale)
                font: spinBox.font
                color: root.textcolor
                selectionColor: "#21be2b"
                selectedTextColor: "#ffffff"
                horizontalAlignment: Qt.AlignHCenter
                verticalAlignment: Qt.AlignVCenter
                readOnly: !spinBox.editable
                validator: spinBox.validator
                inputMethodHints: Qt.ImhFormattedNumbersOnly
            }

            validator: DoubleValidator {
                bottom: Math.min(spinBox.from, spinBox.to)
                top: Math.max(spinBox.from, spinBox.to)
                decimals: spinBox.decimals
                notation: DoubleValidator.StandardNotation
            }

            textFromValue: function(value, locale) {
                return Number(value / decimalFactor).toLocaleString(locale, 'f', spinBox.decimals) + suffix;
            }

            valueFromText: function(text, locale) {
                let re = /\D*(-?\d*\.?\d*)\D*/;
                let match = re.exec(text);
                if (match && match[1] !== "") {
                    return Math.round(Number.fromLocaleString(locale, match[1]) * decimalFactor);
                }
                return spinBox.value;
            }

            background: Rectangle {
                color: root.backgroundColor
                radius: 4
                border.color: spinBox.activeFocus ? Qt.lighter("#404ec5"):"gray"
                border.width: 2
            }
            onValueModified:
            {
                root.spinBoxvalueChanged(realValue)
            }
        }
    }
}
