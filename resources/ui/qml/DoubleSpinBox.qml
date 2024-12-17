import QtQuick 2.9
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.0
Rectangle{
    property alias value:spinBox.realValue
    property alias min:spinBox.from
    property alias max: spinBox.to
    property alias decimalPlaces:spinBox.decimals
    height:name.height+2
    width: 170+name.width
    property alias label:name.text
    RowLayout
    {
        id:layout
        anchors.fill: parent
        spacing:2
        Text {
            id: name
            text: qsTr("text")
            font.pixelSize:24
        }
        SpinBox {
            id: spinBox
            from: -100
            value: decimalToInt(1.1)
            to: decimalToInt(100)
            stepSize: decimalFactor
            editable: true
            height:name.implicitHeight

            // text appearance
            contentItem: TextInput {
                     z: 2
                     text: spinBox.textFromValue(spinBox.value, spinBox.locale)

                     font: spinBox.font
                     color: "black"
                     selectionColor: "#21be2b"
                     selectedTextColor: "#ffffff"
                     horizontalAlignment: Qt.AlignHCenter
                     verticalAlignment: Qt.AlignVCenter

                     readOnly: !spinBox.editable
                     validator: spinBox.validator
                     inputMethodHints: Qt.ImhFormattedNumbersOnly
                 }
            // end of text appearence

            //indicators
            property int decimals: 3
            property real realValue: value / decimalFactor
            readonly property int decimalFactor: Math.pow(10, decimals)

            function decimalToInt(decimal) {
                return decimal * decimalFactor
            }

            validator: DoubleValidator {
                bottom: Math.min(spinBox.from, spinBox.to)
                top:  Math.max(spinBox.from, spinBox.to)
                decimals: spinBox.decimals
                notation: DoubleValidator.StandardNotation
            }

            textFromValue: function(value, locale) {
                return Number(value / decimalFactor).toLocaleString(locale, 'f', spinBox.decimals)
            }

            valueFromText: function(text, locale) {
                return Math.round(Number.fromLocaleString(locale, text) * decimalFactor)
            }
            background:Rectangle {
                color:"white"
                radius: 4

            }
        }
    }
}

